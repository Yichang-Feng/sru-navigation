"""RL Navigation Controller for Stair Descent Navigation.

Inherits from NavigationPolicyNode (rl_nav_controller_adaptive.py) and seamlessly integrates:
1. StairsSlopeGate: 3D path slope monitoring with spatial hysteresis to detect stair descent.
2. StairsDepthRemapper: Remaps the deep stairwell abyss into a progressive virtual ramp,
   guaranteeing 100% bitwise identity on flat ground and preserving real obstacle avoidance.
3. Strict Safety: Clamps lookahead distance to 1.2m on stairs, suppresses door latch/speed boost,
   and prevents false stall detection during slow terrain-mode stair descent.
"""

import os
import math
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from ament_index_python.packages import get_package_share_directory

try:
    from rl_nav_controller.rl_nav_controller_adaptive import NavigationPolicyNode, NavState
    from rl_nav_controller.stairs_depth_processor import StairsSlopeGate, StairsDepthRemapper
    from rl_nav_controller import constants
except ImportError:
    from rl_nav_controller_adaptive import NavigationPolicyNode, NavState
    from stairs_depth_processor import StairsSlopeGate, StairsDepthRemapper
    from rl_nav_controller import constants


class StairsNavigationPolicyNode(NavigationPolicyNode):
    """Extended RL Navigation Controller with 3D Path Slope Gating and Depth Remapping for Stairs."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Declare stairs-specific parameters
        self.declare_parameter('enable_stairs_mode', True)
        self.declare_parameter('enable_stair_remapping', False)      # Whether to remap depth image on stairs (default False: raw depth)
        self.declare_parameter('stair_slope_threshold', -0.20)       # dz / ds_2d (approx. >11.5° drop, allows advance detection)
        self.declare_parameter('stair_lookahead_eval_dist', 1.2)     # Evaluation window ahead (meters)
        self.declare_parameter('stair_exit_slope_threshold', -0.10)  # Level ground threshold when descending
        self.declare_parameter('stair_exit_dist_threshold', 0.6)     # Distance on flat ground before exit
        self.declare_parameter('stair_min_elevation_drop', 0.15)     # Minimum z drop (meters, single step)
        self.declare_parameter('stair_lookahead_dist', 0.40)          # Clamped lookahead on descending stairs (meters, default: 0.40m)
        self.declare_parameter('stair_up_lookahead_dist', 0.45)       # Clamped lookahead on ascending stairs (meters, default: 0.45m)
        self.declare_parameter('stair_up_slope_threshold', 0.20)     # dz / ds_2d for stair ascent (approx. >11.5° up, default: +0.20)
        self.declare_parameter('stair_up_min_elevation_gain', 0.15)  # Minimum z gain for stair ascent (meters, single step rise, default: 0.15m)
        self.declare_parameter('stair_up_exit_slope_threshold', 0.10)# Level ground threshold when ascending (default: +0.10)
        self.declare_parameter('stair_min_forward_vel', 0.0)          # Minimum forward velocity floor during stair descent (m/s, default 0.0: no hard override)
        self.declare_parameter('stair_ramp_base', 1.0)               # Bottom row virtual depth (meters)
        self.declare_parameter('stair_ramp_step', 0.046)             # Increment per row upwards
        self.declare_parameter('stair_abyss_threshold', 2.2)         # Depth above which pixels are abyss
        self.declare_parameter('stair_side_min_clamp', 0.50)         # Railing column depth clamp (meters)
        self.declare_parameter('stair_left_col', 16)                 # Central walking corridor left boundary
        self.declare_parameter('stair_right_col', 48)                # Central walking corridor right boundary

        self.enable_stairs_mode = bool(self.get_parameter('enable_stairs_mode').value)
        self.enable_stair_remapping = bool(self.get_parameter('enable_stair_remapping').value)
        self.stair_lookahead_dist = float(self.get_parameter('stair_lookahead_dist').value)
        self.stair_up_lookahead_dist = float(self.get_parameter('stair_up_lookahead_dist').value)
        self.stair_min_forward_vel = float(self.get_parameter('stair_min_forward_vel').value)

        # Ensure segment_arrive_threshold is realistically tuned for quadruped stair/terrain navigation (default 0.45m)
        if hasattr(self, 'arrive_goal_threshold') and self.arrive_goal_threshold is not None:
            if getattr(self, 'segment_arrive_threshold', 0.15) <= 0.15:
                self.segment_arrive_threshold = self.arrive_goal_threshold

        # Initialize StairsSlopeGate
        self.slope_gate = StairsSlopeGate(
            slope_threshold=float(self.get_parameter('stair_slope_threshold').value),
            lookahead_eval_dist=float(self.get_parameter('stair_lookahead_eval_dist').value),
            exit_slope_threshold=float(self.get_parameter('stair_exit_slope_threshold').value),
            exit_dist_threshold=float(self.get_parameter('stair_exit_dist_threshold').value),
            min_elevation_drop=float(self.get_parameter('stair_min_elevation_drop').value),
            up_slope_threshold=float(self.get_parameter('stair_up_slope_threshold').value),
            min_elevation_gain=float(self.get_parameter('stair_up_min_elevation_gain').value),
            exit_up_slope_threshold=float(self.get_parameter('stair_up_exit_slope_threshold').value)
        )

        # Initialize StairsDepthRemapper
        self.depth_remapper = StairsDepthRemapper(
            ramp_base=float(self.get_parameter('stair_ramp_base').value),
            ramp_step=float(self.get_parameter('stair_ramp_step').value),
            abyss_threshold=float(self.get_parameter('stair_abyss_threshold').value),
            side_min_clamp=float(self.get_parameter('stair_side_min_clamp').value),
            left_col=int(self.get_parameter('stair_left_col').value),
            right_col=int(self.get_parameter('stair_right_col').value),
            min_row=12,
            max_row=39,
            blindzone_val=self.min_depth
        )

        self._prev_stair_state = False
        self._prev_stair_up_state = False

        # Diagnostic publishers
        self.is_stair_pub = self.create_publisher(Bool, '/nav/is_descending_stairs', 10)
        self.is_stair_up_pub = self.create_publisher(Bool, '/nav/is_ascending_stairs', 10)
        self.remapped_depth_pub = self.create_publisher(Image, '/nav/remapped_depth', 1)

        self.get_logger().info(
            '\033[92m' +
            f'[StairsNav] Initialized Stairs-Aware Controller: enable={self.enable_stairs_mode}, '
            f'stair_down_L={self.stair_lookahead_dist}m, stair_up_L={self.stair_up_lookahead_dist}m, '
            f'remapping={self.enable_stair_remapping}, '
            f'down_slope_thresh={self.slope_gate.slope_threshold:.2f}, '
            f'up_slope_thresh={self.slope_gate.up_slope_threshold:.2f}, '
            f'side_clamp={self.depth_remapper.side_min_clamp:.2f}m' +
            '\033[0m'
        )

    def global_path_callback(self, path_msg: Path):
        """Reset stair slope gate on new path arrival and delegate to parent."""
        self.slope_gate.reset()
        self._prev_stair_state = False
        self._prev_stair_up_state = False
        super().global_path_callback(path_msg)

    def generate_cmd_vel(self):
        """Generate velocity commands with 3D stair gating and depth remapping."""
        # 0) Visualize moving goal marker
        self.visualization_manager.publish_moving_goal_marker(
            self.moving_goal_delta, self.robot_pos_w, self.robot_orientation_w,
            self.robot_odom_time, self.map_frame_id, self.moving_goal_marker_publisher
        )

        if self.smart_joystick_mode_active:
            self.get_logger().info(
                f'Smart joystick mode active - Goal: [{self.smart_joystick_goal[0]:.2f}, '
                f'{self.smart_joystick_goal[1]:.2f}, {self.smart_joystick_goal[2]:.2f}]'
            )

        # 1) Check blocked / stopped / error states
        if self.state in (NavState.ERROR, NavState.FAILED, NavState.STOPPED):
            self._stop_robot()
            # If in STOPPED state and path is present, keep blocked_detector window refreshed
            # so pausing does not accumulate stale stationary time into stall timeout
            if self.state == NavState.STOPPED and hasattr(self, 'blocked_detector') and hasattr(self, 'path_tracker') and self.path_tracker.has_path():
                self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)
            return

        if self.is_abort_goal:
            self.get_logger().warn('Goal aborted, stopping robot.')
            self._stop_robot()
            self.target_pos_w = None
            self.target_yaw_w = None
            self.path_tracker.clear()
            self.is_abort_goal = False
            self.set_state(NavState.STOPPED, reason="Goal aborted")
            return

        pos_for_tracking = None
        ori_for_tracking = None
        is_stair_down = False
        is_stair_up = False
        gate_info = {}

        # 2) Path Tracking Mode
        if self.path_tracker.is_active():
            frame_id = getattr(self.path_tracker, 'frame_id', 'map')
            pos_in_frame, ori_in_frame = self.get_robot_pose_in_frame(frame_id)
            pos_for_tracking = pos_in_frame if pos_in_frame is not None else self.robot_pos_w
            ori_for_tracking = ori_in_frame if ori_in_frame is not None else self.robot_orientation_w

            if pos_for_tracking is None:
                self._stop_robot()
                return

            robot_xy = pos_for_tracking[:2]
            progress_s = self.path_tracker.update_progress(robot_xy)

            # Check if reached terminal goal of current segment
            if self.path_tracker.check_arrived(robot_xy, arrive_threshold=self.segment_arrive_threshold):
                self._stop_robot()
                self.path_tracker.clear()
                self.target_pos_w = None
                self.target_yaw_w = None
                self.last_action = self._reset_last_action()
                self.slope_gate.reset()
                self.set_state(NavState.GOAL_REACHED, reason=f"Terminal goal reached (< {self.segment_arrive_threshold}m)")
                self.get_logger().info(
                    '\033[92m' +
                    f'[Path] Arrived at segment terminal goal! Actively braked (/cmd_vel=0), state -> GOAL_REACHED.' +
                    '\033[0m'
                )
                self.set_state(NavState.IDLE, reason="Goal reached reported, immediate transition to IDLE")
                return

            # --- STAIRS SLOPE GATE EVALUATION ---
            if self.enable_stairs_mode:
                is_stair_down, is_stair_up, gate_info = self.slope_gate.evaluate(self.path_tracker, current_progress=progress_s)
                
                # Publish diagnostic booleans
                stair_down_msg = Bool()
                stair_down_msg.data = bool(is_stair_down)
                self.is_stair_pub.publish(stair_down_msg)

                stair_up_msg = Bool()
                stair_up_msg.data = bool(is_stair_up)
                self.is_stair_up_pub.publish(stair_up_msg)

                # Transition logging for descent
                if is_stair_down != self._prev_stair_state:
                    if is_stair_down:
                        self.get_logger().info(
                            '\033[96m' +
                            f'[StairsNav] >>> ENTERING STAIR DESCENT <<< Slope={gate_info.get("slope", 0.0):.2f}, '
                            f'dz={gate_info.get("dz", 0.0):.2f}m. Latching lookahead L={self.stair_lookahead_dist}m, '
                            f'remapping={self.enable_stair_remapping}.' +
                            '\033[0m'
                        )
                    else:
                        self.get_logger().info(
                            '\033[96m' +
                            f'[StairsNav] <<< EXITING STAIR DESCENT >>> Path returned to flat ground. Resuming standard lookahead.' +
                            '\033[0m'
                        )
                    self._prev_stair_state = is_stair_down

                # Transition logging for ascent
                if is_stair_up != self._prev_stair_up_state:
                    if is_stair_up:
                        self.get_logger().info(
                            '\033[96m' +
                            f'[StairsNav] >>> ENTERING STAIR ASCENT <<< Slope={gate_info.get("slope", 0.0):.2f}, '
                            f'dz={gate_info.get("dz", 0.0):.2f}m. Latching lookahead L={self.stair_up_lookahead_dist}m.' +
                            '\033[0m'
                        )
                    else:
                        self.get_logger().info(
                            '\033[96m' +
                            f'[StairsNav] <<< EXITING STAIR ASCENT >>> Path returned to flat ground. Resuming standard lookahead.' +
                            '\033[0m'
                        )
                    self._prev_stair_up_state = is_stair_up

            # Blocked / stall check
            if is_stair_down or is_stair_up:
                # In terrain mode on stairs, walking velocity is slow. Keep blocked detector from false abort.
                self.blocked_detector.reset(self.robot_odom_time, progress_s)
            else:
                robot_vel_xy = self.linear_vel[:2] if self.linear_vel else [0.0, 0.0]
                if self.blocked_detector.update(self.robot_odom_time, progress_s, robot_vel_xy):
                    if self.report_failure:
                        self._stop_robot()
                        self.set_state(
                            NavState.FAILED,
                            reason=f"Stalled: progress < {self.blocked_dist_threshold}m and vel < {self.blocked_vel_threshold}m/s for {self.blocked_timeout}s"
                        )
                        self.get_logger().warn(
                            '\033[91m' +
                            f'[Path] Robot is BLOCKED / STALLED! Stopped moving, state -> FAILED.' +
                            '\033[0m'
                        )
                        return
                    else:
                        self.get_logger().info(
                            f'[Path] Robot currently slow/stalled near obstacle ({self.blocked_timeout:.1f}s), continuing...',
                            throttle_duration_sec=3.0
                        )

            # Carrot point extraction
            if is_stair_down:
                # Suppress narrow passage latching so lookahead does not collapse to 0.5m
                self.path_tracker.is_in_narrow_passage = False
                carrot_pt, carrot_yaw = self.path_tracker.get_lookahead_point(lookahead_dist=self.stair_lookahead_dist)

                # Defensive Safety Guard: Validate that carrot point does not lie behind robot
                # On stairs, backward carrot points induce violent spins at stair edges.
                # Only active when not near the terminal goal (dist_to_term > 0.60m)
                # to allow the carrot to naturally converge and stop at the final destination.
                curr_robot_pos = pos_for_tracking if (pos_for_tracking is not None) else self.robot_pos_w
                curr_robot_ori = ori_for_tracking if (ori_for_tracking is not None) else self.robot_orientation_w
                dist_to_term = 999.0
                if curr_robot_pos is not None and hasattr(self.path_tracker, 'get_distance_to_terminal'):
                    dist_to_term = float(self.path_tracker.get_distance_to_terminal(curr_robot_pos[:2]))

                if dist_to_term > max(0.60, self.segment_arrive_threshold + 0.15) and carrot_pt is not None and curr_robot_pos is not None and curr_robot_ori is not None:
                    _, target_vec_b = self.model.normalize_target_position(
                        carrot_pt, curr_robot_pos, curr_robot_ori
                    )
                    dx_b = float(target_vec_b[0, 0])
                    if dx_b < -0.15:
                        qw, qx, qy, qz = (
                            float(curr_robot_ori[0]), float(curr_robot_ori[1]),
                            float(curr_robot_ori[2]), float(curr_robot_ori[3])
                        )
                        robot_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                        forward_L = self.stair_lookahead_dist
                        slope = gate_info.get("slope", -0.35)
                        drop_z = float(min(-0.25, slope * forward_L))
                        # Prefer path tangent yaw to actively point down the stairs, fallback to robot_yaw
                        guide_yaw = carrot_yaw if carrot_yaw is not None else robot_yaw
                        clamped_x = float(curr_robot_pos[0] + forward_L * math.cos(guide_yaw))
                        clamped_y = float(curr_robot_pos[1] + forward_L * math.sin(guide_yaw))
                        clamped_z = float(curr_robot_pos[2] + drop_z)

                        self.get_logger().warn(
                            f'\033[91m[StairsNav][SAFETY GUARD] Backward carrot on stairs detected! '
                            f'(dx_b={dx_b:.2f}m < -0.15m, dist_term={dist_to_term:.2f}m). Clamped target down stairs along path '
                            f'to [{clamped_x:.2f}, {clamped_y:.2f}, {clamped_z:.2f}], dz={drop_z:.2f}m.\033[0m',
                            throttle_duration_sec=1.0
                        )
                        carrot_pt = [clamped_x, clamped_y, clamped_z]
                        carrot_yaw = guide_yaw

                self.target_pos_w = carrot_pt
                self.target_yaw_w = carrot_yaw
                z_drop = carrot_pt[2] - pos_for_tracking[2]
                self.get_logger().info(
                    f'[StairsNav][DESCENT] L={self.stair_lookahead_dist:.2f}m, slope={gate_info.get("slope", 0.0):.2f}, '
                    f'dz={gate_info.get("dz", 0.0):.2f}m, carrot_z_drop={z_drop:.2f}m',
                    throttle_duration_sec=1.0
                )
            elif is_stair_up:
                # Suppress narrow passage latching so lookahead does not collapse to 0.5m or expand to 2.0m
                self.path_tracker.is_in_narrow_passage = False
                carrot_pt, carrot_yaw = self.path_tracker.get_lookahead_point(lookahead_dist=self.stair_up_lookahead_dist)
                self.target_pos_w = carrot_pt
                self.target_yaw_w = carrot_yaw
                z_rise = carrot_pt[2] - pos_for_tracking[2] if carrot_pt is not None and pos_for_tracking is not None else 0.0
                self.get_logger().info(
                    f'[StairsNav][ASCENT] L={self.stair_up_lookahead_dist:.2f}m, slope={gate_info.get("slope", 0.0):.2f}, '
                    f'dz={gate_info.get("dz", 0.0):.2f}m, carrot_z_rise={z_rise:.2f}m',
                    throttle_duration_sec=1.0
                )
            elif self.enable_adaptive_lookahead and hasattr(self.path_tracker, 'get_adaptive_lookahead_point'):
                carrot_pt, carrot_yaw, debug_info = self.path_tracker.get_adaptive_lookahead_point(
                    current_time=self.robot_odom_time,
                    depth_image=self.depth_image
                )
                self.target_pos_w = carrot_pt
                self.target_yaw_w = carrot_yaw
                self.get_logger().info(
                    f'[AdaptiveLookahead][{debug_info.get("mode", "both")}] L={debug_info.get("lookahead", 0.0):.2f}m '
                    f'(curv={debug_info.get("l_curv", 0.0):.2f}m, clear={debug_info.get("l_clear", 0.0):.2f}m, '
                    f'turn={debug_info.get("turn_deg", 0.0):.1f}°, narrow={debug_info.get("in_narrow", False)})',
                    throttle_duration_sec=1.0
                )
            else:
                carrot_pt, carrot_yaw = self.path_tracker.get_lookahead_point(lookahead_dist=self.lookahead_distance)
                self.target_pos_w = carrot_pt
                self.target_yaw_w = carrot_yaw

            # Publish lookahead marker for RViz
            self.visualization_manager.publish_lookahead_goal_marker(
                self.target_pos_w, self.robot_odom_time, self.map_frame_id, self.lookahead_marker_publisher
            )

        elif self.target_pos_w is None:
            self._stop_robot()
            if self.state in (NavState.RUNNING, NavState.GOAL_REACHED):
                self.set_state(NavState.IDLE, reason="No active target or path")
            return
        else:
            # Single goal tracking mode
            is_arrived = self._check_goal_reached(
                self.target_pos_w, self.robot_pos_w, self.target_yaw_w, self.robot_orientation_w
            )
            if is_arrived:
                self._stop_robot()
                self.target_pos_w = None
                self.target_yaw_w = None
                self.last_action = self._reset_last_action()
                self.set_state(NavState.GOAL_REACHED, reason="Single goal reached")
                self.get_logger().info('Target reached: Target position reset, state -> GOAL_REACHED.')
                self.set_state(NavState.IDLE, reason="Single goal reached reported, immediate transition to IDLE")
                return

        # 3) Joystick timeout logic
        if self.joy_active and (self.robot_odom_time - self.joy_time > self.system_delay):
            self.cmd_vel_ratio = 0.0
            diff = self.robot_odom_time - self.joy_time
            self.get_logger().warn(
                '\033[93m' +
                f'Joystick timeout, stopping the robot. Time diff: {diff:.4f}s' +
                '\033[0m'
            )
        elif not self.joy_active:
            self.cmd_vel_ratio = 1.0

        # 4) Prepare Depth Image for Policy
        # On stairs: if enable_stair_remapping is True, remap abyss into progressive ramp, clamp side railing columns.
        # If enable_stair_remapping is False (raw depth test): pass 100% raw untouched depth image to policy!
        # On flat ground: always 100% bitwise identical untouched depth image!
        depth_for_policy = self.depth_image
        if is_stair_down and self.enable_stair_remapping and self.depth_image is not None:
            depth_for_policy = self.depth_remapper.process(self.depth_image, is_stair_down=True)
            if self.remapped_depth_pub.get_subscription_count() > 0:
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(depth_for_policy.astype(np.float32), encoding="32FC1")
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self.remapped_depth_pub.publish(img_msg)
                except Exception:
                    pass

        # 5) Call the learning model
        curr_robot_pos = pos_for_tracking if (pos_for_tracking is not None) else self.robot_pos_w
        curr_robot_ori = ori_for_tracking if (ori_for_tracking is not None) else self.robot_orientation_w

        cmd, action, target_vec_b = self.model.predict(
            self.linear_vel,
            self.angular_vel,
            self.gravity_vector,
            self.last_action,
            self.target_pos_w,
            curr_robot_pos,
            curr_robot_ori,
            depth_for_policy,
            target_yaw_w=self.target_yaw_w,
            is_reset=self.is_reset_hidden_state
        )

        if self.is_reset_hidden_state:
            self.get_logger().warn('\033[93m' + 'Resetting hidden state.' + '\033[0m')
            self.is_reset_hidden_state = False
            self.last_action = self._reset_last_action()

        self.last_action = action.tolist()

        # Apply low-pass filter and form Twist message
        twist = Twist()
        model_cmd = np.array([
            cmd[0].item() * self.cmd_vel_ratio,
            cmd[1].item() * self.cmd_vel_ratio * constants.LATERAL_VELOCITY_SCALE,
            cmd[2].item() * self.cmd_vel_ratio
        ])

        # Door speed boost only applies on flat-ground doors, NEVER on stairs!
        if not is_stair_down and not is_stair_up and getattr(self.path_tracker, 'is_in_narrow_passage', False) and self.door_speed_boost > 0.0:
            if target_vec_b is not None:
                tv = np.array(target_vec_b).flatten()
                if len(tv) >= 2:
                    yaw_err = math.atan2(tv[1], tv[0])
                    if abs(yaw_err) < math.radians(20.0) and model_cmd[0] < self.door_speed_boost:
                        model_cmd[0] = self.door_speed_boost

        # Stair descent & ascent anti-reversal & stability guard:
        # Prevent backward motion on stairs while allowing policy to freely decelerate and stop (vx >= 0.0).
        # NEVER impose a hard positive forward velocity floor!
        if is_stair_down or is_stair_up:
            if model_cmd[0] < 0.0:
                model_cmd[0] = 0.0
            # Limit rotational spin rate on stairs to prevent violent turns into side railings
            model_cmd[2] = float(np.clip(model_cmd[2], -0.5, 0.5))

        filter_coef = np.array(constants.LOW_PASS_FILTER_COEF)
        filt_model = filter_coef * model_cmd + (1 - filter_coef) * self.prev_cmd
        twist.linear.x = float(filt_model[0]) + self.joy_linear_x
        twist.linear.y = float(filt_model[1]) + self.joy_linear_y
        twist.angular.z = float(filt_model[2]) + self.joy_angular_z
        self.prev_cmd = filt_model

        self.cmd_vel_publisher.publish(twist)
        self.cmd_vel_direct_publisher.publish(twist)

        # Publish visualization markers
        self.visualization_manager.publish_twist_marker(
            twist, self.robot_odom_time, self.robot_frame_id, self.twist_marker_publisher
        )

        if is_stair_down or is_stair_up:
            mode_tag = "DESCENT" if is_stair_down else "ASCENT"
            self.get_logger().info(
                f'[StairsNav][{mode_tag}_CMD] vx={twist.linear.x:.3f}m/s, vy={twist.linear.y:.3f}m/s, wz={twist.angular.z:.3f}rad/s',
                throttle_duration_sec=1.0
            )

        self._reset_joystick()


def main(args=None):
    parser = argparse.ArgumentParser(description="RL Navigation Controller with Stair Descent Support")
    parser.add_argument('--sim', action='store_true', help='run in simulation mode')
    parser.add_argument('--policy-path', type=str, default=None, help='Path to custom nav_policy.onnx')
    parser.add_argument('--arrive-threshold', type=float, default=0.45, help='Goal arrival distance threshold (m)')
    parser.add_argument('--path-topic', type=str, default=None, help='Global path topic name')
    parser.add_argument('--stop-topic', type=str, default=None, help='Mission control topic name')
    parser.add_argument('--control-topic', type=str, default=None, help='Mission control topic alias')
    parser.add_argument('--status-topic', type=str, default=None, help='Local planner status topic name')
    parser.add_argument('--status-msgs-format', type=str, default=None, choices=['string_brackets', 'list', 'empty_string', 'brackets_all'])
    parser.add_argument('--cmd-vel-topic', type=str, default=None, help='Velocity command topic name')
    parser.add_argument('--motion-enable-topic', type=str, default=None, help='Chassis motion enable topic name')
    parser.add_argument('--odom-topic', type=str, default=None, help='Robot odometry topic name')
    parser.add_argument('--lookahead-distance', type=float, default=None, help='Dynamic lookahead distance (m)')
    parser.add_argument('--adaptive-lookahead', dest='enable_adaptive_lookahead', action='store_true', default=None)
    parser.add_argument('--no-adaptive-lookahead', dest='enable_adaptive_lookahead', action='store_false')
    parser.add_argument('--adaptive-mode', type=str, default=None, choices=['both', 'curvature_only', 'clearance_only', 'none', 'curv', 'door'])
    parser.add_argument('--curv-only', '--curvature-only', dest='adaptive_mode', action='store_const', const='curvature_only')
    parser.add_argument('--door-only', '--clearance-only', dest='adaptive_mode', action='store_const', const='clearance_only')
    parser.add_argument('--lookahead-min', type=float, default=None)
    parser.add_argument('--lookahead-max', type=float, default=None)
    parser.add_argument('--door-speed-boost', type=float, default=None)
    parser.add_argument('--report-failure', dest='report_failure', action='store_true', default=None)
    parser.add_argument('--no-report-failure', dest='report_failure', action='store_false')
    parser.add_argument('--segment-arrive-threshold', type=float, default=None)
    parser.add_argument('--blocked-timeout', type=float, default=None)

    # Stairs-specific CLI arguments
    parser.add_argument('--stairs-mode', dest='enable_stairs_mode', action='store_true', default=None, help='Enable stairs descent support')
    parser.add_argument('--no-stairs-mode', dest='enable_stairs_mode', action='store_false', help='Disable stairs descent support')
    parser.add_argument('--enable-stair-remapping', dest='enable_stair_remapping', action='store_true', default=None, help='Enable depth remapping on stairs')
    parser.add_argument('--no-stair-remapping', dest='enable_stair_remapping', action='store_false', help='Disable depth remapping on stairs (raw depth mode)')
    parser.add_argument('--stair-lookahead', type=float, default=None, help='Lookahead distance on descending stairs (default: 0.40m)')
    parser.add_argument('--stair-up-lookahead', type=float, default=None, help='Lookahead distance on ascending stairs (default: 0.45m)')
    parser.add_argument('--stair-min-forward-vel', type=float, default=None, help='Minimum forward velocity floor on stairs (default: 0.0m/s)')
    parser.add_argument('--stair-slope-thresh', type=float, default=None, help='Slope threshold for stair descent (default: -0.20)')
    parser.add_argument('--stair-up-slope-thresh', type=float, default=None, help='Slope threshold for stair ascent (default: +0.20)')
    parser.add_argument('--stair-side-clamp', type=float, default=None, help='Side railing column min clamp (default: 0.50m)')
    parser.add_argument('--stair-min-drop', type=float, default=None, help='Minimum elevation drop for stair descent (default: 0.15m)')
    parser.add_argument('--stair-up-min-gain', type=float, default=None, help='Minimum elevation gain for stair ascent (default: 0.15m)')

    known_args, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    use_sim = known_args.sim

    pkg_name = 'rl_nav_controller'
    pkg_path = get_package_share_directory(pkg_name)
    preprocess_model_path = os.path.join(pkg_path, 'deployment_policies', 'vae_encoder.onnx')

    if known_args.policy_path:
        policy_model_path = os.path.expanduser(known_args.policy_path)
    else:
        policy_model_path = os.path.join(pkg_path, 'deployment_policies', 'nav_policy.onnx')

    node = StairsNavigationPolicyNode(
        preprocess_model_path=preprocess_model_path,
        policy_model_path=policy_model_path,
        min_depth=0.25,
        max_depth=10.0,
        control_frequency=5.0,
        arrive_threshold=known_args.arrive_threshold,
        use_sim=use_sim
    )

    if known_args.path_topic:
        node.path_topic = known_args.path_topic
    if known_args.control_topic:
        node.stop_topic = known_args.control_topic
    elif known_args.stop_topic:
        node.stop_topic = known_args.stop_topic
    if known_args.status_topic:
        node.status_topic = known_args.status_topic
    if known_args.status_msgs_format:
        node.status_msgs_format = known_args.status_msgs_format
    if known_args.cmd_vel_topic:
        node.cmd_vel_topic = known_args.cmd_vel_topic
    if known_args.motion_enable_topic:
        node.motion_enable_topic = known_args.motion_enable_topic
    if known_args.odom_topic:
        node.odom_topic = known_args.odom_topic
    if known_args.lookahead_distance is not None:
        node.lookahead_distance = known_args.lookahead_distance
    if known_args.report_failure is not None:
        node.report_failure = known_args.report_failure
    if known_args.segment_arrive_threshold is not None:
        node.segment_arrive_threshold = known_args.segment_arrive_threshold
    elif known_args.arrive_threshold is not None:
        node.segment_arrive_threshold = known_args.arrive_threshold
    if known_args.enable_adaptive_lookahead is not None:
        node.enable_adaptive_lookahead = known_args.enable_adaptive_lookahead
    if known_args.adaptive_mode is not None:
        node.adaptive_mode = known_args.adaptive_mode
        if hasattr(node.path_tracker, 'set_adaptive_mode'):
            node.path_tracker.set_adaptive_mode(known_args.adaptive_mode)
    if known_args.lookahead_min is not None:
        node.lookahead_min = known_args.lookahead_min
        if hasattr(node.path_tracker, 'lookahead_min'):
            node.path_tracker.lookahead_min = known_args.lookahead_min
    if known_args.lookahead_max is not None:
        node.lookahead_max = known_args.lookahead_max
        if hasattr(node.path_tracker, 'lookahead_max'):
            node.path_tracker.lookahead_max = known_args.lookahead_max
    if known_args.door_speed_boost is not None:
        node.door_speed_boost = known_args.door_speed_boost

    # Configure stairs arguments
    if known_args.enable_stairs_mode is not None:
        node.enable_stairs_mode = known_args.enable_stairs_mode
    if known_args.enable_stair_remapping is not None:
        node.enable_stair_remapping = known_args.enable_stair_remapping
    if known_args.stair_lookahead is not None:
        node.stair_lookahead_dist = known_args.stair_lookahead
    if known_args.stair_up_lookahead is not None:
        node.stair_up_lookahead_dist = known_args.stair_up_lookahead
    if known_args.stair_min_forward_vel is not None:
        node.stair_min_forward_vel = known_args.stair_min_forward_vel
    if known_args.stair_slope_thresh is not None:
        node.slope_gate.slope_threshold = known_args.stair_slope_thresh
    if known_args.stair_up_slope_thresh is not None:
        node.slope_gate.up_slope_threshold = known_args.stair_up_slope_thresh
    if known_args.stair_side_clamp is not None:
        node.depth_remapper.side_min_clamp = known_args.stair_side_clamp
    if known_args.stair_min_drop is not None:
        node.slope_gate.min_elevation_drop = known_args.stair_min_drop
    if known_args.stair_up_min_gain is not None:
        node.slope_gate.min_elevation_gain = known_args.stair_up_min_gain

    node.get_logger().info(
        f'RL Navigation (Stairs-Aware) running. Stairs: {node.enable_stairs_mode} '
        f'(down_L={node.stair_lookahead_dist:.2f}m, up_L={node.stair_up_lookahead_dist:.2f}m, '
        f'remapping={node.enable_stair_remapping}, '
        f'down_slope_thresh={node.slope_gate.slope_threshold:.2f}, '
        f'up_slope_thresh={node.slope_gate.up_slope_threshold:.2f}, '
        f'min_drop={node.slope_gate.min_elevation_drop:.2f}m, '
        f'min_gain={node.slope_gate.min_elevation_gain:.2f}m, '
        f'side_clamp={node.depth_remapper.side_min_clamp:.2f}m)'
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
        try:
            node.get_logger().error(f"Uncaught exception in spin: {e}")
        except Exception:
            pass
    finally:
        try:
            node.lock_chassis()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
