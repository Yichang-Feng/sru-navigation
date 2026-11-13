"""Multi-point navigation node - cycles between predefined waypoints."""

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class MultiPointNav(Node):
    """Multi-point navigation node that cycles between waypoints."""

    def __init__(self):
        super().__init__('multi_point_nav')

        # Waypoints to cycle through: (x, y, z)
        self.waypoints = [
            (0.0, 0.0, 0.0),
            (9.0, 0.0, 0.0),
        ]
        self.current_waypoint_idx = 0

        # Distance threshold to consider waypoint reached (meters)
        self.declare_parameter('reach_threshold', 0.5)
        self.reach_threshold = self.get_parameter('reach_threshold').value

        # Publishing rate (Hz)
        self.declare_parameter('publish_rate', 10.0)
        publish_rate = self.get_parameter('publish_rate').value

        # Current robot position
        self.robot_pos = None

        # Publisher for goal pose
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/lio/robo/odom',
            self.odom_callback,
            10
        )

        # Timer for publishing goal
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Multi-point navigation started. Waypoints: {self.waypoints}, '
            f'reach_threshold: {self.reach_threshold}m'
        )

    def odom_callback(self, msg: Odometry):
        """Handle odometry messages."""
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def get_current_waypoint(self):
        """Get current target waypoint."""
        return self.waypoints[self.current_waypoint_idx]

    def check_reached(self):
        """Check if current waypoint is reached."""
        if self.robot_pos is None:
            return False

        target = np.array(self.get_current_waypoint())
        distance = np.linalg.norm(self.robot_pos[:2] - target[:2])  # 2D distance
        return distance < self.reach_threshold

    def advance_waypoint(self):
        """Advance to next waypoint (cycling)."""
        old_idx = self.current_waypoint_idx
        self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        self.get_logger().info(
            f'Waypoint {old_idx} reached! Advancing to waypoint {self.current_waypoint_idx}: '
            f'{self.get_current_waypoint()}'
        )

    def timer_callback(self):
        """Periodic callback to check waypoint and publish goal."""
        # Check if reached current waypoint
        if self.check_reached():
            self.advance_waypoint()

        # Publish current goal
        self.publish_goal()

    def publish_goal(self):
        """Publish goal pose."""
        waypoint = self.get_current_waypoint()

        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]
        msg.pose.position.z = waypoint[2]

        # Default orientation (facing forward)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiPointNav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
