"""ROS message conversion and String status filtering without DDS startup."""
# flake8: noqa: E402

from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from builtin_interfaces.msg import Time
from std_msgs.msg import String

from task_manager.task_manager_node import TaskManagerNode, make_pose
from test_mission_executor import Simulation, plan


def test_pose_preserves_position_frame_stamp_and_yaw():
    pose = make_pose(plan()['mission']['steps'][0], 'custom', Time(sec=12, nanosec=34))
    assert pose.header.frame_id == 'custom' and pose.header.stamp.nanosec == 34
    assert pose.pose.position.x == 1.0 and pose.pose.position.y == 2.0
    assert pose.pose.orientation.z == pytest.approx(0.706825181105366)


def test_direct_completion_is_accepted_once_during_dwell():
    s = Simulation()
    s.start()
    node = SimpleNamespace(executor_core=s.core)
    TaskManagerNode.local_planner_status_cb(node, String(data='{"state":"goal_reached","msgs":[]}'))
    assert s.core.last_result['outcome'] == 'reached'
    result = s.core.last_result.copy()
    TaskManagerNode.local_planner_status_cb(node, String(data='{"state":"goal_reached","msgs":[]}'))
    assert s.core.index == 0 and s.core.last_result == result and len(s.core.results) == 1


def test_unknown_string_status_does_not_complete_goal():
    s = Simulation()
    s.start()
    node = SimpleNamespace(executor_core=s.core)
    for state in ('ok', 'unknown', '{"state":"running"}'):
        TaskManagerNode.local_planner_status_cb(node, String(data=state))
    assert s.core.last_result is None and not s.core.running_seen
    TaskManagerNode.local_planner_status_cb(node, String(data=' {"state":"running","msgs":[]} '))
    TaskManagerNode.local_planner_status_cb(node, String(data='{"state":"goal_reached","msgs":[]}'))
    assert s.core.step_state == 'dwelling'


def test_default_robot_frame_resolves_actual_lio_tf_chain():
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer

    s = Simulation()
    buffer = Buffer()
    for parent, child in (('map', 'world'), ('world', 'base')):
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp.sec = 100
        transform.transform.rotation.w = 1.0
        buffer.set_transform(transform, 'test_lio')
    node = SimpleNamespace(
        executor_core=s.core, tf_buffer=buffer,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_100_000_000)))
    assert TaskManagerNode.localization_age(node) == pytest.approx(.1)
    assert node.localization_issue == ''
    s.core.config['robot_frame'] = 'base_link'
    assert TaskManagerNode.localization_age(node) is None
    assert 'map -> base_link unavailable' in node.localization_issue


def distance_node():
    import time
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer
    from task_manager.path_distance import RemainingPath

    s = Simulation()
    s.start()
    s.core.global_status('path_found')
    node = SimpleNamespace(
        executor_core=s.core, path_distance=RemainingPath(), path_frame=None,
        goal_sent_stamp=100_000_000_000, path_stamp=None, last_odom=None,
        odom_received=time.monotonic(), tf_buffer=Buffer(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=101_000_000_000)))
    for name in ('point_in_frame', 'path_cb', 'odom_cb', 'remaining_distance'):
        setattr(node, name, getattr(TaskManagerNode, name).__get__(node))
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp.sec = 101
    for x, y in ((0, 0), (1, 0), (1, 2)):
        p = PoseStamped()
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), .5
        path.poses.append(p)
    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.header.stamp.sec = 101
    odom.pose.pose.position.z = .5
    return node, path, odom


def test_path_and_odometry_update_mission_distance_and_clear_on_next_goal():
    node, path, odom = distance_node()
    node.path_cb(path)
    node.odom_cb(odom)
    assert node.executor_core.snapshot()['remaining_distance_m'] == pytest.approx(3)
    odom.pose.pose.position.x, odom.pose.pose.position.y = 1.0, 1.0
    node.odom_cb(odom)
    assert node.executor_core.distance == pytest.approx(1)
    node.executor_core.execution_status('running')
    node.executor_core.execution_status('goal_reached')
    assert node.executor_core.distance == 0
    node.executor_core.tick()
    assert node.executor_core.snapshot()['remaining_distance_m'] is None


def test_latched_old_wrong_goal_empty_and_nonfinite_paths_are_unknown():
    node, path, odom = distance_node()
    node.odom_cb(odom)
    path.header.stamp.sec = 99
    node.path_cb(path)
    assert node.executor_core.distance is None
    path.header.stamp.sec = 101
    path.poses[-1].pose.position.x = 99.0
    node.path_cb(path)
    assert node.executor_core.distance is None
    path.header.stamp.sec = 102
    path.poses[-1].pose.position.x = 1.0
    path.poses[0].pose.position.x = float('nan')
    node.path_cb(path)
    assert node.executor_core.distance is None
    path.header.stamp.sec = 103
    path.poses.clear()
    node.path_cb(path)
    assert node.executor_core.distance is None


def test_odometry_transform_and_stale_or_missing_inputs():
    import time
    from geometry_msgs.msg import TransformStamped

    node, path, odom = distance_node()
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'world'
    transform.transform.translation.x = 1.0
    transform.transform.rotation.w = 1.0
    node.tf_buffer.set_transform_static(transform, 'test_maploc')
    odom.header.frame_id = 'world'
    node.path_cb(path)
    node.odom_cb(odom)
    assert node.executor_core.distance == pytest.approx(2)
    odom.header.frame_id = 'missing_frame'
    node.odom_cb(odom)
    assert node.executor_core.distance is None
    odom.header.frame_id = 'map'
    odom.header.stamp.sec = 90
    node.odom_cb(odom)
    assert node.executor_core.distance is None
    odom.header.stamp.sec = 102
    node.odom_cb(odom)
    assert node.executor_core.distance is None
    odom.header.stamp.sec = 101
    node.odom_cb(odom)
    node.odom_received = time.monotonic() - 3
    assert node.remaining_distance() is None


def test_global_readers_separate_health_from_request_history():
    s = Simulation()
    node = SimpleNamespace(executor_core=s.core)
    TaskManagerNode.global_planner_health_cb(node, String(data='planning_failed: no_path; old'))
    assert s.core.global_planner_status is None and s.core.state == 'idle'
    TaskManagerNode.global_planner_health_cb(node, String(data='ready'))
    assert s.messages[-1]['global_planner_status'] == 'ready'
    s.start()
    TaskManagerNode.global_planner_status_cb(node, String(data='planning'))
    assert s.core.state == 'starting'
    TaskManagerNode.global_planner_status_cb(node, String(data='path_found'))
    assert s.core.state == 'running'
    reason = 'map_load_failed: 地图加载失败; /map.pcd; invalid PCD'
    TaskManagerNode.global_planner_status_cb(node, String(data=reason))
    assert s.core.state == 'running'  # Health is handled exactly once by durable reader.
    TaskManagerNode.global_planner_health_cb(node, String(data=reason))
    assert s.core.state == 'error' and s.core.message == reason


def test_late_path_after_stop_is_ignored_with_empty_idle_run():
    node, path, odom = distance_node()
    node.executor_core.command(
        '{"schema_version":1,"robot_id":"a2","run_id":"run1",'
        '"request_id":"stop","command":"stop"}')
    node.executor_core.execution_status('stopped')
    assert node.executor_core.state == 'idle' and node.executor_core.token is None
    node.path_cb(path)
    node.odom_cb(odom)
    assert node.executor_core.distance is None and node.path_frame is None


def test_stopped_goal_resumes_only_after_fresh_matching_path():
    node, path, _ = distance_node()
    core = node.executor_core
    controls = []
    core.control = controls.append
    core.execution_status('stopped')
    core.command('{"schema_version":1,"robot_id":"a2","run_id":"run1",'
                 '"request_id":"resume","command":"resume"}')
    assert core.state == 'starting' and controls == []
    core.global_status('path_found')
    assert controls == []
    path.header.stamp.sec = 99  # Cached before the current goal was published.
    node.path_cb(path)
    assert controls == []
    path.header.stamp.sec = 101
    path.poses[-1].pose.position.x = 99.0
    node.path_cb(path)
    assert controls == []
    path.header.stamp.sec = 102
    path.poses[-1].pose.position.x = 1.0
    node.path_cb(path)
    assert controls == ['resume'] and core.state == 'starting'
    core.execution_status('running')
    assert core.state == 'running'



def test_arrival_distance_stays_zero_while_canceling_and_after_stopped():
    node, _, _ = distance_node()
    core = node.executor_core
    core.command('{"schema_version":1,"robot_id":"a2","run_id":"run1",'
                 '"request_id":"pause","command":"pause"}')
    core.execution_status('goal_reached')
    assert core.state == 'canceling' and node.remaining_distance() == 0
    core.distance = node.remaining_distance()
    core.tick()
    assert core.snapshot()['remaining_distance_m'] == 0
    core.execution_status('stopped')
    assert core.state == 'paused' and core.snapshot()['remaining_distance_m'] == 0


def test_localization_transport_parses_json_and_keeps_fault_priority():
    s = Simulation(auto_localize=False)
    node = SimpleNamespace(executor_core=s.core)
    for _ in range(2):
        TaskManagerNode.localization_status_cb(node, String(data='{"state":"succeed","msgs":[]}'))
    assert s.core.state == 'idle'
    s.core.global_status('map_load_failed: missing')
    TaskManagerNode.localization_status_cb(
        node, String(data='{"state":"error","msgs":["sensor_data_timeout","unknown"]}'))
    assert s.messages[-1]['localization_status']['msgs'] == ['sensor_data_timeout', 'unknown']
    assert s.core.state == 'error' and s.core.fault == 'map_load_failed: missing'
