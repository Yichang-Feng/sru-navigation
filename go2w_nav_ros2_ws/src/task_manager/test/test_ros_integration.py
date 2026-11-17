"""Live goal/status/TF topics, with isolated names and no action dependency."""
# flake8: noqa: E402

import json
import os
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
pytestmark = pytest.mark.skipif(
    os.environ.get('TASK_MANAGER_ROS_TESTS') != '1',
    reason='set TASK_MANAGER_ROS_TESTS=1 to run live DDS topic tests')
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from task_manager.task_manager_node import TaskManagerNode
from test_mission_executor import plan


@pytest.fixture
def ros(request):
    rclpy.init()
    prefix = '/mission_test_' + uuid.uuid4().hex
    fake = Node('mission_test_topics')
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    global_pub = fake.create_publisher(String, prefix + '/global_status', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    local_pub = fake.create_publisher(String, prefix + '/execution_status', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    path_pub = fake.create_publisher(Path, prefix + '/global_path', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    localization_pub = fake.create_publisher(String, prefix + '/localization_status', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cached = getattr(request, 'param', None)
    localization_options = (cached[1] if isinstance(cached, tuple) and
                            cached[0] == 'localization' else {})
    auto_localize = localization_options.get('auto', True)
    cached_localization = localization_options.get(
        'cached', {'state': 'succeed', 'msgs': []} if auto_localize else None)
    if cached_localization is not None:
        localization_pub.publish(String(data=json.dumps(cached_localization)))
    if isinstance(cached, str):
        global_pub.publish(String(data=cached))
    elif isinstance(cached, dict):
        local_pub.publish(String(data=json.dumps(cached)))
    node = TaskManagerNode(parameter_overrides=[
        Parameter('goal_topic', value=prefix + '/goal'),
        Parameter('command_topic', value=prefix + '/command'),
        Parameter('status_topic', value=prefix + '/status'),
        Parameter('control_topic', value=prefix + '/control'),
        Parameter('local_planner_status_topic', value=prefix + '/execution_status'),
        Parameter('global_planner_status_topic', value=prefix + '/global_status'),
        Parameter('localization_status_topic', value=prefix + '/localization_status'),
        Parameter('global_path_topic', value=prefix + '/global_path'),
        Parameter('odom_topic', value=prefix + '/odom'),
        Parameter('robot_frame', value=prefix.strip('/') + '_base'),
    ], cli_args=['--ros-args', '-r', '/estop:=' + prefix + '/estop'])
    executor = SingleThreadedExecutor()
    executor.add_node(fake)
    executor.add_node(node)
    goals, statuses = [], []
    node.received_controls = []
    node.test_global_status = lambda value: global_pub.publish(String(data=value))
    fake.create_subscription(
        String, prefix + '/control', lambda m: node.received_controls.append(m.data), 10)
    fake.create_subscription(PoseStamped, prefix + '/goal', goals.append, 10)
    fake.create_subscription(
        String, prefix + '/status', lambda m: statuses.append(json.loads(m.data)), 10)
    commands = fake.create_publisher(String, prefix + '/command', 10)
    execution_status = local_pub
    node.test_path = path_pub.publish
    node.test_raw_local = lambda raw: local_pub.publish(String(data=raw))
    node.test_raw_localization = lambda raw: localization_pub.publish(String(data=raw))

    def localization(state, msgs=None):
        localization_pub.publish(String(data=json.dumps({'state': state, 'msgs': msgs or []})))

    node.test_localization = localization
    localization_timer = (fake.create_timer(1.0, lambda: localization('succeed'))
                          if auto_localize else None)
    node.test_localization_timer = localization_timer
    broadcaster = TransformBroadcaster(fake)

    def broadcast():
        msg = TransformStamped()
        msg.header.frame_id = 'map'
        msg.child_frame_id = node.executor_core.config['robot_frame']
        msg.header.stamp = fake.get_clock().now().to_msg()
        msg.transform.rotation.w = 1.0
        broadcaster.sendTransform(msg)

    timer = fake.create_timer(.02, broadcast)

    def until(predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail('ROS topic condition timed out')

    def command(payload):
        commands.publish(String(data=json.dumps(payload)))

    def status(value, msgs=None):
        execution_status.publish(String(data=json.dumps({'state': value, 'msgs': msgs or []})))

    try:
        until(lambda: commands.get_subscription_count() > 0
              and execution_status.get_subscription_count() > 0
              and path_pub.get_subscription_count() > 0
              and global_pub.get_subscription_count() >= 2
              and localization_pub.get_subscription_count() > 0
              and node.goal_pub.get_subscription_count() > 0
              and node.status_pub.get_subscription_count() > 0
              and node.control_pub.get_subscription_count() > 0
              and node.localization_age() is not None)
        if auto_localize:
            until(lambda: node.executor_core.localization_ready)
        yield node, goals, statuses, command, status, until
    finally:
        fake.destroy_timer(timer)
        if localization_timer is not None:
            fake.destroy_timer(localization_timer)
        node.destroy_node()
        fake.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_live_three_goals_complete_without_running(ros):
    node, goals, statuses, command, status, until = ros
    command(plan(dwell=.1))
    for index in range(3):
        until(lambda: len(goals) == index + 1)
        assert goals[-1].pose.position.x == index + 1
        assert goals[-1].header.frame_id == 'map'
        status('goal_reached')
        until(lambda: node.executor_core.last_result is not None
              and node.executor_core.last_result['step_index'] == index)
    until(lambda: any(s['state'] == 'succeeded' for s in statuses))
    until(lambda: statuses[-1]['state'] == 'idle')
    assert node.executor_core.state == 'idle'
    assert len([s for s in statuses if s['state'] == 'succeeded']) == 1
    assert len(goals) == 3


def test_live_pause_arrival_resume_and_stop(ros):
    node, goals, statuses, command, status, until = ros
    command(plan(dwell=.1))
    until(lambda: len(goals) == 1)
    command(control_command('pause'))
    until(lambda: node.executor_core.state == 'canceling')
    status('goal_reached')
    until(lambda: node.executor_core.last_result is not None)
    assert len(goals) == 1 and node.executor_core.state == 'canceling'
    status('stopped')
    until(lambda: node.executor_core.state == 'paused')
    command(control_command('resume'))
    until(lambda: len(goals) == 2)
    assert node.received_controls == ['pause']  # Dwell never resumes the old path.
    node.test_path(matching_path(node, goals[-1]))
    until(lambda: node.received_controls == ['pause', 'resume'])
    status('running')
    until(lambda: node.executor_core.state == 'running')
    command(control_command('stop'))
    until(lambda: node.executor_core.state == 'canceling')
    assert not any(s['state'] == 'canceled' for s in statuses)
    status('stopped')
    until(lambda: any(s['state'] == 'canceled' for s in statuses))
    assert node.executor_core.state == 'idle' and node.executor_core.token is None
    status('goal_reached')
    started = time.monotonic()
    until(lambda: time.monotonic() - started >= .2)
    assert len(goals) == 2


def test_live_distance_updates_from_path_and_odometry(ros):
    from nav_msgs.msg import Odometry, Path
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    node, goals, statuses, command, status, until = ros
    config = node.executor_core.config
    path_pub = node.create_publisher(Path, config['global_path_topic'], QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, config['odom_topic'], qos_profile_sensor_data)
    until(lambda: path_pub.get_subscription_count() and odom_pub.get_subscription_count())
    command(plan(dwell=10))
    until(lambda: len(goals) == 1)
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = node.get_clock().now().to_msg()
    for x, y in ((0, 0), (1, 0), (1, 2)):
        point = PoseStamped()
        point.pose.position.x, point.pose.position.y = float(x), float(y)
        point.pose.position.z = .5
        path.poses.append(point)
    path_pub.publish(path)
    node.test_global_status('path_found')
    until(lambda: node.executor_core.state == 'running')
    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.header.stamp = node.get_clock().now().to_msg()
    odom.pose.pose.position.z = .5
    odom_pub.publish(odom)
    until(lambda: node.executor_core.distance is not None)
    assert node.executor_core.distance == pytest.approx(3)
    odom.pose.pose.position.x, odom.pose.pose.position.y = 1.0, 1.0
    odom.header.stamp = node.get_clock().now().to_msg()
    odom_pub.publish(odom)
    until(lambda: node.executor_core.distance == pytest.approx(1))
    until(lambda: any(s['remaining_distance_m'] == pytest.approx(1) for s in statuses
                     if s['remaining_distance_m'] is not None))
    status('running')
    until(lambda: node.executor_core.running_seen)
    status('goal_reached')
    until(lambda: node.executor_core.step_state == 'dwelling')
    assert node.executor_core.distance == 0


def test_live_accepted_controls_are_published_as_strings(ros):
    node, goals, statuses, command, status, until = ros
    command(plan())
    until(lambda: len(goals) == 1)
    assert node.received_controls == []
    node.test_path(matching_path(node, goals[-1]))
    until(lambda: node.executor_core.path_ready)
    command(control_command('pause'))
    until(lambda: node.received_controls == ['pause'])
    assert node.executor_core.state == 'canceling'
    status('stopped')
    until(lambda: node.executor_core.state == 'paused')
    command(control_command('resume'))
    until(lambda: node.received_controls == ['pause', 'resume'])
    assert node.executor_core.state == 'starting'
    status('running')
    until(lambda: node.executor_core.state == 'running')
    command(control_command('stop'))
    until(lambda: node.received_controls == ['pause', 'resume', 'pause'])
    status('stopped')
    until(lambda: node.executor_core.state == 'idle')


def test_live_global_feedback_is_reported_and_failure_allows_new_run(ros):
    node, goals, statuses, command, status, until = ros
    command(plan())
    until(lambda: len(goals) == 1)
    assert node.executor_core.state == 'starting'
    node.test_global_status('planning')
    until(lambda: node.executor_core.global_planner_status == 'planning')
    node.test_global_status('path_found')
    until(lambda: node.executor_core.state == 'running')
    until(lambda: any(s['global_planner_status'] == 'path_found' for s in statuses))
    raw = 'planning_failed: no_path; 未找到可行路径'
    node.test_global_status(raw)
    until(lambda: any(s['state'] == 'failed' for s in statuses))
    assert node.executor_core.state == 'idle'
    until(lambda: any(s['message'] == raw for s in statuses))
    assert node.executor_core.token is None
    command(plan(run_id='new', request_id='new'))
    until(lambda: len(goals) == 2)
    node.test_global_status('goal_reached')
    until(lambda: node.executor_core.step_state == 'dwelling')
    assert node.executor_core.distance == 0


@pytest.mark.parametrize('ros', ['planning_failed: no_path; cached previous run'], indirect=True)
def test_live_old_cached_request_failure_does_not_fail_new_goal(ros):
    node, goals, statuses, command, status, until = ros
    command(plan())
    until(lambda: len(goals) == 1)
    assert node.executor_core.state == 'starting'
    assert node.executor_core.global_planner_status is None
    node.test_global_status('path_found')
    until(lambda: node.executor_core.state == 'running')


@pytest.mark.parametrize('ros', ['map_load_failed: 地图加载失败; cached'], indirect=True)
def test_live_cached_map_fault_is_available_to_late_task_manager(ros):
    node, goals, statuses, command, status, until = ros
    until(lambda: node.executor_core.state == 'error')
    assert node.executor_core.global_planner_status.startswith('map_load_failed:')
    command(plan())
    until(lambda: any('command' in s for s in statuses))
    assert not goals and node.executor_core.state == 'error'


@pytest.mark.parametrize('ending', ['succeeded', 'canceled', 'failed'])
def test_live_terminal_event_is_sent_once_then_idle_heartbeats(ros, ending):
    node, goals, statuses, command, status, until = ros
    command(plan(1))
    until(lambda: len(goals) == 1)
    if ending == 'succeeded':
        status('goal_reached')
    elif ending == 'canceled':
        command(dict(schema_version=1, robot_id='a2', run_id='run1',
                     request_id='stop', command='stop'))
        until(lambda: node.executor_core.state == 'canceling')
        status('stopped')
    else:
        node.test_global_status('planning_failed: no_path; test once')
    until(lambda: any(s['state'] == ending for s in statuses))
    until(lambda: statuses[-1]['state'] == 'idle')
    final = next(s for s in statuses if s['state'] == ending)
    assert final['run_id'] == 'run1' and final['step_total'] == 1
    started = time.monotonic()
    until(lambda: time.monotonic() - started >= 1.2)
    assert len([s for s in statuses if s['state'] == ending]) == 1
    assert statuses[-1]['state'] == 'idle' and statuses[-1]['run_id'] is None
    assert statuses[-1]['last_step_result'] is None
    assert statuses[-1]['step_index'] is None



def control_command(event):
    return dict(schema_version=1, robot_id='a2', run_id='run1',
                request_id=event, command=event)


def matching_path(node, goal):
    import copy
    path = Path()
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = node.get_clock().now().to_msg()
    point = copy.deepcopy(goal)
    point.pose.position.z += node.executor_core.config['global_path_z_offset']
    path.poses.append(point)
    return path


@pytest.mark.parametrize('ros', [{'state': 'goal_reached', 'msgs': []}], indirect=True)
def test_live_cached_local_completion_does_not_complete_new_run(ros):
    node, goals, statuses, command, status, until = ros
    command(plan())
    until(lambda: len(goals) == 1)
    started = time.monotonic()
    until(lambda: time.monotonic() - started >= .3)
    assert node.executor_core.local_planner_status is None
    assert node.executor_core.last_result is None and node.executor_core.state == 'starting'
    status('goal_reached')
    until(lambda: node.executor_core.last_result is not None)


@pytest.mark.parametrize('recovery', ['running', 'goal_reached'])
def test_live_json_fault_recovers_and_invalid_input_is_ignored(ros, recovery):
    node, goals, statuses, command, status, until = ros
    command(plan(dwell=10))
    until(lambda: len(goals) == 1)
    status('running')
    until(lambda: node.executor_core.state == 'running')
    status('error', ['astar_no_path', 'unknown_reason'])
    until(lambda: statuses[-1]['state'] == 'error')
    assert statuses[-1]['step_state'] == 'navigating'
    assert statuses[-1]['local_planner_status']['msgs'] == ['astar_no_path', 'unknown_reason']
    node.test_raw_local('{"state":"goal_reached","msgs":false}')
    started = time.monotonic()
    until(lambda: time.monotonic() - started >= .2)
    assert node.executor_core.last_result is None
    assert node.executor_core.fault is None and len(goals) == 1
    status(recovery)
    until(lambda: statuses[-1]['state'] == 'running')
    assert statuses[-1]['local_planner_status']['state'] == recovery
    assert not any(s['state'] == 'failed' for s in statuses)


def test_live_canceled_new_run_resumes_only_with_new_matching_path(ros):
    import copy
    node, goals, statuses, command, status, until = ros
    command(plan())
    until(lambda: len(goals) == 1)
    old = matching_path(node, goals[-1])
    command(control_command('stop'))
    until(lambda: node.executor_core.state == 'canceling')
    status('stopped')
    until(lambda: node.executor_core.state == 'idle')
    command(plan(run_id='new', request_id='new'))
    until(lambda: len(goals) == 2)
    assert node.received_controls == ['pause']
    node.test_path(old)
    node.test_global_status('path_found')
    until(lambda: node.executor_core.global_planner_status == 'path_found')
    wrong = matching_path(node, goals[-1])
    wrong.poses[-1].pose.position.x += 10.0
    node.test_path(wrong)
    until(lambda: node.path_stamp is not None)
    assert node.received_controls == ['pause'] and node.executor_core.state == 'starting'
    good = matching_path(node, copy.deepcopy(goals[-1]))
    node.test_path(good)
    until(lambda: node.received_controls == ['pause', 'resume'])
    assert node.executor_core.state == 'starting'
    status('running')
    until(lambda: node.executor_core.state == 'running')


@pytest.mark.parametrize('ros', [('localization', {'auto': False})], indirect=True)
def test_live_localization_startup_rejection_and_no_queued_task(ros):
    node, goals, statuses, command, status, until = ros
    core = node.executor_core
    until(lambda: statuses and statuses[-1]['state'] == 'uninitialized_location')
    assert core.localization_status is None
    command(plan())
    until(lambda: any(s.get('command', {}).get('reason') == 'localization_not_ready'
                     for s in statuses))
    assert not goals and node.received_controls == []
    node.test_localization('ready')
    until(lambda: core.localization_status == {'state': 'ready', 'msgs': []})
    node.test_localization('succeed')
    until(lambda: core.localization_successes == 1)
    assert core.state == 'uninitialized_location'
    node.test_localization('succeed')
    until(lambda: core.state == 'idle')
    assert not goals and node.received_controls == []
    command(plan())
    until(lambda: len(goals) == 1)
    assert core.state == 'starting'


@pytest.mark.parametrize('ros', [
    ('localization', {'auto': False, 'cached': {'state': 'succeed', 'msgs': []}}),
], indirect=True)
def test_live_cached_localization_success_requires_subsequent_heartbeat(ros):
    node, goals, statuses, command, status, until = ros
    core = node.executor_core
    until(lambda: core.localization_successes == 1)
    started = time.monotonic()
    until(lambda: time.monotonic() - started > .3)
    assert not core.localization_ready and core.state == 'uninitialized_location'
    command(plan())
    until(lambda: any(s.get('command', {}).get('reason') == 'localization_not_ready'
                     for s in statuses))
    node.test_localization('succeed')
    until(lambda: core.localization_ready)
    assert not goals


@pytest.mark.parametrize('ros', [
    ('localization', {'auto': False, 'cached': {'state': 'ready', 'msgs': []}}),
], indirect=True)
def test_live_cached_ready_has_no_event_heartbeat_timeout_and_bad_json_is_ignored(ros):
    node, goals, statuses, command, status, until = ros
    core = node.executor_core
    until(lambda: core.localization_status == {'state': 'ready', 'msgs': []})
    node.test_raw_localization('{"state":"succeed","msgs":"not-an-array"}')
    started = time.monotonic()
    until(lambda: time.monotonic() - started > 3.1)
    assert core.state == 'uninitialized_location' and not core.localization_ready
    assert core.localization_status == {'state': 'ready', 'msgs': []}
    assert not any(s['state'] in ('failed', 'idle') for s in statuses)
    assert not goals and node.received_controls == []


@pytest.mark.parametrize('cause', ['ready', 'error', 'initialized_failed', 'failed'])
def test_live_localization_failure_aborts_and_recovery_requires_stop_ack_and_new_run(ros, cause):
    node, goals, statuses, command, status, until = ros
    core = node.executor_core
    command(plan())
    until(lambda: len(goals) == 1)
    node.test_localization_timer.cancel()
    node.test_localization(cause, ['localization_lost', 'unknown_reason'])
    until(lambda: any(s['state'] == 'failed' for s in statuses))
    until(lambda: statuses[-1]['state'] == 'uninitialized_location')
    until(lambda: node.received_controls == ['pause'])
    final = next(s for s in statuses if s['state'] == 'failed')
    assert final['run_id'] == 'run1' and 'unknown_reason' in final['message']
    assert final['localization_status']['state'] == cause
    assert core.fault is None and core.token is not None
    node.test_localization('succeed')
    until(lambda: core.localization_successes == 1)
    node.test_localization('succeed')
    until(lambda: core.state == 'idle')
    command(plan(run_id='new', request_id='new'))
    until(lambda: any(s.get('command', {}).get('reason') == 'run_active' for s in statuses))
    assert len(goals) == 1
    status('stopped')
    until(lambda: core.token is None)
    command(plan(run_id='new', request_id='new'))
    until(lambda: len(goals) == 2)
    assert core.last_result is None
    assert len([s for s in statuses if s['state'] == 'failed']) == 1
    assert node.received_controls == ['pause']  # New goal still requires a matching path.


def test_live_localization_heartbeat_timeout_with_continuous_tf(ros):
    node, goals, statuses, command, status, until = ros
    core = node.executor_core
    command(plan())
    until(lambda: len(goals) == 1)
    node.test_localization_timer.cancel()
    until(lambda: any(s['state'] == 'failed' for s in statuses), timeout=4)
    until(lambda: statuses[-1]['state'] == 'uninitialized_location')
    until(lambda: node.received_controls == ['pause'])
    assert node.localization_age() < .5  # TF remained healthy throughout the status outage.
    final = next(s for s in statuses if s['state'] == 'failed')
    assert 'succeed heartbeat timed out' in final['message']
    assert final['localization_status'] == {'state': 'succeed', 'msgs': []}
    status('stopped')
    until(lambda: core.token is None)
    assert len(goals) == 1 and not core.localization_ready
