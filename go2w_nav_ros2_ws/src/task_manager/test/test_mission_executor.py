"""Deterministic protocol and navigation-event simulation, without ROS."""

import copy
import json
import math

import pytest

from task_manager.mission_executor import MissionExecutor


def plan(count=3, dwell=0.0, **changes):
    payload = {
        'schema_version': 1, 'robot_id': 'a2', 'run_id': 'run1', 'request_id': 'req1',
        'command': 'start', 'frame_id': 'map',
        'mission': {'id': 'mission1', 'name': '巡检', 'map_id': 'map1', 'mode': 'once',
                    'loop_count': 1, 'steps_total': count, 'steps': [
                        {'step_index': i, 'lap': 1, 'point_index': i, 'point_total': count,
                         'name': 'point' + str(i), 'waypoint_id': 'wp' + str(i),
                         'x': i + 1.0, 'y': 2.0, 'z': 0.0, 'yaw': 1.57,
                         'dwell_sec': dwell} for i in range(count)]}}
    payload.update(changes)
    return payload


class Simulation:
    def __init__(self, auto_localize=True, **config):
        self.time, self.messages, self.warnings, self.sent = 0.0, [], [], []
        self.controls = []
        self.core = MissionExecutor(self, self.messages.append, lambda: self.time,
                                    self.stamp, self.warnings.append, config,
                                    control=self.controls.append)
        self.auto_localize = auto_localize
        if auto_localize:
            self.localize('succeed')
            self.localize('succeed')
            # Legacy mission tests begin with an already initialized robot.
            # Localization-specific tests retain the entire startup history.
            self.messages[:] = self.messages[-1:]

    def localize(self, state, msgs=None):
        self.core.localization_status_json(json.dumps({'state': state, 'msgs': msgs or []}))

    def terminal(self, state):
        matches = [msg for msg in self.messages if msg['state'] == state]
        assert len(matches) == 1
        return matches[0]

    def stamp(self):
        return {'sec': int(self.time), 'nanosec': int(self.time % 1 * 1e9)}

    def send(self, step, frame):
        self.sent.append((copy.deepcopy(step), frame))

    def command(self, payload):
        self.core.command(json.dumps(payload))

    def control(self, command):
        self.command(dict(schema_version=1, robot_id='a2', run_id='run1',
                          request_id=command, command=command))

    def tick(self, dt=0.0, ok=True, age=None):
        end = self.time + dt
        if self.auto_localize:
            # Model ongoing successful localization even in day-long goal tests.
            while self.time + 1.0 < end:
                self.time += 1.0
                self.localize('succeed')
            self.time = end
            self.localize('succeed')
        else:
            self.time = end
        self.core.tick(ok, age)

    def start(self, payload=None):
        self.command(payload or plan())
        self.tick()

    def reach(self):
        self.core.execution_status('running')
        self.core.execution_status('goal_reached')


@pytest.mark.parametrize('field,value', [
    ('x', 10 ** 1000), ('x', float('nan')), ('y', float('inf')), ('z', '1'), ('yaw', math.pi + .01),
    ('yaw', True), ('step_index', False), ('point_index', -1), ('point_total', 0),
    ('lap', 0), ('dwell_sec', -1), ('dwell_sec', 601), ('dwell_sec', float('nan')),
    ('name', None), ('waypoint_id', 5),
])
def test_invalid_steps(field, value):
    s = Simulation()
    p = plan()
    p['mission']['steps'][0][field] = value
    s.command(p)
    assert s.core.state == 'idle' and s.messages[-1]['command']['reason'] == 'invalid'


@pytest.mark.parametrize('payload', [[], None, 4, 'text', {'schema_version': True}])
def test_non_object_or_bad_version(payload):
    s = Simulation()
    s.command(payload)
    assert s.core.state == 'idle' and 'command' in s.messages[-1]


def test_bad_json_empty_oversize_and_missing_fields():
    s = Simulation()
    s.core.command('{')
    assert len(s.messages) == 1 and s.warnings
    for p in (plan(0), plan(1001)):
        s.command(p)
        assert s.messages[-1]['command']['reason'] == 'invalid'
    for key in ('run_id', 'request_id', 'mission'):
        p = plan()
        del p[key]
        s.command(p)
        assert s.messages[-1]['command']['reason'] == 'invalid'
    p = plan()
    del p['mission']['steps'][0]['dwell_sec']
    s.command(p)
    assert s.messages[-1]['command']['reason'] == 'invalid'


def test_idle_heartbeat_and_running_heartbeat():
    s = Simulation()
    assert s.messages[0]['state'] == 'idle' and s.messages[0]['run_id'] is None
    s.tick(.99)
    assert len(s.messages) == 1
    s.tick(.01)
    assert len(s.messages) == 2
    s.start()
    count = len(s.messages)
    s.tick(.5)
    assert len(s.messages) == count + 1


def test_three_points_advance_on_single_completion_without_running():
    s = Simulation()
    s.start()
    for index in range(3):
        assert len(s.sent) == index + 1
        assert s.sent[-1][0]['step_index'] == index
        s.tick(.5)
        assert s.core.index == index and len(s.sent) == index + 1
        s.core.execution_status('goal_reached')
        assert s.core.last_result['step_index'] == index
        assert s.messages[-1]['step_state'] == 'dwelling'
        s.tick()
        s.tick()
    terminal = s.terminal('succeeded')
    assert terminal['step_index'] == 2 and terminal['run_id'] == 'run1'
    assert terminal['last_step_result']['outcome'] == 'reached'
    assert s.core.state == 'idle' and s.core.run_id is None
    s.tick(1)
    assert s.messages[-1]['last_step_result'] is None
    s.terminal('succeeded')


def test_dwell_and_repeated_reached_do_not_advance_or_change_result():
    s = Simulation()
    s.start(plan(dwell=3))
    s.reach()
    result = copy.deepcopy(s.core.last_result)
    s.tick(2)
    s.core.execution_status('goal_reached')
    assert len(s.sent) == 1 and s.core.last_result == result
    assert s.messages[-1]['target']['step_index'] == 0
    assert s.messages[-1]['remaining_distance_m'] == 0
    s.tick(1)
    s.tick()
    assert len(s.sent) == 2 and s.core.index == 1


def test_idempotence_and_rejection_preserve_task():
    s = Simulation()
    s.start()
    before = s.core.snapshot()
    s.command(plan())
    assert len(s.sent) == 1 and s.messages[-1] == before
    for p, reason in [(plan(run_id='other'), 'run_active'),
                      (plan(robot_id='other'), 'robot_mismatch'),
                      (plan(schema_version=2), 'unsupported_version')]:
        s.command(p)
        assert s.messages[-1].pop('command')['reason'] == reason
        assert s.messages[-1] == before


def test_pause_stops_dispatch_and_resume_monitors_same_goal():
    s = Simulation()
    s.start()
    s.core.path_available()
    s.core.execution_status('running')
    s.control('pause')
    s.tick(50)
    assert s.core.state == 'canceling' and len(s.sent) == 1
    assert s.core.step_state is None and s.core.snapshot()['target'] is not None
    s.core.execution_status('stopped')
    assert s.core.state == 'paused'
    s.control('resume')
    assert s.core.state == 'starting' and s.controls == ['pause', 'resume']
    s.core.global_status('path_found')
    assert s.core.state == 'starting'
    s.core.execution_status('running')
    assert s.core.state == 'running' and len(s.sent) == 1
    s.core.execution_status('goal_reached')
    s.tick()
    s.tick()
    assert len(s.sent) == 2


def test_arrival_while_paused_and_dwell_resume():
    s = Simulation()
    s.start(plan(dwell=3))
    s.control('pause')
    s.core.execution_status('stopped')
    s.reach()
    result = copy.deepcopy(s.core.last_result)
    s.tick(10)
    assert s.core.state == 'paused' and len(s.sent) == 1
    s.control('resume')
    s.tick(2)
    assert len(s.sent) == 1
    s.control('pause')
    s.core.execution_status('stopped')
    s.control('resume')
    s.tick(2)
    assert len(s.sent) == 1
    s.tick(1)
    s.tick()
    assert len(s.sent) == 2 and s.core.last_result == result


def test_stop_waits_for_confirmation_and_releases_navigation():
    s = Simulation()
    s.start()
    s.control('stop')
    s.tick(100)
    assert s.core.state == 'canceling' and len(s.sent) == 1
    s.command(plan(run_id='other', request_id='other'))
    assert s.messages[-1]['command']['reason'] == 'run_active'
    s.core.execution_status('goal_reached')
    assert s.core.state == 'canceling' and s.core.last_result['outcome'] == 'reached'
    s.core.execution_status('stopped')
    assert s.core.state == 'idle' and s.core.token is None
    assert s.terminal('canceled')['run_id'] == 'run1'
    s.start(plan(run_id='other', request_id='other'))
    assert len(s.sent) == 2 and s.core.run_id == 'other'
    assert s.controls == ['pause']
    s.core.global_status('path_found')
    assert s.core.state == 'starting' and s.controls == ['pause']
    s.core.path_available()
    assert s.controls == ['pause', 'resume']
    s.core.execution_status('running')
    assert s.core.state == 'running'


@pytest.mark.parametrize('cause', ['estop', 'missing_tf', 'stale_tf'])
def test_safety_stops_sequence(cause):
    s = Simulation()
    s.start()
    if cause == 'estop':
        s.core.set_estop(True)
    else:
        s.tick(2.1, False, 2.1 if cause == 'stale_tf' else None)
    assert s.core.state == 'idle'
    assert s.terminal('failed')['last_step_result']['outcome'] == 'failed'
    s.tick(100)
    assert len(s.sent) == 1
    s.reach()
    if cause == 'estop':
        s.command(plan(run_id='new', request_id='new'))
        assert s.messages[-1]['command']['reason'] == 'estop_active'
        s.core.set_estop(False)
    s.command(plan(run_id='new', request_id='new'))
    assert s.core.state == 'starting'


def test_long_navigation_wait_does_not_fail_retry_or_skip():
    s = Simulation()
    s.start()
    for elapsed in (30, 120, 3600, 86400):
        s.core.execution_status('running')
        s.tick(elapsed)
        assert s.core.state == 'running' and s.core.index == 0
        assert s.core.last_result is None and len(s.sent) == 1
    s.core.execution_status('goal_reached')
    s.tick()
    s.tick()
    assert s.core.index == 1 and len(s.sent) == 2


def test_already_at_target_and_repeated_positions_complete_without_running():
    s = Simulation()
    payload = plan(count=2, dwell=1)
    payload['mission']['steps'][1].update(x=1.0, y=2.0, z=0.0)
    s.start(payload)
    for index in range(2):
        s.core.execution_status('goal_reached')
        assert s.core.index == index and s.core.last_result['step_index'] == index
        assert s.core.step_state == 'dwelling' and s.core.distance == 0
        s.tick(1)
        s.tick()
    assert s.core.state == 'idle' and s.core.results == {}
    assert s.terminal('succeeded')['last_step_result']['step_index'] == 1


def test_metadata_frame_and_count_warning():
    s = Simulation()
    p = plan(frame_id='custom')
    p['mission'].update(mode='loop', loop_count=7, steps_total=9)
    p['mission']['steps'][0].update(lap=5, point_index=1, point_total=2)
    s.start(p)
    assert s.sent[0][1] == 'custom' and s.warnings
    status = s.messages[-1]
    assert (status['lap'], status['current_index'], status['total']) == (5, 1, 2)
    assert status['remaining_distance_m'] is None


def test_terminal_new_run_resets_results_and_old_start_is_idempotent():
    s = Simulation()
    s.start(plan(1))
    s.reach()
    s.tick()
    s.start(plan(run_id='new', request_id='new'))
    assert s.core.last_result is None
    s.command(plan(1))
    assert s.core.run_id == 'new' and len(s.sent) == 2


@pytest.mark.parametrize('key,value', [
    ('status_rate_hz', 0), ('goal_topic', ''), ('dwell_default_sec', 601),
    ('tf_timeout_sec', float('nan')), ('local_planner_status_topic', ''),
    ('global_planner_status_topic', ''),
])
def test_invalid_config(key, value):
    s = Simulation(**{key: value})
    assert s.core.state == 'error' and key in s.core.message
    s.tick(1)


def test_invalid_receipt_remains_json_serializable():
    s = Simulation()
    s.command(dict(schema_version=1, robot_id='a2', request_id=float('nan'),
                   run_id='run1', command={'bad': float('inf')}))
    json.dumps(s.messages[-1], allow_nan=False)


def test_control_relay_follows_accepted_state_changes_only():
    s = Simulation()
    s.start()
    s.command(plan())
    assert s.controls == []
    s.core.path_available()
    s.control('pause')
    s.control('pause')
    s.control('resume')  # Still canceling: rejected.
    assert s.controls == ['pause']
    s.core.execution_status('stopped')
    s.control('resume')
    s.control('resume')
    s.core.execution_status('running')
    s.control('stop')
    s.control('stop')
    assert s.controls == ['pause', 'resume', 'pause']
    s.core.execution_status('stopped')
    assert s.core.state == 'idle'


def test_rejected_and_malformed_commands_are_not_relayed():
    s = Simulation()
    s.core.command('{')
    s.command(plan(robot_id='other'))
    s.command(plan(schema_version=2))
    s.command(plan(0))
    s.control('pause')
    assert s.controls == []
    s.start()
    s.command(plan(run_id='other'))
    s.command(dict(schema_version=1, robot_id='a2', run_id='other',
                   request_id='pause', command='pause'))
    assert s.controls == []


def test_default_config_and_global_planning_normal_flow():
    s = Simulation()
    assert s.core.state == 'idle' and s.messages[-1]['global_planner_status'] is None
    s.core.global_status('ready')
    assert s.core.state == 'idle' and s.messages[-1]['global_planner_status'] == 'ready'
    s.start()
    assert s.core.state == 'starting' and s.core.step_state is None
    assert s.core.global_planner_status is None and len(s.sent) == 1
    s.core.global_status('planning')
    s.tick(10000)
    assert s.core.state == 'starting' and len(s.sent) == 1  # No timeout or duplicate send.
    s.core.global_status('path_found')
    assert s.core.state == 'running' and s.core.step_state == 'navigating'
    assert s.core.last_result is None
    s.reach()
    s.tick()
    assert s.core.index == 1 and s.core.global_planner_status is None


@pytest.mark.parametrize('reason', [
    'no_start', 'goal_transform_failed', 'start_not_traversable', 'goal_not_traversable',
    'goal_connection_unsafe', 'disconnected', 'timeout', 'no_path', 'internal_error',
])
def test_global_failure_aborts_and_releases_goal_for_new_run(reason):
    s = Simulation()
    s.start()
    raw = 'planning_failed: ' + reason + '; 请检查规划输入'
    s.core.global_status(raw)  # Early validation failures need not emit planning first.
    assert s.core.state == 'idle' and s.core.token is None
    status = s.terminal('failed')
    assert status['message'] == raw and status['global_planner_status'] == raw
    assert status['last_step_result']['reason'] == raw
    assert status['last_step_result']['outcome'] == 'failed'
    assert status['target'] is None and status['remaining_distance_m'] is None
    s.core.execution_status('goal_reached')
    s.tick(10000)
    assert len(s.sent) == 1 and s.core.state == 'idle'
    s.terminal('failed')
    s.start(plan(run_id='new', request_id='new'))
    assert s.core.state == 'starting' and len(s.sent) == 2
    assert s.core.last_result is None and s.core.global_planner_status is None


def test_global_map_fault_is_reported_without_run_and_remains_latched():
    s = Simulation()
    raw = 'map_load_failed: 地图加载失败; /map.pcd; invalid PCD'
    s.core.global_status(raw)
    assert s.core.state == 'error' and s.core.run_id is None
    assert s.messages[-1]['global_planner_status'] == raw
    s.core.global_status('ready')
    s.command(plan())
    assert s.core.state == 'error' and s.core.message == raw and not s.sent


def test_global_map_fault_during_run_records_failed_step():
    s = Simulation()
    s.start()
    raw = 'map_load_failed: missing map'
    s.core.global_status(raw)
    assert s.core.state == 'error' and s.core.last_result['reason'] == raw
    s.tick(10000)
    assert len(s.sent) == 1


@pytest.mark.parametrize('first', ['global', 'local'])
def test_global_and_local_completion_share_one_result_and_dwell(first):
    s = Simulation()
    s.start(plan(dwell=3))
    if first == 'global':
        s.core.global_status('goal_reached')
        s.core.execution_status('goal_reached')
    else:
        s.core.execution_status('goal_reached')
        s.core.global_status('goal_reached')
    result = copy.deepcopy(s.core.last_result)
    assert s.core.state == 'running' and s.core.step_state == 'dwelling'
    assert s.core.distance == 0 and len(s.core.results) == 1
    s.core.global_status('planning_failed: no_path; late')
    s.core.global_status('path_found')
    s.core.global_status('planning')
    s.tick(2)
    assert len(s.sent) == 1 and s.core.last_result == result
    assert s.core.step_state == 'dwelling'
    s.tick(1)
    s.tick()
    assert len(s.sent) == 2


def test_pause_preserved_during_planning_and_resume_restores_phase():
    s = Simulation()
    s.start()
    s.control('pause')
    s.core.global_status('planning')
    assert s.core.state == 'canceling' and s.core.waiting_for_plan
    s.core.execution_status('stopped')
    s.control('resume')
    assert s.core.state == 'starting' and s.controls == ['pause']
    s.core.global_status('path_found')
    assert s.core.state == 'starting'
    s.core.path_available()
    assert s.controls == ['pause', 'resume']
    s.control('pause')
    s.core.execution_status('stopped')
    s.core.execution_status('running')
    assert s.core.state == 'paused'
    s.control('resume')
    assert s.core.state == 'starting'
    s.core.execution_status('running')
    assert s.core.state == 'running' and len(s.sent) == 1


def test_global_arrival_during_pause_then_full_dwell_on_resume():
    s = Simulation()
    s.start(plan(dwell=3))
    s.control('pause')
    s.core.execution_status('stopped')
    s.core.global_status('goal_reached')
    assert s.core.state == 'paused' and s.core.last_result['outcome'] == 'reached'
    assert s.core.distance == 0
    s.tick(100)
    assert len(s.sent) == 1
    s.control('resume')
    s.tick(2)
    assert s.core.step_state == 'dwelling' and len(s.sent) == 1
    s.tick(1)
    s.tick()
    assert len(s.sent) == 2


def test_unknown_idle_and_terminal_global_feedback_only_updates_raw_status():
    s = Simulation()
    s.core.global_status('planning_failed: no_start; idle')
    assert s.core.state == 'idle' and s.core.last_result is None
    s.start(plan(1))
    s.core.global_status('unknown planner detail')
    assert s.core.state == 'starting' and s.messages[-1]['global_planner_status']
    s.core.execution_status('running')
    s.core.global_status('planning')  # Cross-topic delayed feedback must not regress navigation.
    assert s.core.state == 'running'
    s.core.global_status('goal_reached')
    s.tick()
    assert s.core.state == 'idle'
    result = copy.deepcopy(s.terminal('succeeded'))
    for raw in ('planning', 'path_found', 'goal_reached', 'planning_failed: late'):
        s.core.global_status(raw)
        assert s.core.state == 'idle' and s.core.last_result is None
        assert s.terminal('succeeded') == result
        assert s.messages[-1]['global_planner_status'] == raw


@pytest.mark.parametrize('ending', ['succeeded', 'canceled', 'failed'])
def test_terminal_report_once_then_empty_idle_even_with_retries_and_late_feedback(ending):
    s = Simulation()
    p = plan(1)
    s.start(p)
    if ending == 'succeeded':
        s.reach()
        s.tick()
    elif ending == 'canceled':
        s.control('stop')
        s.core.execution_status('stopped')
    else:
        s.core.global_status('planning_failed: no_path; failed once')
    final = copy.deepcopy(s.terminal(ending))
    assert final['run_id'] == 'run1' and final['mission_id'] == 'mission1'
    assert final['step_index'] == 0 and final['step_total'] == 1
    idle = s.messages[-1]
    assert idle['state'] == 'idle' and idle['message'] == ''
    for field in ('run_id', 'request_id', 'mission_id', 'step_state', 'step_index',
                  'step_total', 'current_index', 'total', 'lap', 'current_waypoint_id',
                  'target', 'last_step_result', 'remaining_distance_m'):
        assert idle[field] is None
    for _ in range(3):
        s.tick(1)
        assert s.messages[-1]['state'] == 'idle'
    s.command(p)  # A retry of a completed start must not replay its terminal event.
    s.control('stop')
    s.core.execution_status('goal_reached')
    s.core.global_status('goal_reached')
    s.core.global_status('planning_failed: no_path; late')
    assert s.core.state == 'idle' and s.terminal(ending) == final
    s.start(plan(run_id='new', request_id='new'))
    assert s.core.state == 'starting'


def test_safety_idle_retains_unfinished_goal_until_late_global_failure_releases_it():
    s = Simulation()
    s.start()
    s.core.set_estop(True)
    s.core.set_estop(False)
    assert s.core.state == 'idle' and s.core.token is not None
    s.command(plan(run_id='new', request_id='new'))
    assert s.messages[-1]['command']['reason'] == 'run_active'
    s.core.global_status('planning_failed: no_path; old goal ended')
    assert s.core.token is None and s.core.state == 'idle'
    s.terminal('failed')
    s.start(plan(run_id='new', request_id='new'))
    assert len(s.sent) == 2


def test_error_remains_latched_and_repeats_heartbeat():
    s = Simulation()
    s.core.global_status('map_load_failed: missing map')
    s.tick(1)
    assert s.core.state == 'error'
    assert len([m for m in s.messages if m['state'] == 'error']) >= 2


@pytest.mark.parametrize('state', ['idle', 'running', 'goal_reached', 'error', 'stopped'])
def test_json_local_status_preserves_multiple_unknown_reasons(state):
    s = Simulation()
    s.start(plan(dwell=10))
    reasons = ['astar_no_path', '未知原因; detail', 'optimize_failed']
    s.core.local_status_json(json.dumps({'state': state, 'msgs': reasons}))
    assert s.messages[-1]['local_planner_status'] == {'state': state, 'msgs': reasons}
    assert s.core.fault is None


@pytest.mark.parametrize('raw', [
    'running', '{', 'null', '[]', '1', '"running"',
    '{"state":"running"}', '{"msgs":[]}', '{"state":"unknown","msgs":[]}',
    '{"state":[],"msgs":[]}', '{"state":"error","msgs":"astar_no_path"}',
    '{"state":"error","msgs":[1]}', '{"state":"error","msgs":[null]}',
    '{"state":"error","msgs":[true]}',
])
def test_invalid_local_json_changes_nothing(raw):
    s = Simulation()
    s.start()
    before = s.core.snapshot()
    count = len(s.messages)
    s.core.local_status_json(raw)
    assert s.core.snapshot() == before and len(s.messages) == count
    assert s.warnings


@pytest.mark.parametrize('recovery', ['running', 'goal_reached'])
def test_local_error_recovers_same_goal_without_failed_result(recovery):
    s = Simulation()
    s.start(plan(dwell=10))
    s.core.execution_status('running')
    token = s.core.token
    s.core.local_status_json('{"state":"error","msgs":["astar_no_path","new_reason"]}')
    error = s.messages[-1]
    assert error['state'] == 'error' and error['target']['step_index'] == 0
    assert error['step_state'] == 'navigating' and 'new_reason' in error['message']
    assert s.core.fault is None and s.core.last_result is None
    s.tick(10000)
    assert len(s.sent) == 1 and s.core.token == token and s.messages[-1]['state'] == 'error'
    s.core.execution_status('error', ['changed_reason'])
    assert 'changed_reason' in s.messages[-1]['message']
    s.core.execution_status(recovery)
    assert s.messages[-1]['state'] == 'running' and s.core.fault is None
    assert not any(m['state'] == 'failed' for m in s.messages)
    if recovery == 'running':
        assert s.core.token == token and s.core.last_result is None
    else:
        assert s.core.step_state == 'dwelling' and s.core.last_result['outcome'] == 'reached'
    assert s.messages[-1]['local_planner_status'] == {'state': recovery, 'msgs': []}


@pytest.mark.parametrize('intent', ['pause', 'stop'])
@pytest.mark.parametrize('source', ['local', 'global'])
def test_arrival_races_with_pending_control(intent, source):
    s = Simulation()
    s.start(plan(dwell=1))
    s.control(intent)
    assert s.core.state == 'canceling'
    if source == 'local':
        s.core.execution_status('goal_reached')
    else:
        s.core.global_status('goal_reached')
    result = copy.deepcopy(s.core.last_result)
    assert result['outcome'] == 'reached' and s.core.state == 'canceling'
    assert s.core.step_state is None and s.core.distance == 0
    s.core.execution_status('goal_reached')
    s.tick(100)
    assert len(s.sent) == 1 and s.core.last_result == result
    s.core.execution_status('stopped')
    if intent == 'stop':
        assert s.core.state == 'idle' and s.terminal('canceled')['last_step_result'] == result
    else:
        assert s.core.state == 'paused' and s.core.last_result == result
        s.control('resume')
        assert s.controls == ['pause']  # Dwell does not restart the old trajectory.
        s.tick(1)
        s.tick()
        assert len(s.sent) == 2 and s.controls == ['pause']
        s.core.global_status('path_found')
        assert s.core.state == 'starting' and s.controls == ['pause']
        s.core.path_available()
        assert s.controls == ['pause', 'resume']


def test_local_recovery_never_unpauses_manual_pause():
    s = Simulation()
    s.start()
    s.control('pause')
    s.core.execution_status('error', ['emergency_safety'])
    s.core.execution_status('running')
    assert s.core.state == 'canceling'
    s.core.execution_status('stopped')
    s.core.execution_status('error', ['max_replan_fail'])
    s.core.execution_status('running')
    s.core.global_status('path_found')
    assert s.core.state == 'paused' and s.core.fault is None
    s.core.execution_status('goal_reached')
    assert s.core.state == 'paused' and s.core.last_result['outcome'] == 'reached'


def test_external_stopped_pauses_and_stop_supersedes_pause():
    s = Simulation()
    s.start()
    s.core.execution_status('running')
    s.core.execution_status('stopped')
    assert s.core.state == 'paused' and s.controls == []
    s.control('stop')
    assert s.core.state == 'canceling' and s.controls == ['pause']
    s.control('pause')
    s.core.execution_status('stopped')
    assert s.core.state == 'idle'
    s.terminal('canceled')


def test_cancel_intent_dominates_global_failure_and_releases_only_on_stopped():
    s = Simulation()
    s.start()
    s.control('pause')
    s.control('stop')
    assert s.controls == ['pause']
    s.core.global_status('planning_failed: no_path')
    s.core.execution_status('running')
    s.tick(100)
    assert s.core.state == 'canceling' and s.core.token is not None
    s.core.execution_status('stopped')
    assert s.core.state == 'idle' and s.core.token is None
    s.terminal('canceled')
    assert not any(m['state'] == 'failed' for m in s.messages)


def test_new_goal_while_stopped_requires_path_and_running_ack():
    s = Simulation()
    s.core.execution_status('stopped')
    s.start()
    assert s.core.state == 'starting' and s.controls == [] and len(s.sent) == 1
    s.core.global_status('path_found')
    s.core.execution_status('running')  # Cannot bypass the matching-path gate.
    assert s.core.state == 'starting' and s.controls == []
    s.core.path_available()
    assert s.controls == ['resume'] and s.core.state == 'starting'
    s.core.path_available()
    s.core.global_status('path_found')
    assert s.controls == ['resume'] and s.core.state == 'starting'
    s.core.execution_status('running')
    assert s.core.state == 'running'


def test_local_error_holds_dwell_until_recovered():
    s = Simulation()
    s.start(plan(dwell=2))
    s.reach()
    result = copy.deepcopy(s.core.last_result)
    s.core.execution_status('error', ['unknown_reason'])
    s.tick(100)
    assert len(s.sent) == 1 and s.core.last_result == result
    assert s.messages[-1]['state'] == 'error' and s.core.step_state == 'dwelling'
    s.core.execution_status('running')
    s.tick()
    s.tick()
    assert len(s.sent) == 2


def test_safety_abort_releases_old_goal_on_stopped_confirmation():
    s = Simulation()
    s.start()
    s.core.set_estop(True)
    assert s.core.state == 'idle' and s.core.token is not None
    assert s.controls == ['pause']
    s.core.execution_status('stopped')
    assert s.core.token is None
    s.core.set_estop(False)
    s.start(plan(run_id='new', request_id='new'))
    assert s.core.state == 'starting' and len(s.sent) == 2
    assert s.controls == ['pause']
    s.core.path_available()
    assert s.controls == ['pause', 'resume']


def test_global_failure_during_pause_waits_for_stopped_then_fails():
    s = Simulation()
    s.start()
    s.control('pause')
    s.core.global_status('planning_failed: detail')
    assert s.core.state == 'canceling'
    s.core.execution_status('stopped')
    assert s.core.state == 'idle' and s.core.token is None
    assert s.terminal('failed')['last_step_result']['reason'] == 'planning_failed: detail'
