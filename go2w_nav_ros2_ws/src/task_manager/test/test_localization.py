"""Localization initialization, freshness and mission safety gates."""

import copy

import pytest

from test_mission_executor import Simulation, plan


def initialized():
    s = Simulation(auto_localize=False)
    s.localize('succeed')
    s.tick(1)
    s.localize('succeed')
    assert s.core.state == 'idle'
    return s


def test_startup_and_ready_heartbeat_never_publish_idle():
    s = Simulation(auto_localize=False)
    assert s.messages[0]['state'] == 'uninitialized_location'
    assert s.messages[0]['localization_status'] is None
    assert s.core.run_id is None
    s.tick(.49)
    assert len(s.messages) == 1
    s.tick(.01)
    assert len(s.messages) == 2
    s.localize('ready')
    assert s.messages[-1]['localization_status'] == {'state': 'ready', 'msgs': []}
    s.tick(100)
    for raw in ('ready', 'planning', 'path_found', 'goal_reached', 'planning_failed: late'):
        s.core.global_status(raw)
    for state in ('idle', 'running', 'goal_reached', 'stopped', 'error'):
        s.core.execution_status(state)
    assert all(m['state'] == 'uninitialized_location' for m in s.messages)
    assert not s.sent and not s.controls and s.core.last_result is None


def test_early_start_is_rejected_without_queue_and_requires_resubmission():
    s = Simulation(auto_localize=False)
    s.start()
    assert s.messages[-1]['command']['reason'] == 'localization_not_ready'
    assert s.core.accepted_starts == {} and s.core.steps == []
    assert s.core.token is None and not s.sent and not s.controls
    s.localize('succeed')
    assert s.core.state == 'uninitialized_location'
    s.localize('succeed')
    s.tick()
    assert s.core.state == 'idle' and not s.sent
    s.start()  # The previously rejected run/request can now be submitted.
    assert len(s.sent) == 1 and s.core.state == 'starting'


@pytest.mark.parametrize('state', ['ready', 'error', 'initialized_failed', 'failed'])
def test_non_success_resets_confirmation_without_creating_mission_failure(state):
    s = Simulation(auto_localize=False)
    s.localize('succeed')
    s.localize(state, ['unknown_reason', '中文提示'])
    assert s.core.state == 'uninitialized_location' and s.core.fault is None
    assert s.core.localization_successes == 0
    assert s.messages[-1]['localization_status']['msgs'] == ['unknown_reason', '中文提示']
    assert 'unknown_reason' in s.messages[-1]['message']
    s.tick(100)
    assert 'timed out' not in s.core.message  # Event states have no heartbeat requirement.
    s.localize('succeed')
    assert not s.core.localization_ready
    s.localize('succeed')
    assert s.core.state == 'idle' and s.core.localization_ready
    assert not any(m['state'] == 'failed' for m in s.messages)


def test_confirmation_timeout_and_exact_three_second_boundary():
    s = Simulation(auto_localize=False)
    s.localize('succeed')
    s.tick(3)
    s.localize('succeed')
    assert s.core.localization_ready
    s.tick(3)
    assert s.core.state == 'idle'
    s.tick(.001)
    assert s.core.state == 'uninitialized_location'
    assert 'timed out' in s.core.message and s.core.last_localization_success is None
    assert s.messages[-1]['localization_status'] == {'state': 'succeed', 'msgs': []}
    s.localize('succeed')
    assert not s.core.localization_ready
    s.tick(3.001)
    assert s.core.localization_successes == 0
    s.localize('succeed')
    assert not s.core.localization_ready
    s.localize('succeed')
    assert s.core.state == 'idle'
    assert not s.controls and not any(m['state'] == 'failed' for m in s.messages)


def test_late_success_before_timer_cannot_mask_timeout_or_unlock_with_one_message():
    s = initialized()
    s.start()
    s.time += 3.001  # No timer tick before this late callback.
    s.localize('succeed')
    assert s.core.state == 'uninitialized_location' and not s.core.localization_ready
    assert 'timed out' in s.terminal('failed')['message']
    assert s.controls == ['pause']
    s.localize('succeed')
    assert s.core.state == 'idle' and s.core.token is not None
    assert len(s.sent) == 1  # Recovery does not restart the failed run.


@pytest.mark.parametrize('state', ['ready', 'error', 'initialized_failed', 'failed'])
@pytest.mark.parametrize('phase', ['starting', 'navigating', 'dwelling', 'paused', 'canceling'])
def test_localization_event_aborts_each_execution_phase_once(state, phase):
    s = initialized()
    s.start(plan(dwell=10))
    if phase == 'navigating':
        s.core.execution_status('running')
    elif phase == 'dwelling':
        s.reach()
    elif phase in ('paused', 'canceling'):
        s.control('pause')
        if phase == 'paused':
            s.core.execution_status('stopped')
    controls = len(s.controls)
    s.localize(state, ['localization_lost', '未知原因'])
    final = s.terminal('failed')
    assert final['run_id'] == 'run1' and final['step_index'] == 0
    assert final['localization_status'] == {
        'state': state, 'msgs': ['localization_lost', '未知原因']}
    assert '未知原因' in final['message']
    assert s.core.state == 'uninitialized_location' and s.core.fault is None
    assert len(s.controls) == controls + (phase != 'canceling')
    assert s.controls[-1] == 'pause'
    s.localize(state, ['localization_lost', '未知原因'])
    s.core.execution_status('goal_reached')
    s.core.global_status('path_found')
    s.tick(100)
    assert s.terminal('failed') == final and len(s.sent) == 1
    assert s.messages[-1]['state'] == 'uninitialized_location'
    assert not any(m['state'] == 'idle' for m in s.messages[s.messages.index(final) + 1:])


@pytest.mark.parametrize('release', [
    'stopped', 'local_arrival', 'global_arrival', 'global_failure',
])
def test_recovery_requires_new_run_and_old_execution_release(release):
    s = initialized()
    s.start()
    s.localize('error', ['sensor_data_timeout'])
    s.command(plan())  # Accepted start retry stays idempotent while gated.
    assert 'command' not in s.messages[-1] and len(s.sent) == 1
    s.command(plan(run_id='new', request_id='new'))
    assert s.messages[-1]['command']['reason'] == 'localization_not_ready'
    s.localize('succeed')
    s.localize('succeed')
    assert s.core.state == 'idle' and s.core.token is not None
    s.start(plan(run_id='new', request_id='new'))
    assert s.messages[-1]['command']['reason'] == 'run_active' and len(s.sent) == 1
    if release == 'stopped':
        s.core.execution_status('stopped')
    elif release == 'local_arrival':
        s.core.execution_status('goal_reached')
    else:
        s.core.global_status('goal_reached' if release == 'global_arrival'
                             else 'planning_failed: old request ended')
    assert s.core.token is None
    s.start(plan(run_id='new', request_id='new'))
    assert len(s.sent) == 2 and s.core.run_id == 'new'
    assert s.core.last_result is None


@pytest.mark.parametrize('callback', ['command', 'local', 'global', 'path'])
def test_overdue_heartbeat_blocks_feedback_and_commands_before_timer(callback):
    s = initialized()
    s.start(plan(1))
    s.time += 3.01
    if callback == 'command':
        s.control('resume')
    elif callback == 'local':
        s.core.execution_status('goal_reached')
    elif callback == 'global':
        s.core.global_status('goal_reached')
    else:
        s.core.path_available()
    assert s.core.state == 'uninitialized_location'
    assert s.terminal('failed')['last_step_result']['outcome'] == 'failed'
    assert len(s.sent) == 1 and s.controls == ['pause']
    assert not any(m['state'] == 'succeeded' for m in s.messages)


def test_success_timeout_aborts_even_when_tf_is_fresh_and_keeps_original_status():
    s = initialized()
    s.start()
    s.tick(3.01, ok=True, age=0)
    assert s.core.state == 'uninitialized_location'
    assert s.terminal('failed')['localization_status'] == {'state': 'succeed', 'msgs': []}
    assert 'heartbeat timed out' in s.terminal('failed')['last_step_result']['reason']
    s.core.execution_status('stopped')
    assert s.core.token is None and s.core.state == 'uninitialized_location'
    s.tick(20)
    s.terminal('failed')


@pytest.mark.parametrize('raw', [
    '{', 'ready', 'null', '[]', '1', '"succeed"', '{"state":"succeed"}', '{"msgs":[]}',
    '{"state":[],"msgs":[]}', '{"state":"unknown","msgs":[]}',
    '{"state":"succeed","msgs":null}', '{"state":"succeed","msgs":"ok"}',
    '{"state":"succeed","msgs":[null]}', '{"state":"succeed","msgs":[1]}',
    '{"state":"succeed","msgs":[true]}',
])
def test_invalid_localization_json_changes_nothing_and_does_not_refresh_success(raw):
    s = initialized()
    s.time += 2
    before = copy.deepcopy(s.core.snapshot())
    count, received = len(s.messages), s.core.last_localization_success
    s.core.localization_status_json(raw)
    assert s.core.snapshot() == before and len(s.messages) == count
    assert s.core.last_localization_success == received and s.warnings
    s.tick(1.01)
    assert s.core.state == 'uninitialized_location'


def test_reason_changes_publish_immediately_and_unchanged_success_is_not_extra_status():
    s = initialized()
    count = len(s.messages)
    s.localize('succeed')
    assert len(s.messages) == count
    s.localize('error', ['one'])
    s.localize('error', ['one', 'two'])
    assert len(s.messages) == count + 2 and 'two' in s.messages[-1]['message']
    s.localize('error', ['one', 'two'])
    assert len(s.messages) == count + 2


@pytest.mark.parametrize('fault', ['map_load_failed: broken map', 'publish goal: transport failed'])
def test_localization_cannot_clear_latched_fault(fault):
    s = Simulation(auto_localize=False)
    s.core.error(fault)
    s.localize('ready')
    s.localize('succeed')
    s.localize('succeed')
    s.tick(3.1)
    assert s.core.state == 'error' and s.core.message == fault and s.core.fault == fault
    s.localize('succeed')
    s.localize('succeed')
    s.start()
    assert s.core.state == 'error' and not s.sent


@pytest.mark.parametrize('config', [
    {'localization_status_topic': ''}, {'localization_status_timeout_sec': 0},
    {'localization_status_timeout_sec': -1}, {'localization_status_timeout_sec': True},
    {'localization_status_timeout_sec': float('nan')},
    {'localization_status_timeout_sec': float('inf')},
])
def test_localization_parameter_validation(config):
    s = Simulation(auto_localize=False, **config)
    assert s.core.state == 'error' and 'localization_status' in s.core.message


def test_custom_heartbeat_timeout():
    s = Simulation(auto_localize=False, localization_status_timeout_sec=.5)
    s.localize('succeed')
    s.tick(.4)
    s.localize('succeed')
    assert s.core.state == 'idle'
    s.tick(.51)
    assert s.core.state == 'uninitialized_location'
