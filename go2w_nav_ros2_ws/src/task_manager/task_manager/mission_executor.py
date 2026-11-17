"""ROS-independent mission state machine; all entry points run serially."""

import copy
import json
import math


DEFAULTS = {
    'robot_id': 'go2w', 'command_topic': '/mission/command',
    'status_topic': '/mission/status', 'goal_topic': '/goal_3d',
    'control_topic': '/mission/control',
    'global_planner_status_topic': '/global_planner/status',
    'local_planner_status_topic': '/local_planner/status',
    'localization_status_topic': '/localization/status',
    'localization_status_timeout_sec': 3.0,
    'global_path_topic': '/global_path', 'odom_topic': '/loc/robo/odom',
    'global_path_z_offset': 0.5,
    'frame_id': 'map', 'robot_frame': 'base', 'status_rate_hz': 2.0,
    'tf_timeout_sec': 2.0,
    'dwell_default_sec': 1.0,
}
ACTIVE = ('starting', 'running', 'canceling', 'paused')
TERMINAL = ('succeeded', 'canceled', 'failed')


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def valid_config(config):
    for key in ('robot_id', 'command_topic', 'status_topic', 'goal_topic',
                'global_planner_status_topic', 'local_planner_status_topic',
                'localization_status_topic',
                'frame_id', 'robot_frame', 'global_path_topic', 'odom_topic', 'control_topic'):
        if not isinstance(config[key], str) or not config[key].strip():
            return 'invalid parameter: ' + key
    for key in ('status_rate_hz', 'tf_timeout_sec', 'localization_status_timeout_sec'):
        if not finite(config[key]) or config[key] <= 0:
            return 'invalid parameter: ' + key
    if not finite(config['global_path_z_offset']):
        return 'invalid parameter: global_path_z_offset'
    if not finite(config['dwell_default_sec']) or not 0 <= config['dwell_default_sec'] <= 600:
        return 'invalid parameter: dwell_default_sec'
    return None


def validate_start(payload):
    mission = payload.get('mission')
    if not isinstance(mission, dict):
        return False
    for key in ('id', 'name', 'map_id'):
        if not isinstance(mission.get(key), str):
            return False
    if not mission['id'] or not mission['map_id']:
        return False
    if mission.get('mode') not in ('once', 'loop', 'pingpong'):
        return False
    if type(mission.get('loop_count')) is not int or mission['loop_count'] < 1:
        return False
    if type(mission.get('steps_total')) is not int:
        return False
    if 'frame_id' in payload and (
            not isinstance(payload['frame_id'], str) or not payload['frame_id'].strip()):
        return False
    steps = mission.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 1000:
        return False
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            return False
        for key in ('step_index', 'lap', 'point_index', 'point_total'):
            if type(step.get(key)) is not int:
                return False
        if (step['step_index'] != index or step['lap'] < 1 or
                step['point_total'] < 1 or not 0 <= step['point_index'] < step['point_total']):
            return False
        if mission['mode'] == 'once' and step['lap'] != 1:
            return False
        if not isinstance(step.get('name'), str):
            return False
        if step.get('waypoint_id') is not None and not isinstance(step['waypoint_id'], str):
            return False
        if not all(finite(step.get(key)) for key in ('x', 'y', 'z', 'yaw', 'dwell_sec')):
            return False
        if not -math.pi <= step['yaw'] <= math.pi or not 0 <= step['dwell_sec'] <= 600:
            return False
    return True


class MissionExecutor:
    """Goal publisher provides send(step, frame); execution status drives progression."""

    def __init__(self, navigation, publish, monotonic, stamp, warning, config=None, control=None):
        self.config = dict(DEFAULTS, **(config or {}))
        self.nav, self.publish, self.now, self.stamp = navigation, publish, monotonic, stamp
        self.warning = warning
        self.control = control if control is not None else lambda command: None
        self.localization_status = None
        self.localization_ready = False
        self.localization_successes = 0
        self.last_localization_success = None
        self.localization_message = 'waiting for localization status'
        self.state, self.message = 'uninitialized_location', self.localization_message
        self.run_id = self.request_id = self.mission_id = None
        self.steps, self.index, self.results = [], 0, {}
        self.frame = self.config['frame_id']
        self.step_state = self.distance = self.last_result = None
        self.token = None
        self.serial = 0
        self.running_seen = False
        self.waiting_for_plan = False
        self.global_planner_status = None
        self.local_planner_status = None
        self.local_stopped = False
        # Recoverable local errors overlay the saved phase in public snapshots.
        self.local_error = False
        self.control_intent = None
        self.pending_failure = None
        self.resume_pending = self.awaiting_running = self.path_ready = False
        self.dwell_deadline = None
        self.estop = False
        self.accepted_starts = {}
        self.last_tf_ok = self.now()
        self.fault = valid_config(self.config)
        if self.fault:
            self.state, self.message = 'error', self.fault
        self.emit()

    @property
    def step(self):
        return self.steps[self.index] if self.steps else None

    def snapshot(self):
        step = self.step
        target = None
        if step and self.state in ACTIVE:
            target = {key: step.get(key) for key in (
                'step_index', 'point_index', 'waypoint_id', 'name', 'x', 'y', 'z', 'yaw')}
        return copy.deepcopy({
            'schema_version': 1, 'robot_id': self.config['robot_id'],
            'run_id': self.run_id, 'request_id': self.request_id, 'mission_id': self.mission_id,
            'state': ('error' if self.local_error and self.state in ('starting', 'running')
                      else self.state), 'step_state': self.step_state,
            'global_planner_status': self.global_planner_status,
            'local_planner_status': self.local_planner_status,
            'localization_status': self.localization_status,
            'step_index': self.index if step else None,
            'step_total': len(self.steps) if step else None,
            'current_index': step['point_index'] if step else None,
            'total': step['point_total'] if step else None,
            'lap': step['lap'] if step else None,
            'current_waypoint_id': step.get('waypoint_id') if step else None,
            'target': target, 'last_step_result': self.last_result,
            'remaining_distance_m': self.distance if target else None,
            'message': ('local planner error: ' + '; '.join(self.local_planner_status['msgs'])
                        if self.local_error and self.state in ('starting', 'running')
                        else self.message), 'stamp': self.stamp(),
        })

    def emit(self, receipt=None):
        status = self.snapshot()
        if receipt is not None:
            status['command'] = receipt
        self.publish(status)
        self.last_publish = self.now()

    def transition(self, state, message, step_state=None):
        self.state, self.message, self.step_state = state, message, step_state
        if state != 'running':
            reached = self.results.get(self.index, {}).get('outcome') == 'reached'
            self.distance = 0.0 if state in ACTIVE and reached else None
        self.emit()
        if state in TERMINAL:
            self.return_to_idle()

    def return_to_idle(self):
        # The terminal snapshot has already been sent exactly once. Keep only
        # an outstanding navigation token, estop/fault and idempotency history.
        self.state = 'idle' if self.localization_ready else 'uninitialized_location'
        self.message = '' if self.localization_ready else self.localization_message
        self.run_id = self.request_id = self.mission_id = None
        self.steps, self.index, self.results = [], 0, {}
        self.step_state = self.distance = self.last_result = None
        self.frame = self.config['frame_id']
        self.running_seen = self.waiting_for_plan = False
        self.dwell_deadline = None
        self.control_intent = None
        self.pending_failure = None
        self.resume_pending = self.awaiting_running = self.path_ready = False
        self.local_error = False
        self.emit()

    def reject(self, payload, reason):
        self.emit({
            'request_id': payload.get('request_id')
            if isinstance(payload.get('request_id'), str) else None,
            'command': payload.get('command')
            if isinstance(payload.get('command'), str) else None,
            'accepted': False, 'reason': reason,
        })

    def close_localization_gate(self, reason, status_changed=False):
        changed = (status_changed or self.localization_ready or
                   self.localization_successes != 0 or self.localization_message != reason)
        self.localization_ready = False
        self.localization_successes = 0
        self.last_localization_success = None
        self.localization_message = reason
        if self.state in ACTIVE:
            self.safety_abort(reason)
        elif self.state == 'idle':
            self.transition('uninitialized_location', reason)
        elif self.state == 'uninitialized_location':
            self.message = reason
            if changed:
                self.emit()
        elif changed:
            # Node/map faults have priority over initialization diagnostics.
            self.emit()

    def check_localization_timeout(self):
        limit = self.config['localization_status_timeout_sec']
        if (self.last_localization_success is not None and finite(limit) and limit > 0 and
                self.now() - self.last_localization_success > limit):
            self.close_localization_gate('localization succeed heartbeat timed out')

    def localization_status_json(self, text):
        try:
            value = json.loads(text)
            if (not isinstance(value, dict) or
                    value.get('state') not in ('ready', 'succeed', 'error',
                                               'initialized_failed', 'failed') or
                    not isinstance(value.get('msgs'), list) or
                    not all(isinstance(msg, str) for msg in value['msgs'])):
                raise ValueError('expected localization state and string array msgs')
        except (ValueError, TypeError, RecursionError):
            self.warning('Ignore invalid localization JSON status')
            return
        status = {'state': value['state'], 'msgs': list(value['msgs'])}
        changed = status != self.localization_status
        if status['state'] != 'succeed':
            self.localization_status = status
            reason = 'localization ' + status['state']
            if status['state'] == 'ready':
                reason += '; waiting for initialization'
            if status['msgs']:
                reason += ': ' + '; '.join(status['msgs'])
            self.close_localization_gate(reason, changed)
            return
        # A late heartbeat cannot conceal a timeout before the next timer tick.
        self.check_localization_timeout()
        self.localization_status = status
        self.last_localization_success = self.now()
        self.localization_successes = min(2, self.localization_successes + 1)
        self.localization_ready = self.localization_successes == 2
        self.localization_message = ('' if self.localization_ready else
                                     'waiting for subsequent localization succeed heartbeat')
        if self.state == 'uninitialized_location':
            self.transition('idle' if self.localization_ready else 'uninitialized_location',
                            self.localization_message)
        elif changed:
            self.emit()

    def command(self, text):
        try:
            payload = json.loads(text)
        except (ValueError, TypeError, RecursionError):
            self.warning('Ignore invalid JSON command')
            return
        if not isinstance(payload, dict):
            self.reject({}, 'invalid')
            return
        if type(payload.get('schema_version')) is not int or payload['schema_version'] != 1:
            self.reject(payload, 'unsupported_version')
            return
        if payload.get('robot_id') != self.config['robot_id']:
            self.reject(payload, 'robot_mismatch')
            return
        if any(not isinstance(payload.get(k), str) or not payload[k].strip()
               for k in ('run_id', 'request_id', 'command')):
            self.reject(payload, 'invalid')
            return
        cmd = payload['command']
        self.check_localization_timeout()
        if cmd == 'start':
            if self.accepted_starts.get(payload['run_id']) == payload['request_id']:
                self.emit()
            elif not self.localization_ready:
                self.reject(payload, 'localization_not_ready')
            elif self.state in ACTIVE or self.token is not None:
                self.reject(payload, 'run_active')
            elif self.estop:
                self.reject(payload, 'estop_active')
            elif (self.fault or payload['run_id'] in self.accepted_starts or
                  not validate_start(payload)):
                self.reject(payload, 'invalid')
            else:
                self.start(payload)
            return
        if cmd not in ('pause', 'resume', 'stop'):
            self.reject(payload, 'invalid')
        elif payload['run_id'] != self.run_id or self.state not in ACTIVE:
            self.reject(payload, 'run_id_mismatch')
        elif cmd == 'stop' or (cmd == 'pause' and self.state in ('starting', 'running')):
            if self.control_intent is None:
                self.control('pause')
            self.control_intent = 'stop' if cmd == 'stop' else 'pause'
            self.resume_pending = self.awaiting_running = False
            self.dwell_deadline = None
            self.transition('canceling', 'waiting for local stopped confirmation: ' +
                            self.control_intent)
        elif cmd == 'resume' and self.state == 'paused':
            if self.index in self.results:
                # The old trajectory must remain stopped throughout dwell.
                self.enter_dwell()
            elif self.token is not None:
                self.resume_pending = True
                self.transition('starting', 'waiting for matching global path before resume')
                self.maybe_resume()
            else:
                self.prepare()
        elif ((cmd == 'pause' and self.state in ('paused', 'canceling')) or
              (cmd == 'resume' and self.state in ('starting', 'running'))):
            self.emit()
        else:
            self.reject(payload, 'invalid')

    def maybe_resume(self):
        self.check_localization_timeout()
        if (self.localization_ready and self.resume_pending and self.path_ready and
                self.token is not None and
                self.state == 'starting' and self.control_intent is None):
            self.resume_pending = False
            self.awaiting_running = True
            self.control('resume')
            self.transition('starting', 'waiting for local running confirmation')

    def path_available(self):
        """Called only for a fresh path whose endpoint matches the current goal."""
        self.path_ready = True
        self.maybe_resume()

    def start(self, payload):
        mission = payload['mission']
        self.run_id, self.request_id = payload['run_id'], payload['request_id']
        self.accepted_starts[self.run_id] = self.request_id
        self.mission_id = mission['id']
        self.steps = copy.deepcopy(mission['steps'])
        if mission['steps_total'] != len(self.steps):
            self.warning('steps_total differs from steps length; using steps length')
        self.frame = payload.get('frame_id', self.config['frame_id'])
        self.index, self.results = 0, {}
        self.last_result = None
        self.last_tf_ok = self.now()
        self.prepare()

    def prepare(self):
        self.path_ready = False
        self.local_error = False
        self.awaiting_running = False
        self.resume_pending = self.local_stopped
        self.global_planner_status = None
        self.waiting_for_plan = True
        self.dwell_deadline = None
        self.distance = None
        self.transition('starting', 'sending current step to ' + self.config['goal_topic'])

    def tick(self, localization_ok=True, localization_age=None):
        now = self.now()
        self.check_localization_timeout()
        if self.state in ACTIVE:
            if localization_ok:
                self.last_tf_ok = now - (localization_age or 0.0)
            elif (now - self.last_tf_ok >= self.config['tf_timeout_sec'] or
                  (localization_age is not None and
                   localization_age >= self.config['tf_timeout_sec'])):
                self.safety_abort('localization lost')
        if (self.localization_ready and self.state == 'starting' and
                self.token is None and not self.local_error):
            self.serial += 1
            self.token = self.serial
            self.running_seen = False
            self.global_planner_status = None
            try:
                self.nav.send(self.step, self.frame)
                self.transition('starting', 'waiting for global planning result')
            except Exception as exc:
                self.error('publish goal: ' + str(exc))
        elif (self.state == 'running' and self.step_state == 'dwelling' and
              not self.local_error):
            if now >= self.dwell_deadline:
                self.advance()
        rate = self.config['status_rate_hz']
        if self.state == 'idle' or not finite(rate) or rate <= 0:
            rate = 1.0
        if now - self.last_publish >= 1.0 / rate:
            self.emit()

    def navigation_started(self):
        self.waiting_for_plan = False
        if (self.state == 'starting' and not self.resume_pending and
                not self.awaiting_running):
            self.transition('running', 'navigating to ' + self.step['name'], 'navigating')
            return True
        return False

    def local_status_json(self, text):
        try:
            value = json.loads(text)
            if (not isinstance(value, dict) or
                    value.get('state') not in ('idle', 'running', 'goal_reached', 'error',
                                               'stopped') or
                    not isinstance(value.get('msgs'), list) or
                    not all(isinstance(msg, str) for msg in value['msgs'])):
                raise ValueError('expected state and string array msgs')
        except (ValueError, TypeError, RecursionError):
            self.warning('Ignore invalid local planner JSON status')
            return
        self.execution_status(value['state'], value['msgs'])

    def execution_status(self, status, msgs=None):
        self.check_localization_timeout()
        previous = self.snapshot()
        previous.pop('stamp')
        self.local_planner_status = {'state': status, 'msgs': list(msgs or [])}
        if status == 'stopped':
            self.local_stopped = True
            self.local_error = False
            if self.state in ('idle', 'uninitialized_location'):
                # A safety abort also requests pause and retains occupancy until
                # the local planner confirms that the old execution has stopped.
                self.token = None
            if self.state == 'canceling':
                intent = self.control_intent
                self.control_intent = None
                if intent == 'stop':
                    self.token = None
                    self.transition('canceled', 'local planner confirmed stopped')
                elif self.pending_failure is not None:
                    self.record('failed', self.pending_failure)
                    self.token = None
                    self.transition('failed', self.pending_failure)
                else:
                    self.transition('paused', 'local planner confirmed stopped')
            elif self.state in ('starting', 'running'):
                if not (self.resume_pending or self.awaiting_running):
                    self.dwell_deadline = None
                    self.transition('paused', 'local planner stopped externally')
        elif status == 'running':
            self.local_stopped = False
            self.local_error = False
            if self.token is not None and self.state in ACTIVE:
                self.running_seen = True
                # Feedback cannot bypass the path gate or an outstanding pause.
                self.awaiting_running = False
                self.navigation_started()
        elif status == 'goal_reached':
            self.local_error = False
            self.complete_goal()
        elif status == 'error' and self.state in ACTIVE:
            self.local_error = True
        current = self.snapshot()
        current.pop('stamp')
        if current != previous:
            self.emit()

    def complete_goal(self):
        self.check_localization_timeout()
        # Both planners can complete an already-reached goal without running.
        if self.token is None:
            return
        self.token = None
        self.running_seen = self.waiting_for_plan = False
        if self.state in ACTIVE:
            self.local_error = False
            self.resume_pending = self.awaiting_running = False
            self.record('reached', '')
            if self.state in ('paused', 'canceling'):
                self.distance = 0.0
                self.emit()
            else:
                self.enter_dwell()

    def global_status(self, status):
        self.check_localization_timeout()
        self.global_planner_status = status
        if status.startswith('map_load_failed:'):
            self.error(status)
            return
        if (self.state in ('idle', 'uninitialized_location') and self.token is not None and
                (status == 'goal_reached' or status.startswith('planning_failed:'))):
            # A stopped/failed run is no longer exposed in idle, but its old
            # goal must finish before another run can safely be dispatched.
            self.token = None
        if self.token is not None and self.state in ACTIVE:
            if status == 'planning' and self.waiting_for_plan:
                if self.state not in ('paused', 'canceling'):
                    self.transition('starting', 'global planning')
                    return
            elif status == 'path_found':
                if self.navigation_started():
                    return
            elif status == 'goal_reached':
                self.complete_goal()
                return
            elif status.startswith('planning_failed:'):
                if self.state == 'canceling':
                    # Stop wins; a pause must still confirm stopped before a
                    # concurrent planning failure can release the current run.
                    if self.control_intent == 'pause':
                        self.pending_failure = status
                    self.emit()
                    return
                self.record('failed', status)
                self.token = None
                self.waiting_for_plan = False
                self.dwell_deadline = None
                self.transition('failed', status)
                return
        # Health, unknown or late feedback is observable without reviving a run
        # or changing an existing result. Node/map faults remain latched until restart.
        self.emit()

    def enter_dwell(self):
        self.distance = 0.0
        self.dwell_deadline = self.now() + self.step['dwell_sec']
        self.transition('running', 'dwelling at ' + self.step['name'], 'dwelling')

    def safety_abort(self, reason):
        if self.state in ACTIVE:
            self.dwell_deadline = None
            self.record('failed', reason)
            if self.control_intent is None:
                self.control('pause')
            self.transition('failed', reason + '; pause requested; navigation end pending')

    def set_estop(self, active):
        self.estop = active
        if active:
            self.safety_abort('estop')

    def error(self, reason):
        self.fault = reason
        self.record('failed', reason)
        self.transition('error', reason)

    def record(self, outcome, reason):
        if self.step is None or self.index in self.results:
            return
        self.last_result = {
            'step_index': self.step['step_index'], 'point_index': self.step['point_index'],
            'outcome': outcome, 'reason': reason, 'finished_at': self.stamp(),
        }
        self.results[self.index] = self.last_result

    def advance(self):
        if self.index + 1 == len(self.steps):
            self.transition('succeeded', 'mission succeeded')
        else:
            self.index += 1
            self.prepare()
