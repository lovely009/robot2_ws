"""ROS diagnostic levels and public-state projection, without any transport."""
import pytest
from diagnostic_msgs.msg import DiagnosticStatus

from robot_supervisor.diagnostics import supervisor_diagnostics


def _state(**changes):
    values = {
        'state': 2, 'ready': True, 'armed': False, 'fault_latched': False,
        'owner': 'operator', 'mode': 1, 'session_id': 1_800_000_000_000,
        'lease_remaining_ms': 800, 'calibration_id': 'assembly-v3',
        'config_hash': 'a' * 64, 'motor_ids': [1, 2], 'blockers': [],
        'lease_id': 'never-publish-this-token', 'arbitrary_internal': 'private',
    }
    values.update(changes)
    return values


@pytest.mark.parametrize('changes,level,message', [
    ({}, DiagnosticStatus.OK, 'ready; disarmed'),
    ({'armed': True}, DiagnosticStatus.OK, 'ready; armed'),
    ({'ready': False, 'blockers': ['calibration_not_commissioned']},
     DiagnosticStatus.WARN, 'calibration_not_commissioned'),
    ({'ready': False}, DiagnosticStatus.WARN, 'not ready'),
    ({'ready': False, 'fault_latched': True, 'blockers': ['control_lease_expired']},
     DiagnosticStatus.ERROR, 'control_lease_expired'),
])
def test_readiness_levels_and_public_values(changes, level, message):
    health, control = supervisor_diagnostics(_state(**changes), 'robot-01', 'arm')
    assert health.level == level
    assert health.message == message
    assert health.name == 'robot_supervisor/readiness'
    assert health.hardware_id == 'robot-01'
    data = {item.key: item.value for item in health.values}
    assert data['session_id'] == '1800000000000'
    assert data['lease_remaining_ms'] == '800'
    assert data['calibration_id'] == 'assembly-v3'
    assert data['config_hash'] == 'a' * 64
    assert data['motor_ids'] == '1; 2'
    assert 'lease_id' not in data
    assert 'arbitrary_internal' not in data
    assert 'never-publish-this-token' not in repr((health, control))


def test_rejection_is_visible_and_preserved_after_next_accepted_input():
    rejection = {'channel': 'command', 'sequence': 9, 'accepted': False,
                 'reason': 'control_lease_mismatch', 'time_ns': 123,
                 'lease_id': 'never-publish-this-token'}
    _, control = supervisor_diagnostics(_state(), 'robot-01', 'arm',
        last_input=rejection, last_rejection=rejection)
    assert control.level == DiagnosticStatus.WARN
    assert control.message == 'control_lease_mismatch'
    _, control = supervisor_diagnostics(_state(), 'robot-01', 'arm',
        last_input={'channel': 'command', 'sequence': 10, 'accepted': True},
        last_rejection=rejection)
    assert control.level == DiagnosticStatus.OK
    values = {item.key: item.value for item in control.values}
    assert values['last_sequence'] == '10'
    assert values['last_rejection_sequence'] == '9'
    assert values['last_rejection_reason'] == 'control_lease_mismatch'
    assert values['last_rejection_time_ns'] == '123'
    assert 'never-publish-this-token' not in repr(control)


def test_namespaced_diagnostic_names_do_not_collide():
    health, control = supervisor_diagnostics(_state(), 'robot-01', 'arm',
                                            node_name='robot1/robot_supervisor')
    assert health.name == 'robot1/robot_supervisor/readiness'
    assert control.name == 'robot1/robot_supervisor/control_input'
