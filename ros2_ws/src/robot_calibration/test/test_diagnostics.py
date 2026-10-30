from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus
from robot_calibration.diagnostics import calibration_diagnostic


def _report(**changes):
    report = dict(robot_id='robot-01', model_name='arm', valid=True, commissioned=True,
                  calibration_id='assembly-v3', config_hash='a' * 64,
                  motor_ids=[1, 2], blockers=[])
    report.update(changes)
    return SimpleNamespace(**report)


def test_accepted_commissioning_uses_standard_ok_status():
    status = calibration_diagnostic(_report())
    assert status.level == DiagnosticStatus.OK
    assert status.name == 'calibration_validator/commissioning'
    assert status.hardware_id == 'robot-01'
    values = {entry.key: entry.value for entry in status.values}
    assert values['commissioned'] == 'true'
    assert values['calibration_id'] == 'assembly-v3'
    assert values['config_hash'] == 'a' * 64


def test_uncommissioned_or_invalid_configuration_is_warn():
    status = calibration_diagnostic(_report(valid=False, commissioned=False,
        blockers=['assembly is not commissioned', 'm1 identity is not verified']))
    assert status.level == DiagnosticStatus.WARN
    assert 'assembly is not commissioned' in status.message
    assert 'm1 identity is not verified' in status.message
    assert not any('lease' in entry.key for entry in status.values)
