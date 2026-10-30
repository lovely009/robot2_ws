"""Standard ROS diagnostic messages for managed robot supervision.

Only explicitly selected public state is exported. Lease tokens and submitted
messages are never copied into diagnostics.
"""
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def _value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return '; '.join(str(item) for item in value)
    return str(value)


def supervisor_diagnostics(state, robot_id, model_name, *, node_name='robot_supervisor',
                           last_input=None, last_rejection=None):
    """Describe readiness and the latest ingress result using ROS status levels."""
    fault = bool(state.get('fault_latched', False))
    ready = bool(state.get('ready', False))
    level = DiagnosticStatus.ERROR if fault else (
        DiagnosticStatus.OK if ready else DiagnosticStatus.WARN)
    blockers = list(state.get('blockers', []))
    health = DiagnosticStatus(
        level=level, name=node_name + '/readiness', hardware_id=robot_id,
        message=('; '.join(blockers) if blockers else
                 ('not ready' if not ready else
                  ('ready; armed' if state.get('armed') else 'ready; disarmed'))))
    fields = {
        'robot_id': robot_id, 'model_name': model_name,
        'state': state.get('state', 0), 'ready': ready,
        'armed': bool(state.get('armed', False)), 'fault_latched': fault,
        'owner': state.get('owner', ''), 'mode': state.get('mode', 0),
        'session_id': state.get('session_id', 0),
        'lease_remaining_ms': state.get('lease_remaining_ms', 0),
        'calibration_id': state.get('calibration_id', ''),
        'config_hash': state.get('config_hash', ''),
        'motor_ids': state.get('motor_ids', []), 'blockers': blockers,
    }
    health.values = [KeyValue(key=key, value=_value(value)) for key, value in fields.items()]

    last_input = last_input or {}
    rejected = last_input.get('accepted') is False
    control = DiagnosticStatus(
        level=DiagnosticStatus.WARN if rejected else DiagnosticStatus.OK,
        name=node_name + '/control_input', hardware_id=robot_id,
        message=(str(last_input.get('reason', 'input rejected')) if rejected else
                 ('last input accepted' if last_input else 'no input received')))
    control_fields = {
        'last_channel': last_input.get('channel', ''),
        'last_sequence': last_input.get('sequence', 0),
        'last_accepted': last_input.get('accepted', ''),
    }
    if last_rejection:
        control_fields.update({
            'last_rejection_channel': last_rejection.get('channel', ''),
            'last_rejection_sequence': last_rejection.get('sequence', 0),
            'last_rejection_reason': last_rejection.get('reason', ''),
            'last_rejection_time_ns': last_rejection.get('time_ns', 0),
        })
    control.values = [KeyValue(key=key, value=_value(value))
                      for key, value in control_fields.items()]
    return [health, control]
