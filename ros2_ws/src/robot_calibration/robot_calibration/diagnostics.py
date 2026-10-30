"""Standard ROS diagnostic projection of a commissioning validation result."""
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def calibration_diagnostic(report, *, node_name='calibration_validator'):
    ready = report.valid and report.commissioned and not report.blockers
    status = DiagnosticStatus(
        level=DiagnosticStatus.OK if ready else DiagnosticStatus.WARN,
        name=node_name + '/commissioning', hardware_id=report.robot_id,
        message=('configuration and commissioning record accepted' if ready else
                 '; '.join(report.blockers) or 'commissioning is not accepted'))
    fields = {
        'robot_id': report.robot_id, 'model_name': report.model_name,
        'valid': 'true' if report.valid else 'false',
        'commissioned': 'true' if report.commissioned else 'false',
        'calibration_id': report.calibration_id, 'config_hash': report.config_hash,
        'motor_ids': '; '.join(str(item) for item in report.motor_ids),
        'blockers': '; '.join(report.blockers),
    }
    status.values = [KeyValue(key=key, value=value) for key, value in fields.items()]
    return status
