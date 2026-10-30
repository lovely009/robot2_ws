"""Exercise rosbridge with real generated standard Action request types."""
import pytest


def test_standard_trajectory_action_services_and_nested_payload_resolve():
    pytest.importorskip('rosbridge_library')
    from control_msgs.action import FollowJointTrajectory
    from rosbridge_library.internal import message_conversion, ros_loader
    from web_host.rosbridge_shim import _patch_standard_trajectory_action

    _patch_standard_trajectory_action()
    _patch_standard_trajectory_action()  # startup patch remains idempotent
    service = ros_loader.get_service_class('control_msgs/action/FollowJointTrajectory_SendGoal')
    assert service is FollowJointTrajectory.Impl.SendGoalService
    request = service.Request()
    payload = {'goal_id': {'uuid': list(range(16))}, 'goal': {
        'trajectory': {'joint_names': ['joint'], 'points': [
            {'positions': [.1], 'time_from_start': {'sec': 0, 'nanosec': 0}},
            {'positions': [.2], 'time_from_start': {'sec': 1, 'nanosec': 0}},
        ]}, 'path_tolerance': [], 'goal_tolerance': [],
        'goal_time_tolerance': {'sec': 1, 'nanosec': 0}}}
    message_conversion.populate_instance(payload, request)
    assert list(request.goal_id.uuid) == list(range(16))
    assert list(request.goal.trajectory.points[-1].positions) == [.2]
    result_type = ros_loader.get_service_class('control_msgs/action/FollowJointTrajectory_GetResult')
    result = result_type.Response(status=4, result=FollowJointTrajectory.Result(error_code=0))
    converted = message_conversion.extract_values(result)
    assert converted['status'] == 4 and converted['result']['error_code'] == 0
