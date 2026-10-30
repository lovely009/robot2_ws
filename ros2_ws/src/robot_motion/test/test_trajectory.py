import copy

import pytest
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_motion.trajectory import TrajectoryCompiler, signed_tick_difference


def goal():
    value = FollowJointTrajectory.Goal()
    value.trajectory.joint_names = ['yaw', 'pitch']
    value.trajectory.points = [
        JointTrajectoryPoint(positions=[0., 0.]),
        JointTrajectoryPoint(positions=[.2, .1], time_from_start=Duration(sec=1)),
    ]
    return value


def test_differential_conversion_preserves_joint_targets_and_mcu_time(model):
    checked = []
    compiler = TrajectoryCompiler(model, checked.append)
    plan = compiler.compile(goal(), 42)
    assert checked == [plan.message]
    assert plan.message.trajectory_id == 42
    assert plan.message.target_ids.tolist() == [8, 9]
    assert [(p.motor_id, p.time_from_start_ms) for p in plan.message.points] == [
        (8, 0), (9, 0), (8, 1000), (9, 1000)]
    assert plan.message.points[-2].position == pytest.approx(.3)
    assert plan.message.points[-1].position == pytest.approx(.1)
    middle = plan.desired(500)
    assert middle['yaw'].position == pytest.approx(.1)
    assert middle['pitch'].position == pytest.approx(.05)
    assert middle['yaw'].velocity == pytest.approx(.3)
    assert plan.desired(2000)['yaw'].velocity == 0.


@pytest.mark.parametrize('change,reason', [
    (lambda g: setattr(g.trajectory, 'joint_names', ['yaw']), 'every configured joint'),
    (lambda g: setattr(g.trajectory, 'joint_names', ['yaw', 'yaw']), 'every configured joint'),
    (lambda g: setattr(g.trajectory.header.stamp, 'sec', 1), 'scheduled starts'),
    (lambda g: setattr(g.trajectory.points[0].time_from_start, 'nanosec', 1_000_000), 'start at zero'),
    (lambda g: setattr(g.trajectory.points[-1].time_from_start, 'nanosec', 1), 'integer-millisecond'),
    (lambda g: setattr(g.trajectory.points[-1].time_from_start, 'sec', 3), 'duration bound'),
    (lambda g: setattr(g.trajectory.points[-1].time_from_start, 'sec', 0), 'strictly increase'),
    (lambda g: setattr(g.trajectory.points[-1], 'positions', [float('nan'), 0.]), 'finite'),
    (lambda g: setattr(g.trajectory.points[-1], 'velocities', [0., 0.]), 'empty velocity'),
    (lambda g: setattr(g.trajectory.points[-1], 'accelerations', [1., 0.]), 'empty velocity'),
    (lambda g: setattr(g.trajectory.points[-1], 'effort', [0., 1.]), 'empty velocity'),
    (lambda g: setattr(g.multi_dof_trajectory, 'joint_names', ['floating']), 'multi-DOF'),
    (lambda g: setattr(g, 'path_tolerance', [JointTolerance(name='yaw', position=-1.)]), 'disabling'),
    (lambda g: setattr(g, 'goal_tolerance', [JointTolerance(name='pitch', acceleration=.1)]), 'acceleration'),
    (lambda g: setattr(g, 'goal_tolerance', [JointTolerance(name='other', position=.1)]), 'known'),
    (lambda g: setattr(g, 'goal_tolerance', [JointTolerance(name='yaw'), JointTolerance(name='yaw')]), 'unique'),
    (lambda g: setattr(g.goal_time_tolerance, 'sec', 11), 'execution bound'),
])
def test_unsupported_goal_semantics_fail_closed(model, change, reason):
    value = goal()
    change(value)
    with pytest.raises(ValueError, match=reason):
        TrajectoryCompiler(model, lambda _message: None).compile(value, 1)


def test_final_validation_is_not_bypassed(model):
    def reject(_message):
        raise ValueError('configured mechanical limit')
    with pytest.raises(ValueError, match='mechanical limit'):
        TrajectoryCompiler(model, reject).compile(goal(), 1)


def test_expanded_motor_capacity_is_enforced(model):
    value = goal()
    value.trajectory.points = [JointTrajectoryPoint(
        positions=[0., 0.], time_from_start=Duration(nanosec=index * 1_000_000))
        for index in range(129)]
    with pytest.raises(ValueError, match='256'):
        TrajectoryCompiler(model, lambda _message: None).compile(value, 1)


def test_requested_tolerances_and_default_goal_time(model):
    value = goal()
    value.path_tolerance = [JointTolerance(name='yaw', position=.03, velocity=.04)]
    plan = TrajectoryCompiler(model, lambda _message: None).compile(value, 1)
    assert plan.tolerances.path_position['yaw'] == .03
    assert plan.tolerances.path_position['pitch'] == .2
    assert plan.tolerances.path_velocity['yaw'] == .04
    assert plan.tolerances.goal_time_s == 1.


def test_tick_alignment_wraps_without_accepting_backwards_time():
    assert signed_tick_difference(4, 0xFFFFFFFE) == 6
    assert signed_tick_difference(0xFFFFFFFE, 4) == -6
