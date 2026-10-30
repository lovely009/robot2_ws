"""Managed intent validation against real ROS transmission implementations."""
from __future__ import annotations

from types import SimpleNamespace as NS

import pytest
import yaml

from robot_supervisor.validation import CommandValidator


def target(motor_id=1, position=0., velocity=0., effort=0., **extra):
    return NS(motor_id=motor_id, position=position, velocity=velocity, effort=effort, **extra)


def command(*targets):
    return NS(count=len(targets), targets=list(targets))


def policy_target(motor_id=1, enabled=True, kp=1., kd=.1, confirm_unprotected=False):
    return NS(motor_id=motor_id, enabled=enabled, kp=kp, kd=kd,
              confirm_unprotected=confirm_unprotected)


def trajectory(ids=(1,), samples=((0, (0.,)), (1000, (.5,)))):
    points = [target(i, position=p, time_from_start_ms=t)
              for t, positions in samples for i, p in zip(ids, positions)]
    return NS(count=len(points), points=points, target_count=len(ids), target_ids=list(ids),
              segment_index=0, segment_count=1)


@pytest.fixture
def files(tmp_path):
    specs = {'motor_types': {'sample': {'protocol': 'cerb', 'parameters': {
        'mit_position_limit_rad': 3., 'mit_velocity_limit_rad_s': 3.,
        'mit_torque_limit_nm': 10., 'peak_torque_nm': 4.,
        'mit_kp_limit': 5., 'mit_kd_limit': 1.,
    }}}}
    (tmp_path/'specs.yaml').write_text(yaml.safe_dump(specs))
    transmissions = {'transmissions': {'differential': {
        'plugin': 'transmission_interface/DifferentialTransmission',
        'actuators': ['a2', 'a3'], 'joints': ['j2', 'j3'],
        'actuator_reduction': [1., 1.], 'joint_reduction': [1., 1.],
        'joint_offset': [0., 0.],
    }}}
    (tmp_path/'transmissions.yaml').write_text(yaml.safe_dump(transmissions))
    driver = {'motor_hardware': {
        'motor_specs_file': 'specs.yaml', 'transmissions_file': 'transmissions.yaml',
        'motors': {f'a{i}': {'id': i, 'joint': f'j{i}', 'type': 'sample'}
                   for i in (1, 2, 3)},
    }}
    driver_file = tmp_path/'driver.yaml'
    driver_file.write_text(yaml.safe_dump(driver))
    urdf_file = tmp_path/'robot.urdf'
    urdf_file.write_text('<robot name="test"><link name="base"/>' + ''.join(
        f'<link name="l{i}"/><joint name="j{i}" type="revolute">'
        f'<parent link="base"/><child link="l{i}"/><axis xyz="0 0 1"/>'
        '<limit lower="-1" upper="1" velocity="1" effort="10"/></joint>'
        for i in (1, 2, 3)) + '</robot>')
    return driver_file, urdf_file


@pytest.fixture
def validator(files):
    return CommandValidator(*(str(p) for p in files))


def test_real_model_config_and_safe_intents(validator):
    assert validator.motor_ids == [1, 2, 3]
    assert set(validator.model.joint_names) == {'j1', 'j2', 'j3'}
    validator.command(command(target(position=.5)))
    validator.command(command(target(2, .5), target(3, -.5)))
    validator.command(command())
    validator.policy(command(policy_target(2), policy_target(3)))
    validator.policy(command())
    validator.trajectory(trajectory())
    validator.trajectory(trajectory(ids=(), samples=()))


@pytest.mark.parametrize('message', [
    NS(count=2, targets=[target()]),
    command(target(999)), command(target(), target()), command(target(2)),
    command(target(position=float('nan'))), command(target(velocity=float('inf'))),
    command(target(effort=4.1)), command(target(position=1.1)),
    command(target(velocity=1.1)), command(target(position=3.1)),
    NS(count=True, targets=[target()]), NS(count=1, targets=None),
])
def test_invalid_snapshots_are_rejected(validator, message):
    with pytest.raises(ValueError):
        validator.command(message)


def test_transmission_output_limits_are_checked(files):
    driver, urdf = files
    urdf.write_text(urdf.read_text().replace('effort="10"', 'effort="1"'))
    validator = CommandValidator(str(driver), str(urdf))
    # Each actuator is below its physical limit, but joint torque is their sum.
    with pytest.raises(ValueError, match='effort'):
        validator.command(command(target(2, effort=.6), target(3, effort=.6)))


@pytest.mark.parametrize('targets', [
    [policy_target(kp=-1)], [policy_target(kp=5.1)], [policy_target(kd=float('nan'))],
    [policy_target(enabled=1)], [policy_target(confirm_unprotected=1)],
    [policy_target(2)], [policy_target(2), policy_target(3, enabled=False)],
    [policy_target(999)], [policy_target(), policy_target()],
])
def test_policy_flags_gains_identity_and_group_consistency(validator, targets):
    with pytest.raises(ValueError):
        validator.policy(command(*targets))


@pytest.mark.parametrize('mutate', [
    lambda m: setattr(m, 'count', 100),
    lambda m: setattr(m, 'segment_count', 2),
    lambda m: setattr(m, 'segment_index', 1),
    lambda m: setattr(m, 'segment_index', False),
    lambda m: setattr(m.points[0], 'time_from_start_ms', 10),
    lambda m: setattr(m.points[1], 'time_from_start_ms', 0),
    lambda m: setattr(m.points[1], 'time_from_start_ms', 2001),
    lambda m: setattr(m.points[1], 'motor_id', 999),
    lambda m: setattr(m.points[1], 'velocity', .1),
    lambda m: setattr(m.points[1], 'position', float('nan')),
    lambda m: setattr(m, 'target_ids', [1, 1]),
    lambda m: setattr(m, 'target_count', 0),
])
def test_trajectory_structure_rejected(validator, mutate):
    message = trajectory()
    mutate(message)
    with pytest.raises(ValueError):
        validator.trajectory(message)


def test_every_trajectory_sample_must_cover_full_target_group(validator):
    message = trajectory((2, 3), ((0, (0., 0.)), (1000, (.5, -.5))))
    validator.trajectory(message)
    message.points.pop()
    message.count -= 1
    with pytest.raises(ValueError, match='every trajectory time'):
        validator.trajectory(message)


def test_interpolation_peak_velocity_is_not_only_average_velocity(validator):
    # Average is 1 rad/s, but endpoint-zero Hermite peaks at 1.5 rad/s.
    message = trajectory(samples=((0, (0.,)), (500, (.5,))))
    with pytest.raises(ValueError, match='interpolated velocity'):
        validator.trajectory(message)


def test_differential_combination_cannot_overshoot_between_safe_waypoints(files):
    driver, urdf = files
    content = urdf.read_text().replace('velocity="1"', 'velocity="10"')
    # The second differential joint is allowed a larger angle, isolating the
    # first joint's interior overshoot despite all waypoint values being <= 1.
    before, j3 = content.split('<joint name="j3"', 1)
    content = before + '<joint name="j3"' + j3.replace('upper="1"', 'upper="3"')
    urdf.write_text(content)
    validator = CommandValidator(str(driver), str(urdf), max_trajectory_ms=4000)
    message = trajectory((2, 3), ((0, (0., 0.)), (1000, (1., 1.)),
                                     (2000, (2., 0.)), (3000, (3., -1.))))
    with pytest.raises(ValueError, match='j2.interpolated position'):
        validator.trajectory(message)


@pytest.mark.parametrize('change', ['duplicate_id', 'missing_limits', 'nonfinite_limit',
                                   'unknown_joint', 'missing_specs', 'bad_gain'])
def test_invalid_configuration_cannot_construct_validator(files, change):
    driver, urdf = files
    document = yaml.safe_load(driver.read_text())
    if change == 'duplicate_id':
        document['motor_hardware']['motors']['a3']['id'] = 2
    elif change == 'unknown_joint':
        document['motor_hardware']['motors']['a1']['joint'] = 'missing'
    elif change == 'missing_specs':
        document['motor_hardware']['motor_specs_file'] = 'missing.yaml'
    elif change == 'bad_gain':
        document['motor_hardware']['motors']['a1']['parameters'] = {'mit_kp_limit': float('inf')}
    elif change == 'missing_limits':
        urdf.write_text(urdf.read_text().replace(' velocity="1"', ''))
    else:
        urdf.write_text(urdf.read_text().replace('velocity="1"', 'velocity="nan"'))
    driver.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        CommandValidator(str(driver), str(urdf))
