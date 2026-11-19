"""Small articulated fixture: verify IK math and model/export compatibility."""

import copy
import json
from pathlib import Path
import runpy
import sys
import os
import signal
import subprocess
import time

import numpy as np
import pytest
from scipy.optimize._numdiff import approx_derivative

pytest.importorskip('pinocchio')
from bvh_broadcaster.bvh import BvhClip
from bvh_broadcaster.retarget import ArmRetargeter, digest, load_motion


@pytest.fixture
def arm(tmp_path):
    urdf = tmp_path / 'arm.urdf'
    links = ''.join(f'<link name="{name}"/>' for name in ('base', 'shoulder', 'elbow', 'hand'))
    joints = ''
    for i, (parent, child, xyz, axis) in enumerate([
            ('base', 'shoulder', '0 0 0', '0 0 1'),
            ('shoulder', 'elbow', '1 0 0', '0 0 1'),
            ('elbow', 'hand', '1 0 0', '1 0 0')]):
        joints += f'''<joint name="q{i}" type="revolute"><parent link="{parent}"/>
        <child link="{child}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/>
        <limit lower="-1.5" upper="1.5" velocity="0.8" effort="1"/></joint>'''
    urdf.write_text(f'<robot name="arm">{links}{joints}</robot>')
    bvh = tmp_path / 'arm.bvh'
    bvh.write_text('''HIERARCHY
ROOT Torso {
 OFFSET 0 0 0
 CHANNELS 6 Xposition Yposition Zposition Yrotation Xrotation Zrotation
 JOINT Shoulder {
  OFFSET 0 0 0
  CHANNELS 3 Yrotation Xrotation Zrotation
  JOINT Elbow {
   OFFSET 0 0 100
   CHANNELS 3 Yrotation Xrotation Zrotation
   JOINT Hand {
    OFFSET 0 0 100
    CHANNELS 3 Yrotation Xrotation Zrotation
    End Site { OFFSET 0 0 10 }
   }
  }
 }
}
MOTION
Frames: 6
Frame Time: 0.1
0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
0 0 0 0 0 0 60 0 0 30 0 0 0 0 20
0 0 0 0 0 0 60 0 0 30 0 0 0 0 20
0 0 0 0 0 0 60 0 0 30 0 0 0 0 20
0 0 0 0 0 0 60 0 0 30 0 0 0 0 20
0 0 0 0 0 0 60 0 0 30 0 0 0 0 20
''')
    clip = BvhClip.load(bvh)
    mapping = {'torso': 'Torso', 'source_to_robot': np.eye(3).tolist(),
               'arms': {'right': {'source': ['Shoulder', 'Elbow', 'Hand'],
                                  'robot': ['shoulder', 'elbow', 'hand']}}}
    return urdf, mapping, clip


def test_direction_and_orientation_jacobian(arm):
    solver = ArmRetargeter(*arm)
    targets, rotations = solver.targets(1)
    q = np.array([.3, -.2, .4])
    actual = solver.residual(q, targets, rotations)[1]
    numerical = approx_derivative(lambda value: solver.residual(value, targets, rotations)[0], q)
    np.testing.assert_allclose(actual, numerical, atol=1e-7)


def test_ik_enforces_step_velocity_and_joint_bounds(arm):
    solver = ArmRetargeter(*arm)
    positions, direction_errors, orientation_errors = solver.convert()
    assert positions.shape == (6, 3)
    assert np.isfinite([direction_errors.ravel().tolist() + orientation_errors.ravel().tolist()]).all()
    assert np.all(positions >= solver.lower) and np.all(positions <= solver.upper)
    assert np.all(np.abs(np.diff(positions, axis=0)) <= solver.velocity * .1 + 1e-8)
    assert positions[-1, 0] > positions[0, 0] + .2


def test_source_torso_transform_does_not_move_fixed_base_targets(arm):
    solver = ArmRetargeter(*arm)
    before = solver.targets(1)
    arm[2].frames[1, :6] = [10, 20, 30, 65, 25, 10]
    after = solver.targets(1)
    for a, b in zip(before, after):
        np.testing.assert_allclose(a, b, atol=1e-12)


def test_one_sided_reference_bounds_do_not_stall(arm):
    urdf, mapping, clip = arm
    urdf.write_text(urdf.read_text().replace('lower="-1.5"', 'lower="0"'))
    clip.frames[0, 6] = 45
    solver = ArmRetargeter(urdf, mapping, clip)
    assert solver.reference[0] == pytest.approx(np.pi / 4, abs=.01)


def test_model_geometry_reload_and_missing_link(arm):
    urdf, mapping, clip = arm
    old = ArmRetargeter(*arm)
    old_points = old.poses(np.zeros(3))[0]
    urdf.write_text(urdf.read_text().replace('xyz="1 0 0"', 'xyz="2 0 0"'))
    new = ArmRetargeter(*arm)
    np.testing.assert_allclose(new.poses(np.zeros(3))[0], old_points * 2)
    mapping = copy.deepcopy(mapping)
    mapping['arms']['right']['robot'][-1] = 'renamed_hand'
    with pytest.raises(ValueError, match='URDF link missing: renamed_hand'):
        ArmRetargeter(urdf, mapping, clip)


def test_added_unmapped_joint_requires_mapping_update(arm):
    urdf, _, _ = arm
    extra = '''<link name="extra"/><joint name="new_joint" type="revolute">
    <parent link="base"/><child link="extra"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/></joint>'''
    urdf.write_text(urdf.read_text().replace('</robot>', extra + '</robot>'))
    with pytest.raises(ValueError, match='unmapped joints'):
        ArmRetargeter(*arm)


def test_export_rejects_stale_model_and_invalid_motion(arm, tmp_path):
    urdf, _, clip = arm
    solver = ArmRetargeter(*arm)
    positions, _, _ = solver.convert()
    motion = tmp_path / 'motion.npz'

    def save(p):
        np.savez(motion, positions=p, frame_time=clip.frame_time,
                 joint_names=solver.names, urdf_sha256=digest(urdf))

    save(positions)
    assert load_motion(motion, urdf)[0] == solver.names
    invalid = positions.copy()
    invalid[1, 0] = 1.4
    save(invalid)
    with pytest.raises(ValueError, match='velocity limits'):
        load_motion(motion, urdf)
    save(positions)
    urdf.write_text(urdf.read_text() + '\n')
    with pytest.raises(ValueError, match='URDF changed'):
        load_motion(motion, urdf)


def test_preview_launch_isolates_joint_topic_and_tf(arm, tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    try:
        get_package_prefix('bvh_broadcaster')
    except PackageNotFoundError:
        pytest.skip('Build and source bvh_broadcaster before the ROS integration test')
    urdf, _, clip = arm
    solver = ArmRetargeter(*arm)
    positions, _, _ = solver.convert()
    motion = tmp_path / 'motion.npz'
    np.savez(motion, positions=positions, frame_time=clip.frame_time,
             joint_names=solver.names, urdf_sha256=digest(urdf))
    rclpy.init(domain_id=98)
    node = rclpy.create_node('retarget_test_observer')
    joints, transforms = [], []
    subscriptions = [
        node.create_subscription(JointState, '/mocap_robot/joint_states', joints.append, 10),
        node.create_subscription(TFMessage, '/tf', transforms.append, 10)]
    with (tmp_path / 'preview.log').open('w+') as log:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'bvh_broadcaster', 'retarget.launch.py',
             f'motion_file:={motion}', f'urdf:={urdf}'],
            env={**os.environ, 'ROS_DOMAIN_ID': '98'}, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while ((not joints or not np.allclose(joints[-1].position, positions[-1])
                    or len(transforms) < 5) and proc.poll() is None and time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=.1)
            assert len(joints) >= 5 and len(transforms) >= 5, (tmp_path / 'preview.log').read_text()
            assert joints[-1].name == solver.names
            np.testing.assert_allclose(joints[-1].position, positions[-1])
            assert all(t.child_frame_id.startswith('retarget_robot/')
                       for batch in transforms for t in batch.transforms)
            topics = dict(node.get_topic_names_and_types())
            assert '/joint_states' not in topics
            assert not any('motor' in topic or 'command' in topic for topic in topics)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


@pytest.fixture
def retarget_packages(arm, tmp_path, monkeypatch):
    import yaml

    # 使用隔离的 ament 索引，不让工作区中的旧安装产物掩盖包归属错误。
    prefix = tmp_path / 'install'
    index = prefix / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    for package in ('bvh_broadcaster', 'robot_description'):
        (index / package).touch()
    mapping = prefix / 'share/bvh_broadcaster/config/half_body_robot.yaml'
    mapping.parent.mkdir(parents=True)
    mapping.write_text(yaml.safe_dump(arm[1]))
    urdf = prefix / 'share/robot_description/models/half_body_robot/urdf/half_body_robot.urdf'
    urdf.parent.mkdir(parents=True)
    urdf.write_text(arm[0].read_text())
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    return prefix, urdf


@pytest.mark.parametrize('external_urdf', [False, True], ids=['default-model', 'external-model'])
def test_retarget_cli_resolves_model_without_robot_sim(
        arm, retarget_packages, tmp_path, monkeypatch, external_urdf):
    from bvh_broadcaster.retarget import main

    prefix, default_urdf = retarget_packages
    urdf = arm[0] if external_urdf else default_urdf
    output = tmp_path / 'converted.npz'
    argv = ['retarget_bvh', str(tmp_path / 'arm.bvh'), '--output', str(output)]
    if external_urdf:
        # 外部模型不应要求安装默认模型包。
        (prefix / 'share/ament_index/resource_index/packages/robot_description').unlink()
        argv.extend(['--urdf', str(urdf)])
    monkeypatch.setattr(sys, 'argv', argv)

    main()

    report = json.loads(output.with_suffix('.json').read_text())
    assert report['urdf'] == str(urdf.resolve())
    names, positions, frame_time = load_motion(output, urdf)
    assert names == ['q0', 'q1', 'q2']
    assert positions.shape == (6, 3)
    assert frame_time == pytest.approx(.1)


@pytest.mark.parametrize('external_urdf', [False, True], ids=['default-model', 'external-model'])
def test_retarget_launch_resolves_model_without_robot_sim(
        arm, retarget_packages, external_urdf):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    prefix, default_urdf = retarget_packages
    context = LaunchContext()
    if external_urdf:
        (prefix / 'share/ament_index/resource_index/packages/robot_description').unlink()
        context.launch_configurations['urdf'] = str(arm[0])
    entry = Path(__file__).resolve().parents[1] / 'launch/retarget.launch.py'
    description = runpy.run_path(str(entry))['generate_launch_description']()
    argument = next(action for action in description.entities
                    if isinstance(action, DeclareLaunchArgument) and action.name == 'urdf')

    argument.execute(context)

    selected = Path(context.launch_configurations['urdf'])
    assert selected == (arm[0] if external_urdf else default_urdf)
    assert selected.is_file()
