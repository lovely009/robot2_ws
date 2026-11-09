"""仿真派生资源、限位与来源隔离回归；使用 MuJoCo venv 运行。"""
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
MODEL = Path(__file__).resolve().parents[1] / 'models/trial_arm_7dof'


def test_simulation_assets_and_source_isolation():
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    for name, digest in report['sha256'].items():
        assert hashlib.sha256((MODEL / name).read_bytes()).hexdigest() == digest
    original = ET.parse(MODEL / 'urdf/trial_arm_7dof.urdf').getroot()
    standalone = ET.parse(MODEL / 'urdf/trial_arm_7dof.standalone.urdf').getroot()
    for joint in original.findall("joint[@type='revolute']"):
        assert joint.find('limit').get('lower') == joint.find('limit').get('upper') == '0'
        derived = standalone.find(f"joint[@name='{joint.get('name')}']/limit")
        limits = report['joints'][joint.get('name')]['range_rad']
        np.testing.assert_allclose([float(derived.get('lower')), float(derived.get('upper'))], limits)
    for mesh in ET.parse(MODEL / 'mujoco/trial_arm_7dof.xml').getroot().find('asset'):
        assert not Path(mesh.get('file')).is_absolute()
        assert (MODEL / 'meshes' / mesh.get('file')).is_file()


def test_mujoco_names_zero_pose_and_inertia():
    path = MODEL / 'tools/build_mujoco_model.py'
    spec = importlib.util.spec_from_file_location('trial_mujoco_builder', path)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    expected, _ = builder.import_model()
    actual = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/trial_arm_7dof.xml'))
    assert actual.nq == actual.nv == actual.nu == 7
    assert [actual.joint(i).name for i in range(7)] == [f'Joint_Link{i}' for i in range(1, 8)]
    np.testing.assert_allclose(actual.body_mass, expected.body_mass, atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual.body_ipos, expected.body_ipos, atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual.body_iquat, expected.body_iquat, atol=1e-12, rtol=0)
    np.testing.assert_allclose(actual.body_inertia, expected.body_inertia, atol=1e-14, rtol=0)
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    np.testing.assert_allclose(actual.jnt_range, [report['joints'][f'Joint_Link{i}']['range_rad']
                                               for i in range(1, 8)], atol=1e-11, rtol=0)
    for q in [np.zeros(7), np.array([.01, -.01, .02, -.02, .01, -.01, .02])]:
        a, b = mujoco.MjData(actual), mujoco.MjData(expected)
        a.qpos[:], b.qpos[:] = q, q
        mujoco.mj_forward(actual, a)
        mujoco.mj_forward(expected, b)
        np.testing.assert_allclose(a.xpos, b.xpos, atol=1e-8)
        np.testing.assert_allclose(a.xmat, b.xmat, atol=1e-8)


def test_finite_gravity_step_and_joint_stops():
    model = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/trial_arm_7dof.xml'))
    data = mujoco.MjData(model)
    for _ in range(2000):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert np.all(data.qpos >= model.jnt_range[:, 0] - .05)
    assert np.all(data.qpos <= model.jnt_range[:, 1] + .05)
    assert not any(w.number for w in data.warning)
    # 隔离自接触和重力，验证每个关节本身的双侧止挡，而非被其他碰撞卡住。
    model.opt.gravity[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    for index in range(7):
        assert model.jnt_limited[index]
        assert model.jnt_range[index, 0] <= 0 <= model.jnt_range[index, 1]
        assert model.jnt_range[index, 0] < model.jnt_range[index, 1]
        for side in (0, 1):
            mujoco.mj_resetData(model, data)
            data.qpos[index] = model.jnt_range[index, side]
            data.ctrl[index] = -.1 if side == 0 else .1
            for _ in range(250):
                mujoco.mj_step(model, data)
            assert abs(data.qpos[index] - model.jnt_range[index, side]) < .02


def test_shoulder_rejects_opposite_input_branch():
    from robot_transmission import Axis, Model
    transmission = Model.load(str(MODEL / 'config/motor_driver.yaml'))
    for q1, q2 in [(0, -22), (-17, -18)]:
        state = {f'Joint_Link{i}': Axis() for i in range(1, 8)}
        state['Joint_Link1'] = Axis(float(np.deg2rad(q1)), 0, 0)
        state['Joint_Link2'] = Axis(float(np.deg2rad(q2)), 0, 0)
        with pytest.raises(ValueError, match='half-circle branch'):
            transmission.joint_to_actuator(state)
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    first = report['joints']['Joint_Link1']['range_rad']
    second = report['joints']['Joint_Link2']['range_rad']
    import yaml
    topology = yaml.safe_load((MODEL / 'config/simulation_topology.yaml').read_text())['simulation']
    profiles = {m['motor']: topology['profiles'][m['profile']] for m in topology['motors']}
    for q1 in np.linspace(*first, 31):
        for q2 in np.linspace(*second, 31):
            state = {f'Joint_Link{i}': Axis() for i in range(1, 8)}
            state['Joint_Link1'] = Axis(float(q1), 0, 0)
            state['Joint_Link2'] = Axis(float(q2), 0, 0)
            axes = transmission.joint_to_actuator(state)
            for name in ('arm_joint1', 'arm_joint2'):
                profile = profiles[name]
                angle = float(np.rad2deg(axes[name].position))
                assert profile['hard_lower_deg'] - 1e-8 <= angle <= profile['hard_upper_deg'] + 1e-8
                assert profile['command_lower_deg'] - 1e-8 <= angle <= profile['command_upper_deg'] + 1e-8


def test_partial_collision_cache_is_not_complete(tmp_path):
    spec = importlib.util.spec_from_file_location('collision_builder', MODEL / 'tools/build_collision_meshes.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source = MODEL / 'meshes/base_link.STL'
    (tmp_path / 'base_link_00.obj').write_text('partial')
    assert not module.completed(source, tmp_path)
    manifest = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    parameters=module.PARAMETERS, parts={'base_link_00.obj': hashlib.sha256(b'partial').hexdigest()})
    (tmp_path / 'base_link.complete.json').write_text(json.dumps(manifest))
    assert module.completed(source, tmp_path)
    (tmp_path / 'base_link_00.obj').write_text('interrupted replacement')
    assert not module.completed(source, tmp_path)


def test_hil_control_urdf_exposes_simulation_limits():
    import yaml
    path = MODEL.parents[2] / 'motor_hardware_interface/launch/host_trajectory.launch.py'
    spec = importlib.util.spec_from_file_location('trial_host_trajectory', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    urdf = MODEL / 'urdf/trial_arm_7dof.standalone.urdf'
    config = MODEL / 'config/motor_driver.simulation.yaml'
    document, joints = module.hardware_description(str(config), str(urdf))
    assert len(joints) == 7 and set(joints) == {f'Joint_Link{i}' for i in range(1, 8)}
    root = ET.fromstring(document)
    report = json.loads((MODEL / 'mujoco/geometry_limits.json').read_text())
    topology = yaml.safe_load((MODEL / 'config/simulation_topology.yaml').read_text())['simulation']
    profiles = {m['joint']: topology['profiles'][m['profile']] for m in topology['motors']}
    for name in joints:
        limits = root.find(f"joint[@name='{name}']/limit")
        np.testing.assert_allclose([float(limits.get('lower')), float(limits.get('upper'))],
                                   report['joints'][name]['range_rad'], atol=1e-11, rtol=0)
        assert float(limits.get('velocity')) == profiles[name]['velocity_rad_s']
        assert float(limits.get('effort')) == profiles[name]['effort_nm']
        interface = root.find(f"ros2_control/joint[@name='{name}']/command_interface[@name='position']")
        np.testing.assert_allclose([float(interface.find(f"param[@name='{key}']").text)
                                   for key in ('min', 'max')], report['joints'][name]['range_rad'], atol=1e-11, rtol=0)
