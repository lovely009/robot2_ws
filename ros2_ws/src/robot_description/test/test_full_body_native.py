"""Main model must use native mesh names and the verified assembly, not old ICP."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'
SOURCE = MODEL.parent / 'robot_body/urdf/cnm.urdf'


def arm_delta(name):
    mounting = json.loads((MODEL / 'config/arm_mount_alignment.json').read_text())
    return np.asarray(mounting['world_transform']) if name in mounting['affected_links'] else np.eye(4)


def test_calibration_uses_current_source_files():
    import hashlib
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    assert alignment['urdf_sha256'] == hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert alignment['step_sha256'] == hashlib.sha256((MODEL.parent / '全身不带头.STEP').read_bytes()).hexdigest()
    assert json.loads((MODEL / 'config/display_meshes.json').read_text())['source'] == 'robot_body'
    for entry in alignment['links'].values():
        path = SOURCE.parents[1] / 'meshes' / entry['source']
        assert entry['source_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()


def expected_names():
    source = ET.parse(SOURCE).getroot()
    return [j.get('name').strip().removesuffix('_link') for j in source.findall('joint')]


def test_canonical_names_cover_urdf_simulation_and_motor_groups():
    expected = expected_names()
    root = ET.parse(MODEL / 'urdf/full_body_robot.urdf').getroot()
    assert [j.get('name') for j in root.findall('joint')] == expected
    assert {l.get('name') for l in root.findall('link')} == {'Base_link', *(n + '_link' for n in expected)}
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())
    names = [m['joint'] for m in topology['motors'].values() if 'joint' in m]
    names += [j for g in topology['differential_groups'].values() for j in g['joints']]
    assert len(names) == len(set(names)) == 37
    assert set(names) == set(expected)
    assert sorted(topology['motors']) == list(range(1, 38))
    simulation = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    config = json.loads(simulation.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    assert config['joints'] == expected
    assert config['joint_aliases']['BN_Hips_axis_1'] == 'BN_hips_roll'
    assert config['joint_aliases']['BN_LeftHand_axis_1'] == 'BN_LeftHand_roll'


def raw_mesh_pose(model, data, geom_name):
    from scipy.spatial.transform import Rotation
    geom = model.geom(geom_name)
    mesh = model.geom_dataid[geom.id]
    source = np.eye(4)
    source[:3,:3] = Rotation.from_quat(model.mesh_quat[mesh][[1,2,3,0]]).as_matrix()
    source[:3,3] = model.mesh_pos[mesh]
    current = np.eye(4)
    current[:3,:3] = data.geom_xmat[geom.id].reshape(3,3)
    current[:3,3] = data.geom_xpos[geom.id]
    return current @ np.linalg.inv(source)


@pytest.fixture(scope='module')
def simulation():
    mujoco = pytest.importorskip('mujoco')
    model = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/full_body_robot.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return mujoco, model, data


def test_main_zero_pose_matches_verified_native_mesh_frames(simulation):
    _, model, data = simulation
    from scipy.spatial.transform import Rotation
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    for link, frame in alignment['links'].items():
        actual = raw_mesh_pose(model, data, link + '_robot_body')
        expected = arm_delta(link) @ np.asarray(frame['world_transform'])
        # Compiled mesh normalization can discard sub-micrometre COM offsets.
        np.testing.assert_allclose(actual, expected, atol=1e-6, err_msg=link)
    assert (model.nq, model.nu, model.neq) == (67,37,10)
    assert np.max(np.abs(data.efc_pos)) < .000025


def test_actual_hips_visuals_keep_installed_fixture_pose(simulation):
    _, model, data = simulation
    calibration = json.loads((MODEL / 'config/hips_mount_alignment.json').read_text())
    for side, entry in calibration['sides'].items():
        for part in ('flange','rod','platform'):
            actual = raw_mesh_pose(model, data, f'hips_{side}_{part}_visual')
            expected = np.asarray(entry[part + '_transform']) @ calibration['base_frame_world']
            np.testing.assert_allclose(actual, expected, atol=2e-7, err_msg=f'{side}/{part}')


def test_both_hand_plates_extend_outward_from_forearms_at_neutral(simulation):
    """A closed wrist can still have its hand plate mounted backwards."""
    _, model, data = simulation
    vtk = pytest.importorskip('vtk')
    from vtk.util.numpy_support import vtk_to_numpy
    for side in ('Left', 'Right'):
        name = f'BN_{side}Hand_pitch_link'
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(MODEL / 'mujoco/visual/native' / f'{name}.STL'))
        reader.Update()
        vertices = vtk_to_numpy(reader.GetOutput().GetPoints().GetData())
        tip = vertices[vertices[:, 1] < vertices[:, 1].min() + .001].mean(0)
        pose = raw_mesh_pose(model, data, name + '_robot_body')
        tip = pose[:3, :3] @ tip + pose[:3, 3]
        hinge = data.body(name).xpos
        outward = hinge - data.body(f'BN_{side}LowerArm_2_link').xpos
        outward /= np.linalg.norm(outward)
        assert np.dot(tip - hinge, outward) > .04, f'{side}: hand plate folds back into forearm'


def test_remounted_wrist_crank_meshes_keep_fixed_motor_axis_lines(simulation):
    _, model, data = simulation
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())['links']
    rods = json.loads((MODEL / 'cad/assembly_audit/2026-11-13-step-linkages.json').read_text())['rods']
    for name, rod in rods.items():
        if not name.startswith('left_wrist_'):
            continue
        source = np.asarray(alignment[rod['source_body']]['world_transform'])
        cylinder = rod['motor_axis_candidates'][0]
        origin, axis = np.asarray(cylinder['point_on_axis_m']), np.asarray(cylinder['axis'])
        prefix = name.removesuffix('_rod')
        pose = raw_mesh_pose(model, data, prefix + '_flange_visual')
        actual_origin = (pose @ np.linalg.solve(source, [*origin, 1]))[:3]
        actual_axis = pose[:3, :3] @ source[:3, :3].T @ axis
        delta = arm_delta(rod['source_body'])
        origin = (delta @ [*origin, 1.])[:3]
        axis = delta[:3, :3] @ axis
        assert np.linalg.norm(np.cross(actual_origin - origin, axis)) < .000025
        assert abs(actual_axis @ axis) > np.cos(np.deg2rad(.02))


def test_native_rods_stay_closed_under_small_output_motion(simulation):
    mujoco, model, _ = simulation
    from robot_transmission.mujoco_positions import JointPositionTargets
    root = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    config = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    controller = JointPositionTargets(model, config)
    topology = yaml.safe_load((MODEL / 'config/motor_topology.yaml').read_text())
    data = mujoco.MjData(model)
    for group in topology['differential_groups'].values():
        for sign in (-1,1):
            targets = dict.fromkeys(controller.joints,0.)
            targets.update({j: sign*np.deg2rad(.5) for j in group['joints']})
            controller.set_configuration(data,list(targets.values()))
            mujoco.mj_forward(model,data)
            assert np.max(np.abs(data.efc_pos)) < .000025


def test_moving_rod_mesh_ends_follow_both_bearing_centers(simulation):
    """Check the actual visible meshes, so closed helper sites cannot hide a gap."""
    mujoco, model, _ = simulation
    from robot_transmission.mujoco_positions import JointPositionTargets
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    report = json.loads((MODEL / 'cad/assembly_audit/2026-11-13-step-linkages.json').read_text())
    root = ET.parse(MODEL / 'mujoco/full_body_robot.xml').getroot()
    config = json.loads(root.find("custom/text[@name='robot_sim_joint_position']").get('data'))
    controller = JointPositionTargets(model, config)
    data = mujoco.MjData(model)
    for sign in (0, -1, 1):
        targets = [sign * np.deg2rad(.5) for _ in controller.joints]
        controller.set_configuration(data, targets)
        mujoco.mj_forward(model, data)
        for name, rod in report['rods'].items():
            source = np.asarray(alignment['links'][rod['source_body']]['world_transform'])
            pose = raw_mesh_pose(model, data, name + '_visual')
            prefix = name.removesuffix('_rod')
            for side in ('flange', 'platform'):
                local = np.linalg.solve(source, [*rod['endpoints'][side]['center_m'], 1])
                actual = (pose @ local)[:3]
                site = prefix + ('_motor_ball' if side == 'flange' else '_platform_ball')
                np.testing.assert_allclose(actual, data.site(site).xpos, atol=.000025, err_msg=f'{name}/{side}')


def test_relocated_hinges_follow_actual_step_cylinder_lines(simulation):
    _,model,data=simulation
    calibration=json.loads((MODEL/'config/kinematic_axes.json').read_text())
    features=json.loads((MODEL/'cad/assembly_audit/step-analytic-features.json').read_text())
    assert calibration['step_sha256']==features['step_sha256']
    for name,record in calibration['joints'].items():
        cylinder=features['cylinders'][record['step_cylinder_index']]
        axis=np.asarray(cylinder['axis']);origin=np.asarray(cylinder['origin_m'])
        joint=model.joint(name)
        delta=arm_delta(model.body(int(joint.bodyid[0])).name)
        origin=(delta@[*origin,1.])[:3];axis=delta[:3,:3]@axis
        assert np.linalg.norm(np.cross(data.xanchor[joint.id]-origin,axis))<.000025,name
        assert abs(data.xaxis[joint.id]@axis)>np.cos(np.deg2rad(.02)),name


def test_convex_wrist_ball_stays_on_platform():
    geometry=json.loads((MODEL/'config/native_assembly.json').read_text())
    source=geometry['ownership']['BN_RightLowerArm_2_link']['parts']
    assert source['right_wrist_flange_proximal_platform']==[27]
    assert 27 not in source['right_wrist_flange_proximal_rod']


def test_new_export_needs_no_second_left_hand_remount():
    report = json.loads((MODEL / 'config/left_hand_mount_alignment.json').read_text())
    assert report['rotation_rad'] == 0.
    np.testing.assert_allclose(report['hand_transform'], np.eye(4), atol=1e-12)
    for rod in report['rods'].values():
        assert abs(rod['motor_zero_shift_rad']) < 1e-10


def test_new_base_yaw_frame_keeps_vertical_world_axis(simulation):
    from scipy.spatial.transform import Rotation
    _, model, data = simulation
    raw = ET.parse(SOURCE).getroot().find("joint[@name='BN_Spine_yaw_link ']")
    origin = raw.find('origin')
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
    expected[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
    np.testing.assert_allclose(raw_mesh_pose(model, data, 'BN_Spine_yaw_link_robot_body'), expected, atol=1e-6)
    np.testing.assert_allclose(data.xaxis[model.joint('BN_Spine_yaw').id], [0., 0., -1.], atol=1e-8)
    np.testing.assert_allclose(data.body('Base_link').xmat.reshape(3, 3), np.eye(3), atol=1e-10)


def test_hand_plate_tips_match_step_including_mounting_side(simulation):
    from scipy.spatial import cKDTree
    from vtk.util.numpy_support import vtk_to_numpy
    import vtk
    _, model, data = simulation
    evidence = json.loads((MODEL / 'cad/assembly_audit/step-hand-tips.json').read_text())
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    assert evidence['step_sha256'] == alignment['step_sha256']
    for name, targets in evidence['landmarks'].items():
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(MODEL / 'mujoco/visual/native' / f'{name}.STL'))
        reader.Update()
        vertices = vtk_to_numpy(reader.GetOutput().GetPoints().GetData())
        pose = raw_mesh_pose(model, data, name + '_robot_body')
        world = vertices @ pose[:3, :3].T + pose[:3, 3]
        delta = arm_delta(name)
        targets = np.asarray(targets) @ delta[:3, :3].T + delta[:3, 3]
        assert max(cKDTree(world).query(targets)[0]) < .000025, name


def test_screen_left_arm_rotates_as_one_assembly_at_upper_arm_roll(simulation):
    from scipy.spatial.transform import Rotation
    _, model, data = simulation
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())['links']
    source = ET.parse(SOURCE).getroot().find("joint[@name='BN_LeftUpperArm_3_link']")
    old = np.asarray(alignment['BN_LeftUpperArm_3_link']['world_transform'])
    axis = old[:3, :3] @ np.fromstring(source.find('axis').get('xyz'), sep=' ')
    rotation = Rotation.from_rotvec(np.pi * axis / np.linalg.norm(axis)).as_matrix()
    delta = np.eye(4)
    delta[:3, :3] = rotation
    delta[:3, 3] = old[:3, 3] - rotation @ old[:3, 3]
    for name in [f'BN_Left{part}_link' for part in ('UpperArm_3', 'LowerArm_1', 'Hand_roll', 'Hand_pitch')]:
        expected = delta @ np.asarray(alignment[name]['world_transform'])
        np.testing.assert_allclose(raw_mesh_pose(model, data, name + '_robot_body'), expected, atol=1e-6)
    name = 'BN_LeftUpperArm_2_link'
    np.testing.assert_allclose(raw_mesh_pose(model, data, name + '_robot_body'), alignment[name]['world_transform'], atol=1e-6)
    assert max(abs(data.qpos)) < 1e-12
