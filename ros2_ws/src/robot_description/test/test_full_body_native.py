"""Main model must use native mesh names and the verified assembly, not old ICP."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'
SOURCE = MODEL.parent / 'Robot_body1/urdf/Robot_body1.urdf'


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
    alignment = json.loads((MODEL / 'config/step_alignment.json').read_text())
    for link, frame in alignment['links'].items():
        actual = raw_mesh_pose(model, data, link + '_robot_body')
        # Compiled mesh normalization can discard sub-micrometre COM offsets.
        np.testing.assert_allclose(actual, frame['world_transform'], atol=1e-6, err_msg=link)
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


def test_relocated_hinges_follow_actual_step_cylinder_lines(simulation):
    _,model,data=simulation
    calibration=json.loads((MODEL/'config/kinematic_axes.json').read_text())
    features=json.loads((MODEL/'cad/assembly_audit/step-analytic-features.json').read_text())
    assert calibration['step_sha256']==features['step_sha256']
    for name,record in calibration['joints'].items():
        cylinder=features['cylinders'][record['step_cylinder_index']]
        axis=np.asarray(cylinder['axis']);origin=np.asarray(cylinder['origin_m'])
        joint=model.joint(name)
        assert np.linalg.norm(np.cross(data.xanchor[joint.id]-origin,axis))<.000025,name
        assert abs(data.xaxis[joint.id]@axis)>np.cos(np.deg2rad(.02)),name


def test_convex_wrist_ball_stays_on_platform():
    geometry=json.loads((MODEL/'config/native_assembly.json').read_text())
    source=geometry['ownership']['BN_RightLowerArm_2_link']['parts']
    assert source['right_wrist_flange_proximal_platform']==[16]
    assert 16 not in source['right_wrist_flange_proximal_rod']
