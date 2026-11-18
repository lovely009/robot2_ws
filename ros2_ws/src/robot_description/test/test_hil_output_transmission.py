"""HIL 的输出关节必须通过几何传动到真实电机输入轴。"""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml

MODEL = Path(__file__).parents[1] / 'models/full_body_robot'

def build(tmp_path):
    spec=importlib.util.spec_from_file_location('hil_builder',MODEL/'tools/build_hil_config.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.build(MODEL,tmp_path)
    driver=tmp_path/'config/motor_driver.hil.yaml'
    assert driver.is_file(), 'HIL needs a separate host mapping to output joints'
    return driver


def test_host_mapping_exposes_outputs_while_backend_preserves_inputs(tmp_path):
    from robot_transmission import Model
    host=Model.load(str(build(tmp_path)))
    plant=Model.load(str(tmp_path/'config/motor_driver.simulation.yaml'))
    expected=json.loads(ET.parse(MODEL/'mujoco/full_body_robot.xml').find('custom/text[@name="robot_sim_joint_position"]').get('data'))['joints']
    assert set(host.joint_names)==set(expected)
    assert len(host.joint_names)==37
    assert 'left_wrist_flange_proximal' in plant.joint_names
    group=next(g for g in host.groups if 'left_wrist_pitch' in g.joints)
    assert [host.actuator_ids[a] for a in group.actuators]==[13,14]
    assert list(group.joints)==['left_wrist_pitch','left_wrist_roll']


def test_linkages_match_mujoco_inverse_closure_and_power(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    from robot_transmission import Axis, Model
    from robot_transmission.mujoco_positions import JointPositionTargets
    host=Model.load(str(build(tmp_path)))
    plant=yaml.safe_load((tmp_path/'config/motor_driver.simulation.yaml').read_text())['motor_hardware']['motors']
    mj=mujoco.MjModel.from_xml_path(str(MODEL/'mujoco/full_body_robot.xml'))
    config=json.loads(ET.parse(MODEL/'mujoco/full_body_robot.xml').find('custom/text[@name="robot_sim_joint_position"]').get('data'))
    reference=JointPositionTargets(mj,config); data=mujoco.MjData(mj)
    for group in host.groups:
        if len(group.actuators)!=2:continue
        for q in [(0.,0.),(.025,.02),(-.025,-.02),(.05,-.025)]:
            source={name:Axis(pos,.03*(i+1),.2*(i+1)) for i,(name,pos) in enumerate(zip(group.joints,q))}
            actuators=host.joint_to_actuator(source)
            reference.apply(data,group.joints,q)
            for name,axis in actuators.items():
                index=next(i for i in range(mj.nu) if mj.joint(mj.actuator_trnid[i,0]).name==plant[name]['joint'])
                assert axis.position==pytest.approx(data.ctrl[index],abs=2e-7)
            recovered=host.actuator_to_joint(actuators)
            for name in source:
                for field in ['position','velocity','effort']:
                    assert getattr(recovered[name],field)==pytest.approx(getattr(source[name],field),abs=2e-7)
            assert sum(a.velocity*a.effort for a in source.values())==pytest.approx(sum(a.velocity*a.effort for a in actuators.values()),abs=1e-9)
            eps=1e-6
            for j,name in enumerate(group.joints):
                shifted={n:Axis(q[k]+(eps if n==name else 0)) for k,n in enumerate(group.joints)}
                moved=host.joint_to_actuator(shifted)
                rates=host.joint_to_actuator({n:Axis(q[k],1. if n==name else 0.) for k,n in enumerate(group.joints)})
                for a in actuators:assert (moved[a].position-actuators[a].position)/eps==pytest.approx(rates[a].velocity,abs=2e-5)
            reference.apply(data,group.joints,[0.,0.])


def test_linkage_rejects_unreachable_target_without_poisoning_next_call(tmp_path):
    from robot_transmission import Axis, Model
    host=Model.load(str(build(tmp_path)))
    group=next(g for g in host.groups if 'left_wrist_pitch' in g.joints)
    with pytest.raises((ValueError,RuntimeError)):
        host.joint_to_actuator({group.joints[0]:Axis(2.7),group.joints[1]:Axis(2.7)})
    zero=host.joint_to_actuator({name:Axis() for name in group.joints})
    assert max(abs(a.position) for a in zero.values())<1e-7


def test_linkages_round_trip_across_model_workspace_and_quantized_feedback(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    from robot_transmission import Axis, Model
    from robot_transmission.mujoco_positions import JointPositionTargets
    host = Model.load(str(build(tmp_path)))
    mj = mujoco.MjModel.from_xml_path(str(MODEL / 'mujoco/full_body_robot.xml'))
    config = json.loads(ET.parse(MODEL / 'mujoco/full_body_robot.xml').find(
        'custom/text[@name="robot_sim_joint_position"]').get('data'))
    reference = JointPositionTargets(mj, config)
    for group in host.groups:
        if len(group.actuators) != 2:
            continue
        accepted = 0
        axes = [np.linspace(*config['joint_limits'][name], 9) for name in group.joints]
        for q0 in axes[0]:
            for q1 in axes[1]:
                data = mujoco.MjData(mj)
                try:
                    reference.apply(data, group.joints, [q0, q1])
                except ValueError:
                    continue  # Outside the model's coupled workspace.
                source = {name: Axis(q) for name, q in zip(group.joints, [q0, q1])}
                inputs = host.joint_to_actuator(source)
                restored = host.actuator_to_joint(inputs)
                for name in source:
                    assert restored[name].position == pytest.approx(source[name].position, abs=1e-7)
                # MCU float/int quantization must not prevent normal feedback conversion.
                if q0 != axes[0][0] and q0 != axes[0][-1] and q1 != axes[1][0] and q1 != axes[1][-1]:
                    rounded = host.actuator_to_joint({name: Axis(round(axis.position, 4))
                                                    for name, axis in inputs.items()})
                    for name in source:
                        assert rounded[name].position == pytest.approx(source[name].position, abs=.002)
                accepted += 1
        assert accepted >= 10, group.name
        reference.apply(mujoco.MjData(mj), group.joints, [0., 0.])


def test_host_control_description_accepts_full_output_and_partial_input_sessions(tmp_path):
    from motor_hardware_interface.workbench_control import WorkbenchModel
    from motor_hardware_interface.control_subset import materialize_control_subset
    driver = build(tmp_path)
    urdf = str(tmp_path / 'urdf/full_body_robot.hil.urdf')
    complete = WorkbenchModel(str(driver), urdf)
    assert len(complete.joint_names) == 37
    assert complete.position_tolerance['left_wrist_pitch'] > 0
    session = tmp_path / 'session'; session.mkdir()
    subset = materialize_control_subset(str(driver), '6,7,8,13', session)
    partial = WorkbenchModel(subset, urdf)
    assert set(partial.joint_names) == {'left_scapula_yaw', 'left_scapula_roll',
                                      'left_shoulder_pitch', 'left_wrist_flange_proximal'}
    assert partial.dynamic_joint_names == ['left_scapula_yaw', 'left_scapula_roll']
    positions = dict.fromkeys(partial.joint_names, 0.)
    assert set(partial.active_limits(positions)) == set(partial.joint_names)


def test_hil_host_reuses_mujoco_dynamic_limits_and_preserves_subset_path(tmp_path):
    from motor_hardware_interface.workbench_control import WorkbenchModel
    from motor_hardware_interface.control_subset import materialize_control_subset
    from robot_transmission.coupled_limits import CoupledLimits
    driver = build(tmp_path)
    config = yaml.safe_load(driver.read_text())['motor_hardware']
    path = driver.parent / config['coupled_limits_file']
    expected = json.loads(ET.parse(MODEL / 'mujoco/full_body_robot.xml').find(
        'custom/text[@name="robot_sim_joint_position"]').get('data'))
    assert yaml.safe_load(path.read_text()) == {
        key: expected[key] for key in ['joint_limits', 'coupled_limits']}
    session = tmp_path / 'session'
    session.mkdir()
    subset = materialize_control_subset(str(driver), '13,14', session)
    model = WorkbenchModel(subset, str(tmp_path / 'urdf/full_body_robot.hil.urdf'))
    assert set(model.dynamic_joint_names) == {'left_wrist_pitch', 'left_wrist_roll'}
    reference = CoupledLimits(expected['coupled_limits'], expected['joints'])
    positions = dict.fromkeys(expected['joints'], 0.)
    for pitch in (0., .25, -.25):
        positions['left_wrist_pitch'] = pitch
        bounds = reference.active_limits(expected['joint_limits'], positions)
        assert model.active_limits({name: positions[name] for name in model.joint_names}) == {
            name: bounds[name] for name in model.joint_names}


def test_hil_quantized_wrist_endpoint_keeps_dynamic_limits_and_can_move_back(tmp_path):
    from motor_hardware_interface.workbench_control import WorkbenchModel
    from sensor_msgs.msg import JointState
    driver = build(tmp_path)
    model = WorkbenchModel(str(driver), str(tmp_path / 'urdf/full_body_robot.hil.urdf'))
    positions = dict.fromkeys(model.joint_names, 0.)
    positions.update(left_wrist_pitch=.2618197668203044,
                     left_wrist_roll=.0001795660080412916)
    message = JointState(name=model.joint_names, position=list(positions.values()),
                         velocity=[0.] * len(positions), effort=[0.] * len(positions))
    message.header.stamp.sec = 10
    measured = model.feedback(message, 10_000_000_000)
    bounds = model.active_limits({name: axis.position for name, axis in measured.items()})
    assert bounds['left_wrist_pitch'][1] == pytest.approx(.26179938779914946)
    target = JointState(name=['left_wrist_pitch'], position=[.25], header=message.header)
    result = model.joint_target(target, measured, 10_000_000_000)
    assert result.position[list(result.name).index('left_wrist_pitch')] == .25
