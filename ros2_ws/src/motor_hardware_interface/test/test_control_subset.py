from pathlib import Path
import pytest
import yaml
from robot_transmission import Model
from motor_hardware_interface.control_subset import materialize_control_subset


def test_spatial_subset_preserves_parameters_and_rejects_split_group(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    driver = Path(get_package_share_directory('robot_description')) / 'models/trial_arm_7dof/config/motor_driver.yaml'
    original = Model.load(str(driver)).groups[0]
    for ids in ('1', '2'):
        with pytest.raises(ValueError):
            materialize_control_subset(str(driver), ids, tmp_path)
    subset = Model.load(materialize_control_subset(str(driver), '1,2', tmp_path))
    assert subset.actuator_ids == {'arm_joint1': 1, 'arm_joint2': 2}
    group = subset.groups[0]
    assert group.position_dependent
    assert group.parameters == original.parameters
    assert group.actuator_reduction == original.actuator_reduction
    assert group.actuators == original.actuators and group.joints == original.joints


def source(tmp_path):
    driver=tmp_path/'original'/'motor_driver.yaml';driver.parent.mkdir()
    driver.write_text('motor_hardware:\n  motor_specs_file: specs.yaml\n  motors:\n'
        '    direct: {id: 2, joint: elbow}\n    a: {id: 8}\n    b: {id: 9}\n')
    (driver.parent/'transmissions.yaml').write_text('transmissions:\n  wrist:\n'
        '    plugin: transmission_interface/DifferentialTransmission\n'
        '    actuators: [a, b]\n    joints: [yaw, pitch]\n'
        '    actuator_reduction: [2, 3]\n    joint_offset: [0.1, 0.2]\n')
    return driver


def test_single_motor_subset_preserves_original_model_and_paths(tmp_path):
    driver=source(tmp_path);destination=tmp_path/'session';destination.mkdir()
    result=materialize_control_subset(str(driver),'2',destination)
    assert Model.load(result).actuator_ids=={'direct':2}
    assert list(Model.load(result).joint_names)==['elbow']
    doc=yaml.safe_load(Path(result).read_text())['motor_hardware']
    assert doc['motor_specs_file']==str(driver.parent/'specs.yaml')
    assert set(Model.load(str(driver)).actuator_ids.values())=={2,8,9}


@pytest.mark.parametrize('ids',['8','2,8','2,2','0','65','100','not-an-id'])
def test_rejects_partial_group_unknown_and_invalid_ids(tmp_path,ids):
    driver=source(tmp_path);destination=tmp_path/'session';destination.mkdir()
    with pytest.raises(ValueError):materialize_control_subset(str(driver),ids,destination)


def test_complete_differential_group_keeps_transform_parameters(tmp_path):
    driver=source(tmp_path);destination=tmp_path/'session';destination.mkdir()
    result=Model.load(materialize_control_subset(str(driver),'8,9',destination))
    group=result.groups[0]
    assert list(group.joints)==['yaw','pitch']
    assert list(group.actuator_reduction)==[2.,3.]
    assert list(group.joint_offset)==[.1,.2]


def test_half_body_motor_2_requires_only_its_own_joint_feedback(tmp_path):
    from ament_index_python.packages import get_package_share_directory
    from motor_hardware_interface.workbench_control import WorkbenchModel
    from sensor_msgs.msg import JointState
    model=Path(get_package_share_directory('robot_description'))/'models/half_body_robot'
    config=materialize_control_subset(str(model/'config/motor_driver.yaml'),'2',tmp_path)
    mapping=WorkbenchModel(config,str(model/'urdf/half_body_robot.urdf'))
    assert list(mapping.ids.values())==[2]
    assert mapping.joint_names==['joint_link2']
    frame=JointState(name=['joint_link2'],position=[0.],velocity=[0.],effort=[0.])
    frame.header.stamp.sec=10
    assert list(mapping.feedback(frame,10_000_000_000))==['joint_link2']

