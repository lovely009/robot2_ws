"""xacro 渲染一致性：ros2_control 关节清单必须与电机拓扑配置完全一致。

真实渲染 upper_body.urdf.xacro（不启动 ROS 节点），验证：
- ros2_control joint 集合 == YAML motors joint 集合 == control_joints 参数；
- 每个 ros2_control joint 都存在于 URDF 物理关节中（防止 YAML 与物理模型脱节）。
"""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from motor_hardware_interface.helpers import motor_joints_from_config

HERE = Path(__file__).parent
MODEL_XACRO = HERE.parent / "urdf" / "upper_body.urdf.xacro"
DEFAULT_CONFIG = HERE.parent / "config" / "motor_driver.yaml"
OVERRIDE_CONFIG = HERE / "fixtures" / "override_motor_config.yaml"


def _render_xacro(control_joints=None, config_file=None, bus_overrides=None):
    command = ["xacro", str(MODEL_XACRO)]
    if bus_overrides is not None:
        command.append(f"bus_overrides:={bus_overrides}")
    if config_file is not None:
        command.append(f"motor_config_file:={config_file}")
    if control_joints is not None:
        command.append(f"control_joints:={control_joints}")
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout)


def _joint_sets(root):
    physical = {joint.get("name") for joint in root.findall("joint")}
    control = {
        joint.get("name") for joint in root.findall("ros2_control/joint")
    }
    return physical, control


def test_default_model_control_joints_match_config():
    joints = motor_joints_from_config(DEFAULT_CONFIG)
    control_joints = ",".join(joints)
    root = _render_xacro(
        control_joints,
        str(DEFAULT_CONFIG),
        "left_arm:/dev/ttyACM0,right_arm:/dev/ttyACM1",
    )
    physical, control = _joint_sets(root)

    assert control == set(joints), (
        f"ros2_control 关节清单 {sorted(control)} 与 YAML {sorted(joints)} 不一致"
    )
    missing = control - physical
    assert not missing, f"ros2_control 引用了不存在的物理关节：{sorted(missing)}"


def test_override_model_control_joints_follow_config():
    # 覆盖配置只有 3 台电机（混合 socketcan/serial_slcan），清单必须自动跟随。
    joints = motor_joints_from_config(OVERRIDE_CONFIG)
    control_joints = ",".join(joints)
    root = _render_xacro(
        control_joints,
        str(OVERRIDE_CONFIG),
        "left_arm:can0,right_arm:/dev/ttyACM1",
    )
    physical, control = _joint_sets(root)

    assert control == set(joints), (
        f"覆盖配置 ros2_control 关节清单 {sorted(control)} "
        f"与 YAML {sorted(joints)} 不一致"
    )
    missing = control - physical
    assert not missing, f"ros2_control 引用了不存在的物理关节：{sorted(missing)}"


def test_default_xacro_args_render_without_parameters():
    # 不带任何参数渲染时，control_joints 默认值应与默认配置一致。
    root = _render_xacro()
    joints = motor_joints_from_config(DEFAULT_CONFIG)
    _, control = _joint_sets(root)
    assert control == set(joints)
