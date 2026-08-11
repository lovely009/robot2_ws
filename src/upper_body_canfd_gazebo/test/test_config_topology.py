"""静态检查真机/仿真配置的拓扑一致性与顺序耦合，不启动 Gazebo 或任何进程。

覆盖 ADR 0002 的拓扑一致性承诺，以及 motor_canfd_sim README 里
"URDF、effort controller 关节顺序、仿真电机拓扑三者一致"的约束。
新增关节或重排顺序时，driver position 控制器与动作库也会被这里兜住。
"""

import re
from pathlib import Path
from typing import Dict, List, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
from upper_body_canfd_gazebo.motion_library import JOINT_NAMES


def _config_path(package: str, name: str) -> Path:
    return Path(get_package_share_directory(package)) / "config" / name


HARDWARE_CONFIG = _config_path("motor_hardware_interface", "motor_canfd.yaml")
SIM_CONFIG = _config_path("motor_canfd_sim", "motor_canfd.yaml")
CONTROLLERS_CONFIG = _config_path("upper_body_canfd_gazebo", "controllers.yaml")
DRIVER_CONTROLLERS_CONFIG = _config_path("motor_hardware_interface", "driver_controllers.yaml")


def _load_motors(path: Path) -> Tuple[Dict[str, object], Dict[str, dict]]:
    document = yaml.safe_load(path.read_text())
    hardware = document["motor_hardware"]
    return hardware["buses"], hardware["motors"]


def _motor_topo(motor_node: dict) -> Tuple[str, str, str, int]:
    return (motor_node["joint"], motor_node["bus"], motor_node["type"], motor_node["id"])


def test_real_and_sim_motor_topology_are_identical() -> None:
    real_buses, real_motors = _load_motors(HARDWARE_CONFIG)
    sim_buses, sim_motors = _load_motors(SIM_CONFIG)

    assert set(real_buses) == set(sim_buses), (
        f"逻辑总线不一致：real={sorted(real_buses)} sim={sorted(sim_buses)}"
    )
    assert set(real_motors) == set(sim_motors), (
        f"电机清单不一致：only_real={sorted(set(real_motors) - set(sim_motors))} "
        f"only_sim={sorted(set(sim_motors) - set(real_motors))}"
    )
    for name in real_motors:
        assert _motor_topo(real_motors[name]) == _motor_topo(sim_motors[name]), (
            f"电机 '{name}' 拓扑不一致：real={_motor_topo(real_motors[name])} "
            f"sim={_motor_topo(sim_motors[name])}"
        )


def test_effort_controller_order_matches_sim_motor_order() -> None:
    document = yaml.safe_load(CONTROLLERS_CONFIG.read_text())
    joints: List[str] = document["effort_controller"]["ros__parameters"]["joints"]
    _, sim_motors = _load_motors(SIM_CONFIG)
    sim_joints = [motor["joint"] for motor in sim_motors.values()]
    assert joints == sim_joints, (
        "Gazebo effort controller 关节顺序必须与仿真电机拓扑顺序一致：\n"
        f"controllers.yaml: {joints}\nsim config:      {sim_joints}"
    )


def test_urdf_canfd_joint_order_matches_sim_motor_order() -> None:
    xacro_path = (
        Path(get_package_share_directory("upper_body_canfd_gazebo"))
        / "urdf"
        / "upper_body.urdf.xacro"
    )
    urdf_joints = re.findall(r'canfd_control_joint joint="([^"]+)"', xacro_path.read_text())
    _, sim_motors = _load_motors(SIM_CONFIG)
    sim_joints = [motor["joint"] for motor in sim_motors.values()]
    assert urdf_joints == sim_joints, (
        "URDF canfd 后端关节顺序必须与仿真电机拓扑顺序一致：\n"
        f"urdf:          {urdf_joints}\nsim config:    {sim_joints}"
    )


def test_driver_position_controller_order_matches_sim_motor_order() -> None:
    """真机与 /driver 回环共用 driver_controllers.yaml：两个命名空间的 position
    列表都必须与仿真电机拓扑顺序一致，否则命令会发到错误关节。"""

    document = yaml.safe_load(DRIVER_CONTROLLERS_CONFIG.read_text())
    _, sim_motors = _load_motors(SIM_CONFIG)
    sim_joints = [motor["joint"] for motor in sim_motors.values()]
    for namespace in ("position_controller", "/driver/position_controller"):
        joints: List[str] = document[namespace]["ros__parameters"]["joints"]
        assert joints == sim_joints, (
            "driver position controller 关节顺序必须与仿真电机拓扑顺序一致：\n"
            f"{namespace}: {joints}\nsim config:  {sim_joints}"
        )


def test_motion_library_joint_names_match_sim_arm_order() -> None:
    """动作库的 7 个通用关节名必须等于任一臂去掉臂前缀后的顺序。"""

    _, sim_motors = _load_motors(SIM_CONFIG)
    sim_joints = [motor["joint"] for motor in sim_motors.values()]
    expected_left = [f"left_{name}_joint" for name in JOINT_NAMES]
    expected_right = [f"right_{name}_joint" for name in JOINT_NAMES]
    assert sim_joints[:7] == expected_left, (
        f"仿真配置左臂关节必须等于 left_ + JOINT_NAMES + _joint：\n"
        f"sim config:    {sim_joints[:7]}\nexpected left: {expected_left}"
    )
    assert sim_joints[7:] == expected_right, (
        f"仿真配置右臂关节必须等于 right_ + JOINT_NAMES + _joint：\n"
        f"sim config:    {sim_joints[7:]}\nexpected right: {expected_right}"
    )
