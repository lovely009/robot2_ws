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
from motor_hardware_interface.helpers import motor_joints_from_config, read_buses
from upper_body_canfd_gazebo.motion_library import JOINT_LIMITS, JOINT_NAMES


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


def test_driver_controllers_yaml_has_no_handcopied_joints() -> None:
    """driver_controllers.yaml 不再手抄 joints 列表：由 launch 从配置生成（单一来源）。"""
    document = yaml.safe_load(DRIVER_CONTROLLERS_CONFIG.read_text())
    for namespace in ("position_controller", "/driver/position_controller"):
        parameters = document[namespace]["ros__parameters"]
        assert "joints" not in parameters, (
            f"{namespace} 的 joints 必须由 launch 从 motor_canfd.yaml 生成，"
            "不应在 driver_controllers.yaml 中维护"
        )


def test_config_generated_joints_match_sim_motor_order() -> None:
    """配置生成的 joints 列表必须等于仿真电机拓扑顺序（真机/仿真两侧一致）。"""
    _, sim_motors = _load_motors(SIM_CONFIG)
    sim_joints = [motor["joint"] for motor in sim_motors.values()]
    assert motor_joints_from_config(SIM_CONFIG) == sim_joints, (
        "motor_joints_from_config(仿真配置) 必须等于仿真电机拓扑顺序"
    )
    assert motor_joints_from_config(HARDWARE_CONFIG) == sim_joints, (
        "motor_joints_from_config(真机配置) 必须与仿真电机拓扑顺序一致（同一台机器人）"
    )


def test_config_buses_feed_launch_interface_arguments() -> None:
    """launch 从配置 buses 段生成 interface_<总线名> 参数（总线数量任意）。"""

    assert read_buses(SIM_CONFIG) == [
        ("left_arm", "vcan0"),
        ("right_arm", "vcan1"),
    ]
    assert read_buses(HARDWARE_CONFIG) == [
        ("left_arm", "can0"),
        ("right_arm", "can1"),
    ]


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


def _urdf_joint_limits(xacro_path: Path) -> List[Tuple[float, float]]:
    """按通用关节顺序提取 URDF xacro 的 joint limit（左右臂共用同一 arm 宏）。"""

    text = xacro_path.read_text()
    limits: List[Tuple[float, float]] = []
    for name in JOINT_NAMES:
        pattern = re.compile(
            r'<joint name="\$\{side\}_' + name + r'_joint" type="revolute">'
            r"(.*?)</joint>",
            re.S,
        )
        joint = pattern.search(text)
        assert joint is not None, f"URDF xacro 中找不到关节 {name}"
        limit = re.search(
            r'<limit lower="([^"]+)" upper="([^"]+)"', joint.group(1)
        )
        assert limit is not None, f"URDF xacro 关节 {name} 缺少 limit"
        limits.append((float(limit.group(1)), float(limit.group(2))))
    return limits


def test_motion_library_joint_limits_match_urdf() -> None:
    """动作库限位必须与 URDF joint limit 逐对一致（跨源断言，防止双写漂移）。"""

    xacro_path = (
        Path(get_package_share_directory("upper_body_canfd_gazebo"))
        / "urdf"
        / "upper_body.urdf.xacro"
    )
    urdf_limits = _urdf_joint_limits(xacro_path)
    library_limits = [tuple(limit) for limit in JOINT_LIMITS]
    assert urdf_limits == library_limits, (
        "动作库 JOINT_LIMITS 必须与 URDF joint limit 逐对一致：\n"
        f"urdf:   {urdf_limits}\nlibrary: {library_limits}"
    )
