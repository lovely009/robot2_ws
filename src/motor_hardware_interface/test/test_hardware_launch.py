"""静态检查 motor_hardware_interface 的真机 bring-up launch，不启动任何进程。"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from motor_hardware_interface.helpers import motor_joints_from_config


def _load_launch_module():
    launch_file = Path(__file__).parents[1] / "launch" / "hardware.launch.py"
    spec = importlib.util.spec_from_file_location("hardware_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_launch_description():
    module = _load_launch_module()
    return module.generate_launch_description()


def test_bringup_launch_requires_model_description_and_uses_hardware_config():
    description = _load_launch_description()
    context = LaunchContext()

    declarations = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    for declaration in declarations.values():
        declaration.execute(context)

    # 模型描述默认指向本包的 CAN-FD 真机模型 xacro，不依赖仿真包。
    expected_xacro = Path(get_package_share_directory("motor_hardware_interface")) / (
        "urdf/upper_body.urdf.xacro"
    )
    model_xacro = context.launch_configurations["model_xacro"]
    if not isinstance(model_xacro, str):
        model_xacro = perform_substitutions(context, [model_xacro])
    assert Path(model_xacro).resolve() == expected_xacro.resolve()
    assert context.launch_configurations["interface_left_arm"] == "can0"
    assert context.launch_configurations["interface_right_arm"] == "can1"

    expected_config = Path(get_package_share_directory("motor_hardware_interface")) / (
        "config/motor_canfd.yaml"
    )
    config_default = context.launch_configurations["motor_config_file"]
    if not isinstance(config_default, str):
        config_default = perform_substitutions(context, [config_default])
    assert Path(config_default).resolve() == expected_config.resolve()

    nodes = [action for action in description.entities if isinstance(action, Node)]
    packages = {node.node_package for node in nodes}
    assert "controller_manager" in packages
    assert "robot_state_publisher" in packages

    # joints 列表由权威真机配置生成并注入，不在 driver_controllers.yaml 手抄。
    expected_joints = motor_joints_from_config(
        Path(get_package_share_directory("motor_hardware_interface"))
        / "config"
        / "motor_canfd.yaml"
    )
    assert _load_launch_module().POSITION_JOINTS == expected_joints, (
        f"launch 注入的 joints 必须由配置生成："
        f"{_load_launch_module().POSITION_JOINTS} != {expected_joints}"
    )
