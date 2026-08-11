"""静态检查 motor_hardware_interface 的真机 bring-up launch，不启动任何进程。"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def _load_launch_description():
    launch_file = Path(__file__).parents[1] / "launch" / "hardware.launch.py"
    spec = importlib.util.spec_from_file_location("hardware_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
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

    # 模型描述必须由调用方显式提供，本 launch 不默认指向任何仿真包。
    assert context.launch_configurations["model_xacro"] == ""
    assert context.launch_configurations["left_interface"] == "can0"
    assert context.launch_configurations["right_interface"] == "can1"

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
