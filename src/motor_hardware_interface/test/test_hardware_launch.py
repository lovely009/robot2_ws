"""静态检查 motor_hardware_interface 的真机 bring-up launch，不启动任何进程。"""

import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from motor_hardware_interface.helpers import motor_joints_from_config

HERE = Path(__file__).parent


def _load_launch_module():
    launch_file = HERE.parent / "launch" / "hardware.launch.py"
    spec = importlib.util.spec_from_file_location("hardware_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _execute_launch():
    """执行参数声明与 OpaqueFunction，返回 (actions, context)。"""
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = LaunchContext()
    opaque_function = None
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            opaque_function = action
    assert opaque_function is not None, "launch 必须包含 OpaqueFunction"
    actions = opaque_function.execute(context)
    return actions, context


def _manager_node(actions):
    for action in actions:
        if (
            isinstance(action, Node)
            and action.node_package == "controller_manager"
            and action.node_executable == "ros2_control_node"
        ):
            return action
    raise AssertionError("缺少 ros2_control_node")


def _xacro_command_strings(manager, context):
    """提取 robot_description 的 xacro 命令中的静态字符串片段（不执行命令）。"""
    parameters_list = vars(manager)["_Node__parameters"]
    for parameters in parameters_list:
        if not isinstance(parameters, dict):
            continue
        for key, value in parameters.items():
            name = perform_substitutions(context, list(key))
            if name != "robot_description":
                continue
            commands = value.value
            if not isinstance(commands, list):
                commands = [commands]
            for command in commands:
                if isinstance(command, Command):
                    return [
                        perform_substitutions(context, [piece])
                        for piece in vars(command)["_Command__command"]
                    ]
    raise AssertionError("manager 缺少 robot_description 参数")


def _joints_param_file(manager, context):
    # manager 参数：robot_description、controller_config（ParameterFile）、joints 参数文件。
    parameters_list = vars(manager)["_Node__parameters"]
    for parameters in parameters_list:
        if isinstance(parameters, ParameterFile):
            path = parameters._ParameterFile__param_file
            if isinstance(path, list):
                path = perform_substitutions(context, path)
            data = yaml.safe_load(Path(path).read_text())
            if (
                "position_controller" in data
                and "joints"
                in data["position_controller"].get("ros__parameters", {})
            ):
                return path
    raise AssertionError("缺少 joints 临时参数文件")


def test_bringup_launch_defaults_to_tty_buses():
    actions, context = _execute_launch()

    # 默认真机传输为 serial_slcan 双总线。
    assert context.launch_configurations["interface_left_arm"] == "/dev/ttyACM0"
    assert context.launch_configurations["interface_right_arm"] == "/dev/ttyACM1"

    default_config = (
        Path(get_package_share_directory("motor_hardware_interface"))
        / "config"
        / "motor_canfd.yaml"
    )
    expected_joints = motor_joints_from_config(default_config)

    command_text = "".join(_xacro_command_strings(_manager_node(actions), context))
    assert (
        " bus_overrides:=left_arm:/dev/ttyACM0,right_arm:/dev/ttyACM1"
        in command_text
    )
    assert f" control_joints:={','.join(expected_joints)}" in command_text
    assert f" motor_config_file:={default_config}" in command_text

    # joints 参数文件内容与权威配置一致（单一来源，不在 controller yaml 手抄）。
    params_file = _joints_param_file(_manager_node(actions), context)
    data = yaml.safe_load(Path(params_file).read_text())
    assert data["position_controller"]["ros__parameters"]["joints"] == expected_joints
    assert (
        data["/driver/position_controller"]["ros__parameters"]["joints"]
        == expected_joints
    )


def test_bringup_launch_override_config_drives_everything():
    # 覆盖 motor_config_file 时，joints 与 xacro 参数全部以实际配置为准；
    # interface_<总线名> 可继续覆盖设备路径（此处把 left_arm 切到 can0）。
    override = HERE / "fixtures" / "override_motor_config.yaml"
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = LaunchContext()
    opaque_function = None
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            opaque_function = action
    context.launch_configurations["motor_config_file"] = str(override)
    context.launch_configurations["interface_left_arm"] = "can0"
    actions = opaque_function.execute(context)

    expected_joints = motor_joints_from_config(override)
    command_text = "".join(_xacro_command_strings(_manager_node(actions), context))
    assert " bus_overrides:=left_arm:can0,right_arm:/dev/ttyACM1" in command_text
    assert f" control_joints:={','.join(expected_joints)}" in command_text
    assert f" motor_config_file:={override}" in command_text

    params_file = _joints_param_file(_manager_node(actions), context)
    data = yaml.safe_load(Path(params_file).read_text())
    assert data["position_controller"]["ros__parameters"]["joints"] == expected_joints


def test_bringup_launch_rejects_unknown_bus():
    module = _load_launch_module()
    description = module.generate_launch_description()
    context = LaunchContext()
    opaque_function = None
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            opaque_function = action
    context.launch_configurations["motor_config_file"] = str(
        HERE / "fixtures" / "unknown_bus_motor_config.yaml"
    )
    with pytest.raises(Exception):
        opaque_function.execute(context)
