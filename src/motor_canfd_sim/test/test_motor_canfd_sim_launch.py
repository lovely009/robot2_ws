"""静态检查 motor_canfd_sim 独立 launch，不启动 Gazebo 或任何进程。"""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from motor_canfd_sim.helpers import evaluate_node_parameters, expected_default_config_path


def _load_launch_description():
    launch_file = Path(__file__).parents[1] / "launch" / "motor_canfd_sim.launch.py"
    spec = importlib.util.spec_from_file_location("motor_canfd_sim_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_defaults_to_sim_side_motor_config_and_vcan_buses():
    description = _load_launch_description()
    context = LaunchContext()

    declarations = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    for declaration in declarations.values():
        declaration.execute(context)

    assert context.launch_configurations["interface_left_arm"] == "vcan0"
    assert context.launch_configurations["interface_right_arm"] == "vcan1"

    simulator = next(action for action in description.entities if isinstance(action, Node))
    parameters = evaluate_node_parameters(context, simulator)
    injected = next(parameter for parameter in parameters if isinstance(parameter, dict))

    # 独立 launch 默认使用本包的仿真侧拓扑配置，而不是真机配置。
    expected_config = expected_default_config_path()
    assert Path(injected["motor_config_file"]).resolve() == expected_config.resolve()
    assert injected["bus_override.left_arm"] == "vcan0"
    assert injected["bus_override.right_arm"] == "vcan1"
