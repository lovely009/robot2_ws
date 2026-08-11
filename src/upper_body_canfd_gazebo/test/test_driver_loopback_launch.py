"""静态检查 driver_loopback 的启动时序契约，不启动任何进程。

断言：模拟器启动后才拉起驱动侧 controller_manager；spawner 按
joint_state_broadcaster → position_controller 顺序执行；模拟器退出时整个回环
终止（fail-fast）。
"""

import importlib.util
from pathlib import Path

from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from motor_canfd_sim.helpers import (
    event_handler_actions,
    event_handler_matcher,
    node_arguments_contain,
)


def _load_launch_description():
    launch_file = Path(__file__).parents[1] / "launch" / "driver_loopback.launch.py"
    spec = importlib.util.spec_from_file_location("driver_loopback_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _collect_nodes(description):
    # 驱动侧节点只出现在事件处理器的 on_event 动作列表里，不在 entities 顶层。
    nodes = []
    for action in description.entities:
        if isinstance(action, RegisterEventHandler):
            actions = event_handler_actions(action.event_handler)
            if isinstance(actions, list):
                nodes.extend(item for item in actions if isinstance(item, Node))
    return nodes


def test_startup_and_shutdown_event_chain():
    description = _load_launch_description()

    nodes = _collect_nodes(description)
    driver_manager = next(
        node for node in nodes
        if node.node_package == "controller_manager"
        and node.node_executable == "ros2_control_node"
    )
    jsb = next(
        node for node in nodes
        if node.node_package == "controller_manager"
        and node.node_executable == "spawner"
        and node_arguments_contain(node, "joint_state_broadcaster")
    )
    position = next(
        node for node in nodes
        if node.node_package == "controller_manager"
        and node.node_executable == "spawner"
        and node_arguments_contain(node, "position_controller")
    )

    handlers = [
        action.event_handler
        for action in description.entities
        if isinstance(action, RegisterEventHandler)
    ]
    starts = [h for h in handlers if isinstance(h, OnProcessStart)]
    exits = [h for h in handlers if isinstance(h, OnProcessExit)]

    # 模拟器启动（谓词匹配）→ 驱动侧 controller_manager。
    assert any(
        callable(event_handler_matcher(h)) and driver_manager in event_handler_actions(h)
        for h in starts
    ), "缺少『模拟器启动后拉起驱动侧 manager』的事件处理"
    # 驱动侧 manager 就绪 → joint_state_broadcaster。
    assert any(
        event_handler_matcher(h) is driver_manager
        and jsb in event_handler_actions(h)
        for h in starts
    ), "缺少『manager 就绪后拉起 joint_state_broadcaster』的事件处理"
    # joint_state_broadcaster 退出 → position_controller。
    assert any(
        event_handler_matcher(h) is jsb and position in event_handler_actions(h)
        for h in exits
    ), "缺少『joint_state_broadcaster 退出后拉起 position_controller』的事件处理"
    # 模拟器退出（谓词匹配）→ 终止整个回环。
    assert any(
        callable(event_handler_matcher(h))
        and any(isinstance(action, Shutdown) for action in event_handler_actions(h))
        for h in exits
    ), "缺少『模拟器退出时终止回环』的事件处理"


def test_event_handlers_are_registered_before_model_sim():
    description = _load_launch_description()
    handler_indices = [
        index
        for index, action in enumerate(description.entities)
        if isinstance(action, RegisterEventHandler)
    ]
    include_index = next(
        index
        for index, action in enumerate(description.entities)
        if isinstance(action, IncludeLaunchDescription)
    )
    assert handler_indices and max(handler_indices) < include_index, (
        "事件处理器必须在 model_sim 执行之前注册，否则会漏掉模拟器启动事件"
    )
