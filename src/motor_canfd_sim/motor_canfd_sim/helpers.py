"""launch 与静态测试共用的公共辅助。

- 默认仿真配置路径：launch 用它做 motor_config_file 默认值，测试断言注入值；
- launch 静态测试访问器：集中封装 launch_ros 私有属性（_Node__parameters、
  _OnActionEventBase__*、_Node__arguments）。升级 launch_ros 时私有 API 变化
  只影响本模块，一处替换为公开 API 即可。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.utilities import evaluate_parameters
from launch_ros.substitutions import FindPackageShare


def default_config_rel_path() -> str:
    """仿真侧电机拓扑配置在包内的相对路径。"""
    return "config/motor_canfd.yaml"


def default_config_share_substitution():
    """launch 中默认 motor_config_file 的路径替换表达式。"""
    return PathJoinSubstitution(
        [FindPackageShare("motor_canfd_sim"), default_config_rel_path()]
    )


def expected_default_config_path() -> Path:
    """静态测试期望注入的默认配置绝对路径。"""
    return Path(get_package_share_directory("motor_canfd_sim")) / default_config_rel_path()


def evaluate_node_parameters(context, node):
    """求值 launch_ros Node 的参数注入结果（封装 _Node__parameters）。"""
    return evaluate_parameters(context, node._Node__parameters)


def event_handler_matcher(handler):
    """读取事件处理器的 action matcher（launch_ros 私有属性）。"""
    return handler._OnActionEventBase__action_matcher


def event_handler_actions(handler):
    """读取事件处理器的 on_event 动作列表（launch_ros 私有属性）。"""
    return handler._OnActionEventBase__actions_on_event


def node_arguments_contain(node, token) -> bool:
    """判断 launch_ros Node 的 argv 是否包含给定片段（_Node__arguments）。"""
    return any(token in str(argument) for argument in node._Node__arguments)
