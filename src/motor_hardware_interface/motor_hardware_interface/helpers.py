"""launch 与静态测试共用的配置辅助。

驱动侧 position controller 的 joints 列表由权威电机拓扑配置生成，
不再在 driver_controllers.yaml 里手抄，避免"关节顺序多处副本"漂移。
"""

import tempfile
from pathlib import Path
from typing import List, Tuple

import yaml
from launch.substitutions import LaunchConfiguration, TextSubstitution


def motor_joints_from_config(config_path) -> List[str]:
    """按 motor_canfd.yaml 的 motors 声明顺序返回关节名列表。

    PyYAML 的 dict 保持插入顺序，因此返回顺序即配置中的电机顺序，
    与模拟器/Gazebo 的关节顺序约定一致。
    """

    document = yaml.safe_load(Path(config_path).read_text())
    motors = document["motor_hardware"]["motors"]
    return [motor["joint"] for motor in motors.values()]


def read_buses(config_path) -> List[Tuple[str, str]]:
    """按 motor_canfd.yaml 的 buses 段返回 [(逻辑总线名, 默认接口), ...]。

    每一条逻辑总线取 ``interface``（供 launch 的 interface_<总线名> 参数
    作为默认值）。
    """

    document = yaml.safe_load(Path(config_path).read_text())
    buses = document["motor_hardware"]["buses"]
    result = []
    for name, node in buses.items():
        result.append((name, node["interface"]))
    return result


def bus_overrides_pieces(buses: List[Tuple[str, str]]):
    """构造 xacro 的 bus_overrides 替换片段："name:接口,..."。

    每个接口取自 launch 参数 interface_<总线名>，总线数量任意。
    """

    pieces = []
    for index, (name, _) in enumerate(buses):
        if index:
            pieces.append(TextSubstitution(text=","))
        pieces.append(TextSubstitution(text=f"{name}:"))
        pieces.append(LaunchConfiguration(f"interface_{name}"))
    return pieces


def write_position_joints_params_file(joints: List[str]) -> str:
    """把 joints 写入标准嵌套格式的参数文件并返回路径。

    launch_ros 会把参数 dict 里的列表序列化成 `!!python/tuple`，rcl 解析后
    得到空列表；因此这里生成与 driver_controllers.yaml 同构的临时 YAML 文件，
    由 ros2_control_node 以 --params-file 方式加载。
    """

    # 两个命名空间使用独立列表副本，避免 yaml 生成锚点/别名（rcl 不支持）。
    content = {
        "position_controller": {"ros__parameters": {"joints": list(joints)}},
        "/driver/position_controller": {"ros__parameters": {"joints": list(joints)}},
    }
    with tempfile.NamedTemporaryFile(
        "w", suffix=".yaml", delete=False, encoding="utf-8"
    ) as handle:
        yaml.safe_dump(content, handle, allow_unicode=True, sort_keys=False)
        return handle.name
