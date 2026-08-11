"""launch 与静态测试共用的配置辅助。

驱动侧 position controller 的 joints 列表由权威电机拓扑配置生成，
不再在 driver_controllers.yaml 里手抄，避免"关节顺序多处副本"漂移。
"""

import tempfile
from pathlib import Path
from typing import List

import yaml


def motor_joints_from_config(config_path) -> List[str]:
    """按 motor_canfd.yaml 的 motors 声明顺序返回关节名列表。

    PyYAML 的 dict 保持插入顺序，因此返回顺序即配置中的电机顺序，
    与模拟器/Gazebo 的关节顺序约定一致。
    """

    document = yaml.safe_load(Path(config_path).read_text())
    motors = document["motor_hardware"]["motors"]
    return [motor["joint"] for motor in motors.values()]


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
