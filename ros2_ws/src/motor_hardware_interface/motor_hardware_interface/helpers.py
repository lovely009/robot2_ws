"""launch 与静态测试共用的配置辅助。

驱动侧 position controller 的 joints 列表由权威电机拓扑配置生成，
不再在 driver_controllers.yaml 里手抄，避免"关节顺序多处副本"漂移。
"""

from typing import List


def motor_joints_from_config(config_path) -> List[str]:
    """按 motor_driver.yaml 的 motors 声明顺序返回关节名列表。

    PyYAML 的 dict 保持插入顺序，因此返回顺序即配置中的电机顺序，
    与模拟器/Gazebo 的关节顺序约定一致。
    """

    from robot_transmission import Model
    return list(Model.load(str(config_path)).joint_names)
