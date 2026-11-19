"""launch 与静态测试共用的配置辅助。

驱动侧 position controller 的 joints 列表由权威电机拓扑配置生成，
避免关节顺序多处副本漂移。
"""

from typing import List


def motor_joints_from_config(config_path) -> List[str]:
    """按 motor_driver.yaml 的 motors 声明顺序返回关节名列表。

    返回共享传动模型中的关节顺序。
    """

    from robot_transmission import Model
    return list(Model.load(str(config_path)).joint_names)
