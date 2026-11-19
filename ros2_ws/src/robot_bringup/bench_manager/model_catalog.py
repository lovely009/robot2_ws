"""模型目录的只读查询；不依赖进程管理、仿真器或设备访问。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def default_models_root():
    """默认模型根目录（安装树的 robot_description share）；取不到返回 None。

    各枚举函数的 `root=` 参数可覆盖它；节点把 model_dir 下的 models/
    显式传入，可指源码目录，改模型不必重构建。
    """
    try:
        return Path(get_package_share_directory("robot_description")) / "models"
    except Exception:
        return None
