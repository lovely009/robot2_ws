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


def enumerate_simulation_models(root=None):
    """列出具有完整仿真拓扑的已安装模型。"""
    models_root = Path(root) if root is not None else default_models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir()
        and (entry / "config" / "simulation_topology.yaml").is_file()
    )


def enumerate_mujoco_models(root=None):
    """列出同时具备 MJCF 与 standalone URDF 的 MuJoCo 模型。

    MJCF 用于物理仿真，standalone URDF 用于渲染 robot_description；缺一不可。
    """
    models_root = Path(root) if root is not None else default_models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir()
        and (entry / "mujoco" / f"{entry.name}.xml").is_file()
        and (entry / "urdf" / f"{entry.name}.standalone.urdf").is_file()
    )
