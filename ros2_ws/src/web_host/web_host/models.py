"""模型目录的只读视图（文件域）：臂视图的模型候选与 motor_id → 关节名映射。

只读文件、不起进程——与 bench_manager 的进程管理无关，所以留在 web_host
（static_server 的 /api/models 与 /api/model-joints 直接用，与静态几何同源）。
模型根目录由 model_dir（模型包根，可指源码目录）推导。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

# motor_id → 关节映射的来源文件，按 simulation_model_joints 的解析优先级排列。
_DRIVER_CONFIG = "config/motor_driver.yaml"
_TOPOLOGY_CONFIG = "config/simulation_topology.yaml"


def _models_root():
    """默认模型根目录（安装树的 robot_sim share）；取不到返回 None。

    各函数的 `root=` 参数可覆盖它：web_host.launch.py 的 model_dir 指向模型包根
    时，static_server 传 `<model_dir>/models`（可指源码目录）。
    """
    try:
        return Path(get_package_share_directory("robot_sim")) / "models"
    except Exception:
        return None


def enumerate_arm_models(root=None):
    """List models the arm view can switch between (have `urdf/<模型名>.urdf`)。

    判据只有主 URDF——臂视图不要求模型接进端到端仿真（那要
    simulation_topology.yaml / MJCF，见 bench_manager 的模型枚举）。因此列表里
    **可能**有没配电机映射的预览件（`has_joint_mapping_source` 为假）：它们渲染
    得出几何；页面清空电机映射，不沿用默认或其他模型。
    """
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        return []
    return sorted(
        entry.name
        for entry in models_root.iterdir()
        if entry.is_dir() and (entry / "urdf" / f"{entry.name}.urdf").is_file()
    )


def has_joint_mapping_source(model, root=None):
    """该模型有没有声明 motor_id → 关节映射的来源（`simulation_model_joints` 的 ①② 任一）。

    臂视图候选的判据只有主 URDF——几何能渲染就列出来；而**映射**另要 config 里的
    一份文件。两者不一致的模型（如 `robot_leg`、`zhoubuguanjie_youbi_001`：有 URDF 与
    控制器、但没配电机映射）在页面上退化成
    空映射 + 来源错误提示。调用方用它区分「没配」与「配错了」：
    前者可以跳过，后者必须报错——配了文件却解析不出来仍是契约缺项。
    """
    models_root = Path(root) if root is not None else _models_root()
    if (models_root is None or not model or model != Path(model).name
            or model in (".", "..")):
        return False
    return any((models_root / model / rel).is_file()
               for rel in (_DRIVER_CONFIG, _TOPOLOGY_CONFIG))


def simulation_model_joints(model, root=None):
    """读出某模型的**电机 id → 关节名**映射，供前端臂视图使用。

    前端靠这张表把 motor_id 落到关节上（关节角同步、状态选色、臂上小球点选）。
    它此前硬编码在 `web/js/core/store.js`，写死成 half_body_robot 的 18 台，
    于是**换模型就静默失效**：关节名对不上 → 那些关节角恒为 0、小球点不中，
    表现得像"电机没注册"。这里改为向模型配置取同一个真相源，加模型/改关节名
    不必再动前端。

    解析顺序（与 simulation.launch.py 同序）：
      ① 模型自带 `config/motor_driver.yaml` 的 `motor_hardware.motors`：每个条目
         同时带 `joint:` 与 `id:`，一步到位（half_body_robot 走这条）；
      ② 回落：`config/simulation_topology.yaml` 的 (电机名 → 关节) × motor_
         hardware_interface 权威驱动配置的 (电机名 → id)。新模型常常先有拓扑、
         后补 motor_driver.yaml；没有②时仅提供几何预览。

    Raises:
        ValueError: 目录/模型缺失、非法模型名、解析不出条目、条目缺 id/joint、id 重复。
    """
    models_root = Path(root) if root is not None else _models_root()
    if models_root is None or not models_root.is_dir():
        raise ValueError("找不到模型根目录（models/ 不存在或安装树不完整？）")
    # 防目录穿越：模型名只允许是单层目录名
    if not model or model != Path(model).name or model in (".", ".."):
        raise ValueError(f"非法模型名：{model!r}")

    driver_config = models_root / model / _DRIVER_CONFIG
    if driver_config.is_file():
        return _joints_from_driver_config(driver_config)
    return _joints_from_topology(model, models_root / model)


def _joints_from_driver_config(config):
    """一路：驱动配置里每个电机同时带 `id:` 与 `joint:`。"""
    with config.open("r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    motors = (doc.get("motor_hardware") or {}).get("motors")
    if not isinstance(motors, dict) or not motors:
        raise ValueError(f"{config} 里没有 motor_hardware.motors")

    joints = {}
    for name, spec in motors.items():
        if not isinstance(spec, dict):
            raise ValueError(f"电机 {name!r} 的条目不是映射")
        motor_id, joint = spec.get("id"), spec.get("joint")
        if motor_id is None or joint is None:
            raise ValueError(f"电机 {name!r} 缺 id 或 joint")
        _register_joint(joints, motor_id, joint)
    return dict(sorted(joints.items()))


def _joints_from_topology(model, model_dir):
    """二路：拓扑给 (电机名 → 关节)，id 由电机名回查权威驱动配置。"""
    topology_file = model_dir / _TOPOLOGY_CONFIG
    if not topology_file.is_file():
        raise ValueError(
            f"模型 {model!r} 既没有 config/motor_driver.yaml，"
            f"也没有 config/simulation_topology.yaml")
    try:
        driver_config = (
            Path(get_package_share_directory("motor_hardware_interface"))
            / "config" / "motor_driver.yaml")
    except Exception as exc:
        raise ValueError(
            f"找不到 motor_hardware_interface 的权威驱动配置：{exc}") from exc
    if not driver_config.is_file():
        raise ValueError(f"权威驱动配置不存在：{driver_config}")

    with topology_file.open("r", encoding="utf-8") as handle:
        topology = yaml.safe_load(handle) or {}
    entries = (topology.get("simulation") or {}).get("motors")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{topology_file} 里没有 simulation.motors")

    with driver_config.open("r", encoding="utf-8") as handle:
        driver = yaml.safe_load(handle) or {}
    motors = (driver.get("motor_hardware") or {}).get("motors")
    if not isinstance(motors, dict):
        raise ValueError(f"{driver_config} 里没有 motor_hardware.motors")

    joints = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError(f"{topology_file} 的 motors 条目不是映射：{entry!r}")
        name, joint = entry.get("motor"), entry.get("joint")
        if name is None or joint is None:
            raise ValueError(f"拓扑条目缺 motor 或 joint：{entry!r}")
        spec = motors.get(name)
        if not isinstance(spec, dict) or spec.get("id") is None:
            raise ValueError(f"拓扑电机 {name!r} 在 {driver_config} 里没有 id")
        _register_joint(joints, spec["id"], joint)
    return dict(sorted(joints.items()))


def _register_joint(joints, motor_id, joint):
    motor_id = int(motor_id)
    if motor_id in joints:
        raise ValueError(
            f"电机 id {motor_id} 重复（{joints[motor_id]!r} 与 {joint!r}）："
            f"固件注册表以裸 motor_id 为键，重号会锁存 CONFIG_CONFLICT")
    joints[motor_id] = str(joint)


def _motor_config_dir():
    """权威型号库所在目录（安装树的 motor_hardware_interface/config）。"""
    try:
        return (Path(get_package_share_directory("motor_hardware_interface"))
                / "config")
    except Exception:
        return None


def motor_library(root=None):
    """型号库 `motor.yaml` → {型号名: parameters}（取不到返回 {}）。

    设备页要用减速比把关节侧 rad/s 折算成**电机端 RPM**（设备对象 0x6080 存的是电机端
    量），系数只能取权威型号库——页面另抄一份迟早与固件侧的换算漂移。
    型号名即拓扑里的 `model_name`（固件 Profile 的 .model_name 与库的键同源）。
    """
    config_dir = Path(root) if root is not None else _motor_config_dir()
    if config_dir is None:
        return {}
    path = config_dir / "motor.yaml"
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    types = data.get("motor_types") or {}
    out = {}
    for name, spec in types.items():
        spec = spec or {}
        params = spec.get("parameters") or {}
        out[str(name)] = {"protocol": str(spec.get("protocol") or name), **params}
    return out
