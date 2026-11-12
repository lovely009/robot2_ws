"""静态检查 robot_sim 的 launch，不启动任何进程。

验证「模型选择」这条链真的贯通：
- 两个 launch 都声明 model，默认值在 models/ 下有对应目录，且该目录三件套齐全；
- gazebo.launch.py 渲染的 robot_description 命令指向 models/<model>/urdf/<model>.urdf.xacro；
- 控制器参数文件解析到 models/<model>/config/controllers.yaml 且真实存在。

注意：FindPackageShare 解析的是 install 下的 share，因此这些断言同时验证了
CMakeLists 的 install(models ...) 确实把模型装进了 share。
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

HERE = Path(__file__).parent
LAUNCH_DIR = HERE.parent / "launch"
MODEL = "half_body_robot"


def test_standalone_motor_launch_uses_installed_robot_sim():
    """独立电机模拟入口与可执行文件由统一仿真包提供。"""
    from ament_index_python.packages import get_package_share_directory

    launch_file = Path(get_package_share_directory("robot_sim")) / "launch/motor_sim.launch.py"
    spec = importlib.util.spec_from_file_location("standalone_motor_sim", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = [action for action in module.generate_launch_description().entities
             if isinstance(action, Node)]
    assert len(nodes) == 1
    node = nodes[0]
    assert node.node_package == "robot_sim"
    assert node.node_executable == "motor_sim_node"
    executable = Path(perform_substitutions(LaunchContext(), node.cmd[0]))
    assert executable.is_file()
    assert executable.parent.name == "robot_sim"


def _execute(launch_file):
    """加载 launch 模块并执行参数声明，返回 (actions, context)。"""
    spec = importlib.util.spec_from_file_location(launch_file.stem, launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return description.entities, context


def _package_share():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("robot_description"))


@pytest.mark.parametrize('backend', ['mujoco', 'gazebo'])
@pytest.mark.parametrize('managed_control', [False, True])
def test_simulation_includes_backend_from_sim_package(backend, managed_control, tmp_path, monkeypatch):
    """模型包分离后，总启动入口仍须从 robot_sim 加载被控对象 launch。"""
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription

    spec = importlib.util.spec_from_file_location('simulation_launch', LAUNCH_DIR / 'simulation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'backend': backend, 'model': 'half_body_robot',
        'controller_mode': 'firmware', 'transport': 'socketcan',
        'sim_interface': 'test_can', 'start_fdcan_sim': 'false',
        'start_web_host': 'false', 'start_micro_ros_agent': 'false',
        'start_host_control': 'false' if managed_control else 'true',
    })
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    monkeypatch.setattr(module, '_validate_socketcan_interface', lambda interface: 'can')
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    monkeypatch.setattr(module.tempfile, 'mkdtemp', lambda **kwargs: str(runtime))

    actions = module.launch_simulation(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    assert includes
    source = includes[0].launch_description_source
    source.get_launch_description(context)
    expected = Path(get_package_share_directory('robot_sim')) / 'launch' / f'{backend}.launch.py'
    assert source.location == str(expected)
    assert len(includes) == (1 if managed_control else 2)

    from launch.actions import OpaqueFunction
    from launch.events import Shutdown
    for _ in range(2):
        cleanup = actions[-1].event_handler.handle(Shutdown(reason='test'), context)
        assert all(isinstance(action, OpaqueFunction) for action in cleanup)
        for action in cleanup:
            action.execute(context)
    assert not runtime.exists()


def test_mujoco_cleanup_accepts_repeated_shutdown(tmp_path, monkeypatch):
    from launch.actions import OpaqueFunction
    from launch.events import Shutdown
    spec = importlib.util.spec_from_file_location('mujoco_launch', LAUNCH_DIR / 'mujoco.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _entities, context = _execute(LAUNCH_DIR / 'mujoco.launch.py')
    runtime = tmp_path / 'runtime'
    runtime.mkdir()
    monkeypatch.setattr(module.tempfile, 'mkdtemp', lambda **kwargs: str(runtime))
    actions = module._launch_setup(context)
    assert (runtime / 'bridge.yaml').is_file()
    for _ in range(2):
        cleanup = actions[-1].event_handler.handle(Shutdown(reason='test'), context)
        assert all(isinstance(action, OpaqueFunction) for action in cleanup)
        for action in cleanup:
            action.execute(context)
    assert not runtime.exists()


@pytest.mark.parametrize("available", ["ready", "missing_runtime", "missing_font"])
@pytest.mark.parametrize("pythonpath", ["", "/ros/python:/custom/python"])
def test_mujoco_bridge_loads_available_chinese_runtime(available, pythonpath, tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("mujoco_launch", LAUNCH_DIR / "mujoco.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    runtime = tmp_path / ".cache/robot2-mujoco-zh/runtime"
    font = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    present = set()
    if available != "missing_runtime":
        present.update({runtime / "build.json", runtime / "mujoco/libmujoco.so.3.6.0"})
    if available != "missing_font":
        present.add(font)
    monkeypatch.setattr(module.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(module.Path, "is_file", lambda path: path in present)
    monkeypatch.setattr(module.os, "environ", {
        "PYTHONPATH": pythonpath, "XDG_SESSION_TYPE": "wayland",
        "DISPLAY": ":0", "WAYLAND_DISPLAY": "wayland-0", "ROS_DOMAIN_ID": "93",
    })

    environment = module._bridge_environment()

    assert environment["PYGLFW_LIBRARY_VARIANT"] == "x11"
    assert "WAYLAND_DISPLAY" not in environment
    assert environment["ROS_DOMAIN_ID"] == "93"
    assert module.os.environ["WAYLAND_DISPLAY"] == "wayland-0"
    if available == "ready":
        assert environment["MUJOCO_LANGUAGE"] == "zh_CN"
        assert environment["MUJOCO_ZH_FONT"] == str(font)
        assert environment["PYTHONPATH"].split(":") == [str(runtime)] + (
            pythonpath.split(":") if pythonpath else [])
    else:
        assert "MUJOCO_LANGUAGE" not in environment
        assert environment["PYTHONPATH"] == pythonpath


def _robot_description_command(entities, context):
    """取出 robot_state_publisher 的 robot_description 命令串（运行期求值）。"""
    for action in entities:
        if not (isinstance(action, Node) and action.node_executable == "robot_state_publisher"):
            continue
        for parameters in vars(action)["_Node__parameters"]:
            if not isinstance(parameters, dict):
                continue
            for key, value in parameters.items():
                if perform_substitutions(context, list(key)) != "robot_description":
                    continue
                commands = value.value
                if not isinstance(commands, list):
                    commands = [commands]
                for command in commands:
                    if isinstance(command, Command):
                        return "".join(
                            perform_substitutions(context, [piece])
                            for piece in vars(command)["_Command__command"]
                        )
    raise AssertionError("未找到 robot_state_publisher 的 robot_description 命令")


@pytest.mark.parametrize(
    "launch_name", ["display.launch.py", "gazebo.launch.py", "mujoco.launch.py"])
def test_launch_selects_an_existing_model(launch_name):
    entities, context = _execute(LAUNCH_DIR / launch_name)

    assert "model" in context.launch_configurations, f"{launch_name} 未声明 model 参数"
    model = context.launch_configurations["model"]
    assert model == MODEL

    model_dir = _package_share() / "models" / model
    for relative in (
        f"urdf/{model}.urdf",
        f"urdf/{model}.urdf.xacro",
        "config/controllers.yaml",
    ):
        assert (model_dir / relative).is_file(), (
            f"{launch_name}: 默认模型 {model} 缺少 {relative}（是否漏装 models/？）"
        )
    assert len(entities) > 0


def test_mujoco_launch_default_model_has_mjcf():
    """MuJoCo 入口的被控对象就是模型自带的 MJCF：缺它这条入口根本起不来。"""
    entities, context = _execute(LAUNCH_DIR / "mujoco.launch.py")

    model = context.launch_configurations["model"]
    mjcf = _package_share() / "models" / model / "mujoco" / f"{model}.xml"
    assert mjcf.is_file(), (
        f"mujoco.launch.py 默认模型 {model} 缺少 mujoco/{model}.xml"
        f"（生成：python3 models/{model}/tools/build_mujoco_model.py）"
    )
    assert len(entities) > 0


def test_gazebo_launch_wires_model_into_xacro_and_controllers():
    entities, context = _execute(LAUNCH_DIR / "gazebo.launch.py")

    command = _robot_description_command(entities, context)
    assert f"/models/{MODEL}/urdf/{MODEL}.urdf.xacro" in command
    assert " fixed_base:=true" in command

    controllers_file = Path(context.launch_configurations["controllers_file"])
    assert controllers_file.is_file(), f"控制器参数文件不存在：{controllers_file}"
    assert controllers_file.parent == _package_share() / "models" / MODEL / "config"

    # 生成高度默认 0.0：默认模型的 base_link 网格 z ∈ [0.0000, 0.7029]，零位下全模型
    # 最低点恰为 z = 0.000000（见该模型 README 的实测几何），故无需抬起。
    assert context.launch_configurations["spawn_z"] == "0.0"
    assert context.launch_configurations["world_file"] == "empty.sdf"


@pytest.mark.parametrize("argument", ["spawn_z", "fixed_base"])
def test_simulation_launch_exposes_gazebo_model_parameters(argument):
    """web 上位机「模型」抽屉调的这两个参数要能穿过 simulation.launch.py。

    默认值必须与 gazebo.launch.py 一致——两处不一致时，上位机「不设置」就悄悄
    变成了「改行为」（发出的是 simulation 的默认，被控对象却按另一个默认起）。
    """
    _sim_entities, sim_context = _execute(LAUNCH_DIR / "simulation.launch.py")
    _gazebo_entities, gazebo_context = _execute(LAUNCH_DIR / "gazebo.launch.py")

    assert argument in sim_context.launch_configurations, (
        f"simulation.launch.py 没有声明 {argument}：web 上位机的模型抽屉传不进去")
    assert (sim_context.launch_configurations[argument]
            == gazebo_context.launch_configurations[argument]), (
        f"{argument} 在两处的默认值不一致")
