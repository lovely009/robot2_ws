"""静态检查 simulation.launch.py 生成的运行期配置，不启动任何进程。

只覆盖「值接错了也看不出来」的那类字段：它们在默认工况下要么恒为某个值
（τ_ff 平时恒为 0），要么被下游默默用错，构建、启动与既有测试都不会报错。
"""

import importlib.util
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).parent
LAUNCH_FILE = HERE.parent / "launch" / "simulation.launch.py"
MODEL_DIR = HERE.parent / "models" / "half_body_robot"


def _launch_module():
    spec = importlib.util.spec_from_file_location(LAUNCH_FILE.stem, LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _bound_simulation(module):
    motor_driver = MODEL_DIR / "config" / "motor_driver.yaml"
    specs = module._resolve_motor_specs_path(motor_driver)
    simulation = module._load_simulation(
        MODEL_DIR / "config" / "simulation_topology.yaml")
    return module._bind_motor_topology(simulation, motor_driver, specs), specs


def test_motor_sim_config_carries_per_model_torque_quantization(tmp_path):
    """MIT τ_ff 的 12 位量化满量程必须逐型号落到 motor_sim 的配置里。

    它是型号属性（R48 11.0 / R58 45.0），写成一个常数会让 R58 的前馈被按 11/45
    缩放、R48 放大 4.09 倍；而 τ_ff 平时恒为 0，接错在整个工作区里都不报错。
    """
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    # vcan 场景下驱动侧与模拟器侧共用同一个接口。
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "driver_iface", "sim_iface", specs
    )
    profiles = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]["profiles"]

    assert profiles["cerb_r48"]["mit_torque_limit_nm"] == pytest.approx(11.0)
    assert profiles["cerb_r58"]["mit_torque_limit_nm"] == pytest.approx(45.0)


def test_motor_sim_config_carries_bus_transport_and_endpoint(tmp_path):
    """buses[] 必须带上传输类型与端点——节点已不再收任何总线类参数。

    这是配置单源的落点：写成旧形状（只有总线名）会让节点拿不到端点，
    而节拍与 SDO 初值另有专门的用例（见本文件后两条）。
    """
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "robot_vcan0", "robot_vcan0", specs
    )
    document = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]

    buses = document["buses"]
    assert len(buses) == 1
    assert buses[0]["name"] == "arm"
    assert buses[0]["transport"] == "socket_can"
    assert buses[0]["endpoint"] == "robot_vcan0"
    assert buses[0]["baud_rate"] == 3000000
    assert buses[0]["allow_external_canfd"] is False


def test_motor_topology_carries_feedback_conversion_factors():
    """反馈电流/转速的换算系数必须逐台落到模拟器的 --joint-nm-per-amp / --gear。

    HIL 下这两个系数是「假电机实际出了多少力」与「反馈报多少 A / RPM」之间唯一的
    联系（见 micro_ros_agent/README.md）。它们派生自型号库的
    torque_constant_nm_per_amp × gear_ratio × gear_efficiency，任何一项改名或漏取
    都不会报错——只会让页面上的电流整体缩放，或者干脆恒为 0（模拟器查不到就报 0，
    不猜系数）。vaxor 那台尤其容易漏：效率只在它这一档非 1.0。
    """
    module = _launch_module()
    simulation, _ = _bound_simulation(module)
    by_profile = {}
    for motor in simulation["motors"]:
        by_profile.setdefault(motor["profile"], set()).add(
            (round(motor["joint_nm_per_amp"], 4), motor["gear_ratio"])
        )
    # 每个 profile 内逐台同值（型号库是逐型号的事实，不是逐台的）
    assert by_profile["cerb_r48"] == {(7.777, 101.0)}
    assert by_profile["cerb_r58"] == {(8.686, 101.0)}
    assert by_profile["vaxor_2550"] == {(1.365, 50.0)}
    # 18 台全部带上，一台都不能缺（缺了那台反馈恒 0 A）
    assert len(simulation["motors"]) == 18
    assert all(motor["joint_nm_per_amp"] > 0.0 and motor["gear_ratio"] > 0.0
               for motor in simulation["motors"])


def test_motor_sim_config_endpoint_is_the_simulator_side_not_the_driver_side(tmp_path):
    """模拟器侧的端点必须取 sim_endpoint，不能串成 driver_endpoint。

    slcan/socketcan + controller_mode:=host 时两侧是两个不同设备，两个端点都存在
    且都是 tty ⇒ 所有校验都会通过。传反了不会有任何报错，只会让模拟器开驱动器那
    只口、驱动器开模拟器那只口。
    """
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "driver_iface", "sim_iface", specs
    )
    sim = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]
    driver = yaml.safe_load(
        (tmp_path / "motor_driver.yaml").read_text(encoding="utf-8")
    )["motor_hardware"]

    assert sim["buses"][0]["endpoint"] == "sim_iface"
    assert driver["buses"]["arm"]["interface"] == "driver_iface"


def test_motor_sim_config_timing_defaults_stay_on_legacy_cadence(tmp_path):
    """timing 新字段的缺省必须等于旧节拍，不能预置 Python 后端的数值。

    这几个字段的消费方是后续的反馈调度增量；今天写进 Python 的数值
    （20 ms / 2.2 ms 错峰 / 200 Hz），等于在配置缺省里把 host/external 场景
    提前换成 HIL 的节拍。0 = 沿用 control_hz、不错峰，才是旧行为。
    """
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "driver_iface", "sim_iface", specs
    )
    timing = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]["timing"]

    assert timing["can_period_ms"] == 0
    assert timing["feedback_period_ms"] == 0
    assert timing["stagger_ms"] == 0
    assert timing["effort_rate_hz"] == 0


def test_motor_sim_config_sdo_overrides_are_real_hardware_power_on_values(tmp_path):
    """sdo_overrides 本增量没有输入通路，写出的必须是真机初值本身。

    它不能被写成"从拓扑取"的样子：拓扑、motor_driver.yaml 与 launch 参数都不声明
    这两个键，`motor.get(...)` 恒等缺省值，却让生成的 YAML 看起来像有权威来源。
    通路随 SDO 从站增量一起接（ADR §6 的「保护被关掉」场景）。
    """
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "driver_iface", "sim_iface", specs
    )
    document = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]

    assert document["motors"]
    for motor in document["motors"]:
        assert motor["sdo_overrides"] == {"protection_enabled": 1, "direction": 0}


def test_motor_sim_config_carries_cerb_device_identity_and_sdo_defaults(tmp_path):
    """身份串与 SDO 出厂值必须逐型号落到 motor_sim.yaml。

    固件靠 0x1008 身份串绑型号、靠 0x6080/0x2538 回读核验。缺了它们台架表现为
    「全部电机停在未绑定」，而构建与既有测试都不报错。0x6080 是模块侧共用值，
    逐型号的是 0x2538。
    """
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "driver_iface", "sim_iface", specs
    )
    profiles = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]["profiles"]

    assert profiles["cerb_r58"]["device_identity"] == "R58LITE_V2"
    assert profiles["cerb_r58"]["device_max_speed_rpm"] == 3325
    assert profiles["cerb_r58"]["device_max_current_ma"] == 13300
    assert profiles["cerb_r48"]["device_identity"] == "R48LITE_V2"
    assert profiles["cerb_r48"]["device_max_current_ma"] == 5000
