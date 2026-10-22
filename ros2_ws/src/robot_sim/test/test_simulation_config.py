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
    assert spec is not None and spec.loader is not None
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

    它是型号属性（厂家 2026-10-21：R58/R48 同为 300.0），写成一个常数会让别的
    量程的前馈被按比例缩放；而 τ_ff 平时恒为 0，接错在整个工作区里都不报错。
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

    assert profiles["cerb_r48"]["mit_torque_limit_nm"] == pytest.approx(300.0)
    assert profiles["cerb_r58"]["mit_torque_limit_nm"] == pytest.approx(300.0)


def test_cerb_mit_torque_limit_defaults_to_manufacturer_value(tmp_path):
    """CERB 缺 `mit_torque_limit_nm` 时必须回落到 300，不能回落物理峰值 45/11。

    型号库或拓扑漏写这个字段不会报错；若回落 `peak_torque_nm`，固件按 300 编码的
    τ_ff 会被解码端缩成 45/300（历史上正是这个错误）。vaxor 没有 MIT 量程，仍为 None。
    """
    module = _launch_module()
    motor_driver = MODEL_DIR / "config" / "motor_driver.yaml"
    specs = module._resolve_motor_specs_path(motor_driver)
    document = yaml.safe_load(motor_driver.read_text(encoding="utf-8"))
    for motor in document["motor_hardware"]["motors"].values():
        parameters = motor.setdefault("parameters", {})
        if str(motor.get("type", "")).startswith("cerb"):
            parameters.pop("mit_torque_limit_nm", None)
    missing = tmp_path / "motor_driver.yaml"
    missing.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    simulation = module._load_simulation(
        MODEL_DIR / "config" / "simulation_topology.yaml")
    bound = module._bind_motor_topology(simulation, missing, specs)

    cerb = [motor for motor in bound["motors"] if motor["protocol"] == "cerb"]
    vaxor = [motor for motor in bound["motors"] if motor["protocol"] == "vaxor"]
    assert len(cerb) == 14 and len(vaxor) == 4
    assert {motor["mit_torque_limit_nm"] for motor in cerb} == {300.0}
    assert {motor["mit_torque_limit_nm"] for motor in vaxor} == {None}


@pytest.mark.parametrize(
    "invalid",
    [-300.0, 0.0, float("nan"), float("inf"), "not-a-number"],
    ids=["negative", "zero", "nan", "inf", "non-numeric"],
)
def test_cerb_explicit_invalid_mit_torque_limit_fails_fast(tmp_path, invalid):
    """CERB 显式写坏的 mit_torque_limit_nm 必须 fail-fast，不能静默丢弃。

    旧实现把无效值从 profile 过滤掉，却把原值写进 motors：C++ 模拟器拿到 YAML 里的
    null/坏值，Python 后端拿到 -300/0/nan/inf 去解码，同一份拓扑两个后端分歧。显式
    给出该参数即表示配置错误（只有缺省才回落 300），错误须点名参数与电机。
    """
    module = _launch_module()
    motor_driver = MODEL_DIR / "config" / "motor_driver.yaml"
    specs = module._resolve_motor_specs_path(motor_driver)
    document = yaml.safe_load(motor_driver.read_text(encoding="utf-8"))
    document["motor_hardware"]["motors"]["arm_joint1"].setdefault("parameters", {})[
        "mit_torque_limit_nm"] = invalid
    broken = tmp_path / "motor_driver.yaml"
    broken.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    simulation = module._load_simulation(
        MODEL_DIR / "config" / "simulation_topology.yaml")
    with pytest.raises(ValueError, match=r"arm_joint1.*mit_torque_limit_nm"):
        module._bind_motor_topology(simulation, broken, specs)


def test_firmware_simulator_gets_mit_quantization_and_peak_clamp_separately():
    """MIT 解码量程与物理饱和必须走两条 CLI，不能共用一份数值。

    固件按型号库 `mit_torque_limit_nm`（CERB 300 N·m）编码 τ_ff，而物理出力上限是
    `peak_torque_nm`（R58 45 / R48 11）。把后者传给 `--mit-torque-limits` 会让前馈
    被缩放 45/300；反过来把 300 当饱和值则会放开真机峰值。vaxor 没有 MIT 量程，
    也不能出现在 `--mit-torque-limits` 里。
    """
    module = _launch_module()
    simulation, _ = _bound_simulation(module)

    # ID 1..18：CERB 为除 8/9/17/18 外的 14 台，量程统一 300；
    # 物理峰值按型号逐台 45/11。
    assert module._mit_torque_limit_cli(simulation) == (
        "1:300,2:300,3:300,4:300,5:300,6:300,7:300,"
        "10:300,11:300,12:300,13:300,14:300,15:300,16:300"
    )
    assert module._tau_limit_cli(simulation) == (
        "1:11,2:11,3:45,4:45,5:11,6:11,7:11,"
        "10:11,11:11,12:45,13:45,14:11,15:11,16:11"
    )


@pytest.mark.parametrize("field,expected", [
    ("mit_position_limit_rad", 3.14), ("mit_velocity_limit_rad_s", 3.14),
    ("mit_kp_limit", 2500.0), ("mit_kd_limit", 500.0),
])
def test_all_mit_scales_reach_both_simulator_backends(tmp_path, field, expected):
    module = _launch_module()
    simulation, specs = _bound_simulation(module)
    cerb = [m for m in simulation["motors"] if m["protocol"] == "cerb"]
    assert all(m[field] == expected for m in cerb)
    module._write_runtime_configs(
        simulation, tmp_path, "vcan", "unused", "unused", specs)
    profiles = yaml.safe_load((tmp_path / "motor_sim.yaml").read_text())[
        "motor_simulation"]["profiles"]
    assert profiles["cerb_r48"][field] == expected
    assert profiles["cerb_r58"][field] == expected
    assert module._mit_limit_cli(simulation, field).startswith(f"1:{expected:g},")


@pytest.mark.parametrize("field,value", [
    ("mit_position_limit_rad", 6.28), ("mit_velocity_limit_rad_s", 8.0),
    ("mit_torque_limit_nm", 150.0), ("mit_kp_limit", 1200.0),
    ("mit_kd_limit", 200.0),
])
def test_per_node_scale_override_does_not_overwrite_shared_profile(tmp_path, field, value):
    module = _launch_module()
    motor_driver = MODEL_DIR / "config" / "motor_driver.yaml"
    specs = module._resolve_motor_specs_path(motor_driver)
    document = yaml.safe_load(motor_driver.read_text())
    document["motor_hardware"]["motors"]["arm_joint1"].setdefault("parameters", {})[field] = value
    custom = tmp_path / "motor_driver.yaml"
    custom.write_text(yaml.safe_dump(document))
    simulation = module._load_simulation(MODEL_DIR / "config" / "simulation_topology.yaml")
    bound = module._bind_motor_topology(simulation, custom, specs)
    first, second = bound["motors"][:2]
    assert first[field] == value
    assert bound["profiles"][first["profile"]][field] == value
    assert bound["profiles"][second["profile"]][field] != value


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
    # 厂家 2026-10-21：R48 Kt 0.1 × 101 × 50%、R58 Kt 0.067 × 101 × 75%。
    assert by_profile["cerb_r48"] == {(5.05, 101.0)}
    assert by_profile["cerb_r58"] == {(5.0753, 101.0)}
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
