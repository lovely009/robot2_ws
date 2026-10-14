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
    module._write_runtime_configs(simulation, tmp_path, "vcan", "robot_vcan0", specs)
    profiles = yaml.safe_load(
        (tmp_path / "motor_sim.yaml").read_text(encoding="utf-8")
    )["motor_simulation"]["profiles"]

    assert profiles["cerb_r48"]["mit_torque_limit_nm"] == pytest.approx(11.0)
    assert profiles["cerb_r58"]["mit_torque_limit_nm"] == pytest.approx(45.0)
