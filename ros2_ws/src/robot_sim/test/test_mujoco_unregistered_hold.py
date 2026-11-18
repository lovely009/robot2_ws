"""未注册输入轴物理固定：启动全固定、注册释放、撤销固定当前位置、整组缺失固定输出。

只跑真实 MuJoCo（无 ROS 图、无 systemd）。用完整 full_body MJCF 覆盖差动组。
"""
from pathlib import Path
from types import SimpleNamespace

import pytest

mujoco = pytest.importorskip("mujoco")


def hold_module(monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "scripts"))
    import mujoco_unregistered_hold
    return mujoco_unregistered_hold


MODEL = Path(__file__).parents[2] / (
    "robot_description/models/full_body_robot/mujoco/full_body_robot.xml")
WAIST_INPUTS = ["waist_flange_plus_x", "waist_flange_minus_x"]
HIPS_INPUTS = ["hips_flange_plus_x", "hips_flange_minus_x"]
GROUPS = [
    {"name": "waist", "inputs": WAIST_INPUTS,
     "outputs": ["waist_pitch", "waist_roll"]},
    {"name": "hips", "inputs": HIPS_INPUTS,
     "outputs": ["pelvis_roll", "pelvis_pitch"]},
]


def _make(monkeypatch):
    module = hold_module(monkeypatch)
    spec = mujoco.MjSpec.from_file(str(MODEL))
    hold = module.UnregisteredInputHold(spec, WAIST_INPUTS + HIPS_INPUTS, GROUPS)
    data = mujoco.MjData(hold.model)
    hold.attach(data)
    names = {hold.model.joint(hold.model.actuator_trnid[a, 0]).name: a
             for a in range(hold.model.nu)}
    return module, hold, hold.model, data, names


def _drive(module, model, data, hold, steps=3000):
    for _ in range(steps):
        mujoco.mj_step(model, data)
        hold.project()


def _qpos(model, data, name):
    return float(data.qpos[model.joint(name).qposadr[0]])


def test_start_all_inputs_fixed_gains_disabled(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    hold.sync(set())
    for name in WAIST_INPUTS + HIPS_INPUTS:
        assert model.actuator_gainprm[names[name], 0] == 0.0
        assert data.eq_active[hold._equality_id[name]] == 1


def test_registered_direct_input_moves(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    hold.sync({"waist_flange_plus_x"})
    data.ctrl[:] = 0.0
    data.ctrl[names["waist_flange_plus_x"]] = 0.3
    _drive(module, model, data, hold)
    assert _qpos(model, data, "waist_flange_plus_x") == pytest.approx(0.3, abs=5e-3)
    assert _qpos(model, data, "waist_flange_minus_x") == pytest.approx(0.0, abs=5e-3)


def test_partial_group_registered_drives_output_via_linkage(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    hold.sync({"waist_flange_plus_x"})
    data.ctrl[:] = 0.0
    data.ctrl[names["waist_flange_plus_x"]] = 0.3
    _drive(module, model, data, hold)
    assert abs(_qpos(model, data, "waist_pitch")) > 1e-3
    assert abs(_qpos(model, data, "waist_roll")) > 1e-3


def test_whole_group_unregistered_outputs_fixed_under_other_arm_motion(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    hold.sync({"left_shoulder_pitch"})
    data.ctrl[:] = 0.0
    before = {n: _qpos(model, data, n) for n in
              ("waist_pitch", "waist_roll", "pelvis_roll", "pelvis_pitch")}
    data.ctrl[names["left_shoulder_pitch"]] = 2.0
    _drive(module, model, data, hold)
    for name, value in before.items():
        assert _qpos(model, data, name) == pytest.approx(value, abs=1e-3), name


def test_unregister_holds_current_position(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    hold.sync({"waist_flange_plus_x"})
    data.ctrl[:] = 0.0
    data.ctrl[names["waist_flange_plus_x"]] = 0.3
    _drive(module, model, data, hold)
    reached = _qpos(model, data, "waist_flange_plus_x")
    hold.sync(set())
    assert model.actuator_gainprm[names["waist_flange_plus_x"], 0] == 0.0
    data.ctrl[:] = 0.0
    _drive(module, model, data, hold, steps=2000)
    assert _qpos(model, data, "waist_flange_plus_x") == pytest.approx(reached, abs=2e-3)


def test_fallback_without_groups_fixes_each_input_directly(monkeypatch):
    module = hold_module(monkeypatch)
    spec = mujoco.MjSpec.from_file(str(MODEL))
    hold = module.UnregisteredInputHold(spec, ["left_shoulder_pitch"], [])
    data = mujoco.MjData(hold.model)
    hold.attach(data)
    hold.sync(set())
    names = {hold.model.joint(hold.model.actuator_trnid[a, 0]).name: a
             for a in range(hold.model.nu)}
    assert hold.model.actuator_gainprm[names["left_shoulder_pitch"], 0] == 0.0
    hold.sync({"left_shoulder_pitch"})
    assert hold.model.actuator_gainprm[names["left_shoulder_pitch"], 0] != 0.0


class _Clock:
    def __init__(self, ns):
        self._ns = ns

    def now(self):
        return SimpleNamespace(nanoseconds=self._ns)


class _Stamp:
    def __init__(self, ns):
        self.sec = ns // 1_000_000_000
        self.nanosec = ns % 1_000_000_000


class _Msg:
    def __init__(self, names, stamp_ns):
        self.name = list(names)
        self.header = SimpleNamespace(stamp=_Stamp(stamp_ns))


def _bridge(monkeypatch, now_ns, known):
    pytest.importorskip("rclpy")
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "scripts"))
    import mujoco_effort_bridge
    b = mujoco_effort_bridge.MujocoEffortBridge.__new__(
        mujoco_effort_bridge.MujocoEffortBridge)
    b.actuator_joint_names = list(known)
    b.registry_timeout_s = 0.5
    b.registered_inputs = set()
    b.registry_stamp = None
    b._registry_last_stamp_ns = None
    b._registry_receipt_monotonic = None
    b.get_clock = lambda: _Clock(now_ns)
    b.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
    return b


def test_registry_rejects_duplicate_names(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a", "a"], now))
    assert b.registered_inputs == set()


def test_registry_rejects_unknown_names(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a", "zzz"], now))
    assert b.registered_inputs == set()


def test_registry_rejects_expired_stamp(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a"], now - 2_000_000_000))
    assert b.registered_inputs == set()


def test_registry_accepts_fresh_valid_names(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a"], now - 10_000_000))
    assert b.registered_inputs == {"a"}


SHOULDER_INPUTS = ["left_scapula_yaw", "left_shoulder_flange_linkage"]
SHOULDER_GROUPS = [{
    "name": "left_shoulder", "inputs": SHOULDER_INPUTS,
    "outputs": ["left_scapula_yaw", "left_scapula_roll"]}]


def test_shoulder_unregistered_direct_input_stays_fixed_when_group_partial(monkeypatch):
    # m6 的 left_scapula_yaw 既是输入又是组输出；只注册 m7(flange) 时组为部分，
    # 但 m6 未注册必须保持固定，不能被组输出释放逻辑放开。
    module = hold_module(monkeypatch)
    spec = mujoco.MjSpec.from_file(str(MODEL))
    hold = module.UnregisteredInputHold(spec, SHOULDER_INPUTS, SHOULDER_GROUPS)
    data = mujoco.MjData(hold.model)
    hold.attach(data)
    hold.sync({"left_shoulder_flange_linkage"})
    assert hold.is_active("left_scapula_yaw")
    assert not hold.is_active("left_shoulder_flange_linkage")


def test_registry_rejects_future_stamp(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a"], now + 1_000_000_000))  # 1s 未来
    assert b.registered_inputs == set()


def test_registry_rejects_replay(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a"], now - 10_000_000))
    assert b.registered_inputs == {"a"}
    b._on_registry(_Msg(["b"], now - 20_000_000))  # 更旧的回放
    assert b.registered_inputs == {"a"}


def test_receipt_watchdog_forces_all_fixed(monkeypatch):
    now = 10_000_000_000
    b = _bridge(monkeypatch, now, ["a", "b"])
    b._on_registry(_Msg(["a"], now - 10_000_000))
    assert b.registered_inputs == {"a"}

    class _Hold:
        def __init__(self):
            self.registered = None

        def sync(self, registered):
            self.registered = set(registered)

    b.hold = _Hold()
    import mujoco_effort_bridge as M
    future = b._registry_receipt_monotonic + b.registry_timeout_s + 1.0
    monkeypatch.setattr(M, "time", SimpleNamespace(monotonic=lambda: future))
    b._sync_registry_hold()
    assert b.hold.registered == set()


def test_unregistered_clears_gain_and_all_bias_terms_then_restores(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    a = names["waist_flange_plus_x"]
    original = (model.actuator_gainprm[a, 0], model.actuator_biasprm[a, 0],
                model.actuator_biasprm[a, 1], model.actuator_biasprm[a, 2])
    hold.sync(set())
    assert model.actuator_gainprm[a, 0] == 0.0
    assert model.actuator_biasprm[a, 0] == 0.0
    assert model.actuator_biasprm[a, 1] == 0.0
    assert model.actuator_biasprm[a, 2] == 0.0
    hold.sync({"waist_flange_plus_x"})
    restored = (model.actuator_gainprm[a, 0], model.actuator_biasprm[a, 0],
                model.actuator_biasprm[a, 1], model.actuator_biasprm[a, 2])
    assert restored == pytest.approx(original)


def test_project_zeroes_fixed_joint_velocity(monkeypatch):
    module, hold, model, data, names = _make(monkeypatch)
    hold.sync(set())
    dof = model.joint("waist_flange_plus_x").dofadr[0]
    data.qvel[dof] = 5.0
    hold.project()
    assert data.qvel[dof] == 0.0
