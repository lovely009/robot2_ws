"""腕部闭环求解器：纯运动学部分的自检。不启 ROS、不碰硬件、不启 Gazebo。

判据不采信求解器自报的任何指标，一律把解出的被动角代回**真实树 FK** 端到端验证：

1. 零位就是装配位形 ⇒ 解出的被动角应全为 0；
2. 连杆末端的球心（连杆系里的 crank_pin_offset）必须与**曲柄销球心**（固接在曲柄上、绕电机
   输出轴公转）重合到浮点噪声量级 —— 这就是闭环本身；
3. 两台电机必须**刚性固定在小臂上**：任意关节角下 Joint_link10/_0 在 Joint_link4 系里的
   位姿恒定（这是 2026-10-10 重构的核心主张，此前电机被挂在连杆末端上会乱飞）；
4. 无解或超限只夹紧、不放大；球铰分角必须回绕进 [-π, π]。

可达行程：连杆 A 分支把能闭上的范围限在 |joint_link5|, |joint_link6| ≲ 0.85 rad（≈49°），
见 config/wrist_closure.yaml 的说明。**URDF 里 joint_link5/6 的 ±1.5708 是占位限位，
超出了机构能闭上的范围**，所以下面的行程参数取到 0.8 为止。
"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

HERE = Path(__file__).parent
PACKAGE_DIR = HERE.parent
MODEL = PACKAGE_DIR / "models" / "zhoubuguanjie_youbi_001"
BALL_LIMIT = 3.1416
CRANK_LIMIT = 3.1416
# 两支独立算出的曲柄半径应当几乎相同（轴取对了的旁证；2026-10-10 标定为 22.18 / 21.93 mm）
CRANK_RADIUS_MM = 22.05
CRANK_RADIUS_TOL_MM = 1.0
# 可达行程：±0.8 rad 内 225 个位形全可解；±0.9 rad 起分支 A 开始出现无解位形
REACHABLE_RAD = 0.8
UNREACHABLE_RAD = 1.4


def _load_solver():
    path = PACKAGE_DIR / "scripts" / "wrist_closure_solver.py"
    spec = importlib.util.spec_from_file_location("wrist_closure_solver", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


solver = _load_solver()


@pytest.fixture(scope="module")
def rig():
    """(chain, [Branch, ...], {rod_link: 曲柄销球心在曲柄销连杆局部系里的位置})"""
    chain = solver.Chain((MODEL / "urdf" / f"{MODEL.name}.urdf").read_text(encoding="utf-8"))
    config = yaml.safe_load((MODEL / "config" / "wrist_closure.yaml").read_text(encoding="utf-8"))
    branches = [solver.Branch(chain, dict(config[key])) for key in ("branch_a", "branch_b")]
    zero = chain.world_poses({})
    pin_in_link9 = {}
    for branch in branches:
        pin_world0 = (zero[branch.rod_link] @ np.append(branch.crank_pin_offset, 1.0))[:3]
        pin_link = chain.joints[branch.cfg["crank_joint"]]["child"]
        T_pin = zero[pin_link]
        pin_in_link9[branch.rod_link] = T_pin[:3, :3].T @ (pin_world0 - T_pin[:3, 3])
    assert len(branches) == 2
    return chain, branches, pin_in_link9


def test_zero_pose_solves_to_the_assembled_configuration(rig):
    """零位就是装配位形：被动角应全为 0，否则配置与 URDF 的装配姿态不自洽。"""
    chain, branches, _ = rig
    poses = chain.world_poses({})
    for branch in branches:
        joints, diag = branch.solve(poses, BALL_LIMIT, CRANK_LIMIT)
        assert diag["feasible"], f"{branch.rod_link} 零位不可行：{diag}"
        for name, angle in joints.items():
            assert abs(angle) < np.radians(0.2), (
                f"{branch.rod_link}.{name} 零位应为 0，实为 {np.degrees(angle):.3f}°")


@pytest.mark.parametrize("half_range", [0.3, 0.6, REACHABLE_RAD])
def test_rod_end_lands_on_the_crank_circle(rig, half_range):
    """解出的角代回 FK 后，连杆末端球心必须与曲柄销球心重合（闭环端到端）。"""
    chain, branches, pin_in_link9 = rig
    fresh = [solver.Branch(chain, dict(b.cfg)) for b in branches]  # 每档重置连续性状态
    worst = 0.0
    for wrist_5 in np.linspace(-half_range, half_range, 9):
        for wrist_6 in np.linspace(-half_range, half_range, 9):
            actives = {"joint_link5": float(wrist_5), "joint_link6": float(wrist_6)}
            poses = chain.world_poses(actives)
            solved = {}
            for branch in fresh:
                joints, diag = branch.solve(poses, BALL_LIMIT, CRANK_LIMIT)
                assert diag["feasible"], f"{branch.rod_link} 在 {actives} 不可行：{diag}"
                solved.update(joints)
            poses_after = chain.world_poses({**actives, **solved})
            for branch in fresh:
                rod_end = (poses_after[branch.rod_link]
                           @ np.append(branch.crank_pin_offset, 1.0))[:3]
                pin_link = chain.joints[branch.cfg["crank_joint"]]["child"]
                crank_pin = (poses_after[pin_link]
                             @ np.append(pin_in_link9[branch.rod_link], 1.0))[:3]
                worst = max(worst, float(np.linalg.norm(rod_end - crank_pin)))
    assert worst < 1e-9, f"曲柄销球心偏差 {worst * 1e6:.3f} µm"


def test_motors_are_rigidly_fixed_to_the_forearm(rig):
    """两台 X25S 必须钉死在小臂上：任意关节角下电机在 Joint_link4 系里的位姿恒定。

    这是物理事实（电机安装面贴死小臂），也是本轮重构相对旧树的核心修正 ——
    旧树把电机挂在连杆末端，拖腕部关节时电机会被甩出去。
    """
    chain, _, _ = rig
    reference = None
    worst = 0.0
    for a5 in (-0.8, -0.4, 0.0, 0.4, 0.8):
        for a6 in (-0.8, 0.0, 0.8):
            for crank in (-3.0, -1.0, 0.0, 1.0, 3.0):
                angles = {
                    "joint_link5": a5, "joint_link6": a6,
                    "joint_crank_a": crank, "joint_crank_b": -crank,
                    "joint_wrist_ball_a": a5, "joint_link7": a6,
                    "joint_wrist_ball_b": -a5, "joint_link7_0": -a6,
                }
                poses = chain.world_poses(angles)
                T4 = poses["Joint_link4"]
                current = {}
                for motor in ("Joint_link10", "Joint_link10_0"):
                    current[motor] = (
                        T4[:3, :3].T @ (poses[motor][:3, 3] - T4[:3, 3]),
                        T4[:3, :3].T @ poses[motor][:3, :3],
                    )
                if reference is None:
                    reference = current
                    continue
                for motor in current:
                    worst = max(worst, float(np.abs(
                        current[motor][0] - reference[motor][0]).max()))
                    worst = max(worst, float(np.abs(
                        current[motor][1] - reference[motor][1]).max()))
    assert worst < 1e-12, f"电机相对小臂的位姿波动 {worst:.3e}（应恒为 0）"


def test_both_branches_agree_on_the_crank_radius(rig):
    """两分支各自的曲柄半径必须接近：曲柄半径 = 销心到电机输出轴的垂距，
    两条互不相干的机构算出同一个数，是「摇臂轴 = 电机输出轴」的旁证。"""
    _, branches, _ = rig
    radii = [branch.radius * 1000.0 for branch in branches]
    for radius in radii:
        assert radius == pytest.approx(CRANK_RADIUS_MM, abs=CRANK_RADIUS_TOL_MM), (
            f"曲柄半径 {radius:.2f} mm 偏离标定值 {CRANK_RADIUS_MM} mm —— "
            f"检查 URDF 的 joint_crank_* 或配置里的 crank_pin_offset")
    assert abs(radii[0] - radii[1]) < 1.0, f"两分支曲柄半径不一致：{radii}"


def test_ball_angles_are_wrapped_into_pi(rig):
    """球铰分角必须回绕进 [-π, π]：不回绕会算出 +352° 这种值，被 <limit> 误判为超限，
    把本来精确可解的位形夹出位置残差（本文件早期版本的假故障）。"""
    chain, branches, _ = rig
    for pose in ({"joint_link5": 0.0, "joint_link6": -REACHABLE_RAD},
                 {"joint_link5": -REACHABLE_RAD, "joint_link6": REACHABLE_RAD}):
        poses = chain.world_poses(pose)
        for branch in branches:
            joints, _ = branch.solve(poses, BALL_LIMIT, CRANK_LIMIT)
            for name in (branch.cfg["ball_joint"], branch.cfg["rod_joint"]):
                assert -np.pi - 1e-9 <= joints[name] <= np.pi + 1e-9, (
                    f"{name} = {np.degrees(joints[name]):.2f}° 未回绕进 [-π, π]")


def test_unreachable_pose_is_clamped_not_amplified(rig):
    """销心位置被改到够不着时，必须报不可行并把角夹在限位内 —— 只降级，不放大概率。"""
    chain, branches, _ = rig
    poses = chain.world_poses({"joint_link5": 0.5, "joint_link6": -0.5})
    branch = branches[0]
    impossible = solver.Branch(chain, {
        **branch.cfg, "crank_pin_offset": [1e-3, 0.0, 0.0]})
    joints, diag = impossible.solve(poses, BALL_LIMIT, CRANK_LIMIT)
    assert not diag["feasible"]
    assert abs(joints[impossible.cfg["crank_joint"]]) <= CRANK_LIMIT + 1e-9
    for name in (impossible.cfg["ball_joint"], impossible.cfg["rod_joint"]):
        assert abs(joints[name]) <= BALL_LIMIT + 1e-9, f"{name} 超限"


def test_reachable_wrist_range_is_bounded_and_reported(rig):
    """行程超出可达域时必须**如实报不可行**，而不是外推出一个假的闭合位形。

    连杆 A 分支在 ±0.8 rad 内 225 个位形全可解、±1.4 rad 时 24/225 无解（见 yaml 说明）。
    这条断言把「URDF 的 ±1.5708 占位限位超出了机构可达范围」固化下来 —— 若哪天参数被调到
    能在全行程闭合，这里会变红，提醒同步更新 README 与 yaml 的行程说明。
    """
    chain, branches, _ = rig
    branch_a = branches[0]
    feasible = infeasible = 0
    for wrist_5 in np.linspace(-UNREACHABLE_RAD, UNREACHABLE_RAD, 15):
        for wrist_6 in np.linspace(-UNREACHABLE_RAD, UNREACHABLE_RAD, 15):
            poses = chain.world_poses(
                {"joint_link5": float(wrist_5), "joint_link6": float(wrist_6)})
            probe = solver.Branch(chain, dict(branch_a.cfg))
            _, diag = probe.solve(poses, BALL_LIMIT, CRANK_LIMIT)
            if diag["feasible"]:
                feasible += 1
                assert diag["position_error_m"] < 1e-9, (
                    "报了可行就必须真的闭合，位置残差 "
                    f"{diag['position_error_m'] * 1e6:.3f} µm")
            else:
                infeasible += 1
    assert infeasible > 0, (
        f"±{UNREACHABLE_RAD} rad 内已无不可解位形（{feasible} 个全可解）—— "
        "可达行程变宽了，请同步更新 yaml 与 README 的行程说明")
    assert feasible > 0, "可达域变得过窄，检查 crank_pin_offset"


def test_rod_length_follows_from_the_measured_pin_offset(rig):
    """L = |crank_pin_offset|：销心到**连杆原点**（球铰回转中心）的距离。"""
    _, branches, _ = rig
    lengths = {branch.rod_link: branch.rod_length for branch in branches}
    assert lengths["Joint_link7"] == pytest.approx(0.1021, abs=3e-3)
    assert lengths["Joint_link7_0"] == pytest.approx(0.0697, abs=3e-3)
