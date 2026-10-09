"""腕部闭环求解器：纯运动学部分的自检。不启 ROS、不碰硬件、不启 Gazebo。

判据（方案 α 的核心断言）：给定 config/wrist_closure.yaml 里的摇臂轴与两个实测球心，
求解器解出的被动关节角必须让 Joint_link9/10 的「球心位置 + 朝向」精确等于
「摇臂绕轴刚性转过 φ」的目标 —— 用真实树 FK 端到端验证，不采信求解器自报的任何指标。
这样一旦轴参数或球心被改错，闭环就会散开，测试立刻变红。
"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml

HERE = Path(__file__).parent
PACKAGE_DIR = HERE.parent
MODEL = PACKAGE_DIR / "models" / "zhoubuguanjie_youbi_001"
BALL_LIMIT = 1.0


def _load_solver():
    path = PACKAGE_DIR / "scripts" / "wrist_closure_solver.py"
    spec = importlib.util.spec_from_file_location("wrist_closure_solver", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


solver = _load_solver()


@pytest.fixture(scope="module")
def rig():
    """(chain, [(cfg, L, link9, 球心在 link9 局部系), ...])"""
    chain = solver.Chain((MODEL / "urdf" / f"{MODEL.name}.urdf").read_text(encoding="utf-8"))
    config = yaml.safe_load((MODEL / "config" / "wrist_closure.yaml").read_text(encoding="utf-8"))
    branches = []
    for key, link9 in (("branch_a", "Joint_link9"), ("branch_b", "Joint_link9_0")):
        cfg = dict(config[key])
        cfg["pin_origin_rpy"] = chain.joints[cfg["pin_joint"]]["origin_rpy"]
        for field in ("wrist_ball_offset", "crank_pin_offset",
                      "crank_axis_direction", "crank_axis_point"):
            cfg[field] = np.array(cfg[field])
        length = float(np.linalg.norm(cfg["crank_pin_offset"] - cfg["wrist_ball_offset"]))
        pin_origin = chain.joints[cfg["pin_joint"]]["T"]
        ball_in_link9 = pin_origin[:3, :3].T @ (cfg["crank_pin_offset"] - pin_origin[:3, 3])
        branches.append((cfg, length, link9, ball_in_link9))
    assert len(branches) == 2
    return chain, branches


def _target(cfg, poses, phi):
    """摇臂刚性转过 φ 后的目标：球心位置 + 末端朝向。"""
    rod = poses[cfg["rod_link"]]
    link4 = poses["Joint_link4"]
    axis = link4[:3, :3] @ cfg["crank_axis_direction"]
    pin_zero = (rod @ np.append(cfg["crank_pin_offset"], 1.0))[:3]
    on_axis = (link4 @ np.append(cfg["crank_axis_point"], 1.0))[:3]
    offset = pin_zero - on_axis
    centre = on_axis + float(offset @ axis) * axis
    radius_vector = pin_zero - centre
    radius = np.linalg.norm(radius_vector)
    u_hat = radius_vector / radius
    w_hat = np.cross(axis, u_hat)
    position = centre + radius * (u_hat * np.cos(phi) + w_hat * np.sin(phi))
    rotation = solver.rodrigues(axis, phi) @ (rod[:3, :3] @ solver.rpy_to_R(*cfg["pin_origin_rpy"]))
    return position, rotation


def test_zero_pose_solves_to_the_assembled_configuration(rig):
    """零位就是装配位形：被动角应全为 0，否则配置与 URDF 的装配姿态不自洽。"""
    chain, branches = rig
    poses = chain.world_poses({})
    for cfg, length, _, _ in branches:
        joints, diag = solver.solve_branch(chain, poses, cfg, length, None, BALL_LIMIT)
        assert diag["feasible"], f"{cfg['rod_link']} 零位不可行：{diag}"
        for name, angle in joints.items():
            assert abs(angle) < np.radians(0.2), f"{cfg['rod_link']}.{name} 零位应为 0，实为 {np.degrees(angle):.3f}°"


@pytest.mark.parametrize("half_range", [0.3, 0.8, 1.4])
def test_ball_center_and_orientation_follow_the_crank(rig, half_range):
    """解出的被动角代回树 FK 后，球心与朝向必须等于摇臂刚性转动的目标（端到端）。"""
    chain, branches = rig
    worst_position = 0.0
    worst_orientation = 0.0
    for wrist_5 in np.linspace(-half_range, half_range, 9):
        for wrist_6 in np.linspace(-half_range, half_range, 9):
            actives = {"joint_link5": float(wrist_5), "joint_link6": float(wrist_6)}
            poses = chain.world_poses(actives)
            solved, targets = {}, {}
            for cfg, length, _, _ in branches:
                joints, diag = solver.solve_branch(chain, poses, cfg, length, None, BALL_LIMIT)
                assert diag["feasible"], f"{cfg['rod_link']} 在 {actives} 不可行：{diag}"
                solved.update(joints)
                targets[cfg["rod_link"]] = _target(cfg, poses, diag["phi"])
            poses_after = chain.world_poses({**actives, **solved})
            for cfg, _, link9, ball_in_link9 in branches:
                position, rotation = targets[cfg["rod_link"]]
                achieved = poses_after[link9]
                achieved_position = (achieved @ np.append(ball_in_link9, 1.0))[:3]
                worst_position = max(worst_position, float(np.linalg.norm(achieved_position - position)))
                cosine = (np.trace(rotation.T @ achieved[:3, :3]) - 1.0) / 2.0
                worst_orientation = max(worst_orientation, float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
    # 位置是闭式精确解，朝向由销轴补偿角闭合；两者都应在浮点噪声量级
    assert worst_position < 1e-9, f"球心位置偏差 {worst_position * 1e6:.3f} µm"
    assert worst_orientation < 1e-3, f"朝向偏差 {worst_orientation:.6f}°"


def test_unreachable_pose_is_clamped_not_amplified(rig):
    """杆长被改到够不着时，必须报不可行并把角夹在限位内 —— 只降级，不放大概率。"""
    chain, branches = rig
    poses = chain.world_poses({"joint_link5": 0.9, "joint_link6": -0.9})
    cfg, _, _, _ = branches[0]
    joints, diag = solver.solve_branch(chain, poses, cfg, 0.5, None, BALL_LIMIT)
    assert not diag["feasible"]
    for name, angle in joints.items():
        assert abs(angle) <= BALL_LIMIT + 1e-9, f"{name} 超限：{np.degrees(angle):.2f}°"


def test_rod_length_follows_from_the_measured_ball_centers(rig):
    """L 由两个实测球心推出（而不是沿用 URDF 的 joint_link9 原点），并落在实测区间内。"""
    _, branches = rig
    lengths = {cfg["rod_link"]: length for cfg, length, _, _ in branches}
    assert lengths["Joint_link7"] == pytest.approx(0.1036, abs=3e-3)
    assert lengths["Joint_link7_0"] == pytest.approx(0.0658, abs=3e-3)
