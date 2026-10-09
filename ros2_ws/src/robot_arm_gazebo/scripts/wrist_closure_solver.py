#!/usr/bin/env python3
"""腕部连杆闭环求解器：把切开的闭环补回去，让被动关节角始终满足「二力杆 + 摇臂圆」。

背景见 models/zhoubuguanjie_youbi_001/README.md 的腕部章节与 config/wrist_closure.yaml。
URDF 是树，SW2URDF 把闭环切在「电机—小臂」这条边上，导出树为
    … → joint_link6 → joint_wrist_ball_* → joint_link7* → joint_link9* →（fixed）电机
本节点订阅上位发布的关节角（例如 RViz 的 joint_state_publisher_gui），按闭环约束解出
四个被动关节（每分支：球铰两轴 + 销轴补偿角），把修正后的完整关节状态再发出去。

每分支的解法（全部闭式，无迭代）：
  1. 腕端球心 P7 由主链 FK 给出（与被动角无关：球铰原点与 joint_link7 同点、连杆原点偏移为 0）；
  2. 摇臂轴在小臂系里是常量（定义见配置），摇臂端球心只能落在「以该轴为轴、半径 r 的圆」上，
     r 由零位位形下的球心到轴的垂距推出；
  3. 杆长守恒 |P9 − P7| = L ⇒ 解出摇臂转角 φ（闭式，两支取与前一刻连续的那支）；
  4. 把连杆「指向」目标 P9（球铰两轴，恰好 2 个自由度 ⇒ 位置可精确满足）；
  5. 销轴补偿角取使 Joint_link9/10 姿态最接近「随摇臂刚性转动」的那个角（1 个自由度，
     姿态残差是结构性的，见 README）。

安全：任何一步无解或超限都只降级（夹紧 + 告警），不放大输出；本节点不发任何写命令，
真机链路不经过它。
"""

import math
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# 纯运动学（不依赖 ROS，可直接单测）
# ---------------------------------------------------------------------------


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))


def origin_T(element):
    origin = element.find("origin")
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    if origin is not None:
        if origin.get("xyz"):
            xyz = np.array([float(v) for v in origin.get("xyz").split()])
        if origin.get("rpy"):
            rpy = np.array([float(v) for v in origin.get("rpy").split()])
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class Chain:
    """极简 URDF 运动学：按 joint 树做 FK，够用即可（模型只有 14 个关节）。"""

    def __init__(self, urdf_text):
        root = ET.fromstring(urdf_text)
        self.joints = {}
        self.children = {}
        self.links = [link.get("name") for link in root.findall("link")]
        for joint in root.findall("joint"):
            name = joint.get("name")
            axis = joint.find("axis")
            self.joints[name] = {
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "T": origin_T(joint),
                "origin_rpy": (np.array([float(v) for v in joint.find("origin").get("rpy").split()])
                               if joint.find("origin") is not None and joint.find("origin").get("rpy")
                               else np.zeros(3)),
                "axis": (np.array([float(v) for v in axis.get("xyz").split()])
                         if axis is not None else np.zeros(3)),
                "type": joint.get("type"),
                "limit": (float(joint.find("limit").get("lower")),
                          float(joint.find("limit").get("upper")))
                if joint.find("limit") is not None else None,
            }
            self.children.setdefault(self.joints[name]["parent"], []).append(name)
        child_links = {j["child"] for j in self.joints.values()}
        self.root = next(link for link in self.links if link not in child_links)

    def world_poses(self, angles):
        """返回 {link: T_world}；angles 里没有的关节取 0。"""
        poses = {}

        def walk(link, T):
            poses[link] = T
            for joint_name in self.children.get(link, []):
                joint = self.joints[joint_name]
                T_joint = T @ joint["T"]
                if joint["type"] in ("revolute", "continuous"):
                    T_joint = T_joint @ _axis_T(joint["axis"], angles.get(joint_name, 0.0))
                walk(joint["child"], T_joint)

        walk(self.root, np.eye(4))
        return poses


def _axis_T(axis, angle):
    T = np.eye(4)
    T[:3, :3] = rodrigues(axis, angle)
    return T


def solve_crank_angle(p7, axis_dir, axis_point, p9_zero, rod_length):
    """解摇臂转角 φ：使摇臂圆上的点与 P7 的距离等于杆长。

    返回 (psi, delta, 是否可行, (圆心 O, u_hat, w_hat, r))。圆心 O 是球心到轴的**垂足**，
    圆参数化为 P9(φ) = O + r(u_hat cosφ + w_hat sinφ)，u_hat 指向零位球心。
    不可行（球面与圆不相交）时 delta 取 0，即取最接近的点（夹紧而非放大）。
    """
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    d0 = p9_zero - axis_point
    height0 = float(d0 @ axis_dir)
    offset = d0 - height0 * axis_dir
    r = float(np.linalg.norm(offset))
    O = axis_point + height0 * axis_dir      # 圆心：球心到轴的垂足
    if r < 1e-9:
        # 球心落在轴上 ⇒ 摇臂半径为零，机构退化
        return 0.0, 0.0, False, (O, axis_dir, np.zeros(3), 0.0)
    u_hat = offset / r
    w_hat = np.cross(axis_dir, u_hat)
    d = p7 - axis_point
    height = float(d @ axis_dir)
    radial = d - height * axis_dir
    rho = float(np.linalg.norm(radial))
    if rho < 1e-12:
        return 0.0, 0.0, False, (O, u_hat, w_hat, r)
    # 注意：垂距要相对**圆平面**量（即减掉 height0），用轴上任一点的高度会把 base 算偏
    height_rel = height - height0
    # |O + r(u cosφ + w sinφ) − P7| = L 展开成 cos(φ − ψ) = base/rho
    base = (r * r + rho * rho + height_rel * height_rel - rod_length * rod_length) / (2.0 * r)
    feasible = abs(base / rho) <= 1.0
    ratio = min(1.0, max(-1.0, base / rho))
    psi = math.atan2(float(radial @ w_hat), float(radial @ u_hat))
    delta = math.acos(ratio)
    return psi, delta, feasible, (O, u_hat, w_hat, r)


def pick_crank_branch(psi, delta, previous):
    """两支 φ = ψ ± δ 里取与 previous 连续的那支。

    previous 为 None（第一帧）时取最接近 0 的那支 —— 模型是在零位装配下建的，
    零位对应 φ = 0；另一支是同一个圆的「另一侧」解，几何上同样成立但装配不同。
    """
    candidates = [psi - delta, psi + delta]
    reference = 0.0 if previous is None else previous
    return min(candidates, key=lambda phi: abs(_wrap(phi - reference)))


def aim_rod(d_local, d6, sign):
    """解球铰两轴 (α, β) 使连杆轴向 d_local 在 link6 系里指向 d6。

    T_rod = T_link6 · Rot((0,1,0), α) · Rot((0,0,1), β)（两个关节 origin 的 rpy 都为 0），
    故 Ry(α) Rz(β) d_local = d6。Ry 保 y、Rz 保 z，于是 m = Rz(β) d_local 满足
    m_y = d6_y、m_z = d_local_z，m_x 由单位长度定。

    m_x 的符号对应连杆绕自身轴的自转 —— 那是球铰的自由自由度，两支都精确满足位置约束，
    差别只在末端姿态，所以由 sign 指定、由调用方取姿态残差小的那支。
    返回 (α, β, 是否因几何不成立而夹紧)。
    """
    m_z = float(d_local[2])
    residual = 1.0 - float(d6[1]) ** 2 - m_z ** 2
    saturated = residual < 0.0
    m_x = sign * math.sqrt(max(residual, 0.0))
    m_y = float(d6[1])
    beta = math.atan2(m_y, m_x) - math.atan2(float(d_local[1]), float(d_local[0]))
    s = m_x * m_x + m_z * m_z
    if s < 1e-12:
        alpha = 0.0
    else:
        cos_a = (m_x * float(d6[0]) + m_z * float(d6[2])) / s
        sin_a = (m_z * float(d6[0]) - m_x * float(d6[2])) / s
        alpha = math.atan2(sin_a, cos_a)
    return alpha, beta, saturated


def best_pin_angle(rotation_without_pin, target_rotation):
    """求绕销轴（link 局部 z）的转角，使连杆末端姿态最接近目标（闭式最优）。

    maximize trace(Rz(θ)ᵀ M)，M = R_without_pinᵀ · R_target ⇒ θ = atan2(M10−M01, M00+M11)。
    """
    M = rotation_without_pin.T @ target_rotation
    angle = math.atan2(M[1, 0] - M[0, 1], M[0, 0] + M[1, 1])
    residual = rot_z(angle).T @ M
    cosine = min(1.0, max(-1.0, (float(np.trace(residual)) - 1.0) / 2.0))
    return angle, math.degrees(math.acos(cosine))


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def solve_branch(chain, poses, cfg, rod_length, previous_phi, ball_limit):
    """解一条腕部分支的被动关节角。返回 (dict 关节名→角, 诊断 dict)。"""
    rod = cfg["rod_link"]
    T_rod = poses[rod]
    T_link4 = poses["Joint_link4"]
    T_link6 = poses["Joint_link6"]

    d_local = (np.array(cfg["crank_pin_offset"]) - np.array(cfg["wrist_ball_offset"])) / rod_length
    p7 = T_rod @ np.append(cfg["wrist_ball_offset"], 1.0)
    p9_zero = T_rod @ np.append(cfg["crank_pin_offset"], 1.0)
    axis_dir = T_link4[:3, :3] @ np.array(cfg["crank_axis_direction"])
    axis_point = T_link4 @ np.append(cfg["crank_axis_point"], 1.0)

    psi, delta, feasible, (centre, u_hat, w_hat, r) = solve_crank_angle(
        p7[:3], axis_dir, axis_point[:3], p9_zero[:3], rod_length)
    phi = pick_crank_branch(psi, delta, previous_phi)

    # 目标：摇臂端球心落在圆上，连杆末端随摇臂刚性转动（绕轴 φ）
    p9_target = centre + r * (u_hat * math.cos(phi) + w_hat * math.sin(phi))
    target_rotation = rodrigues(axis_dir, phi) @ (T_rod[:3, :3] @ rpy_to_R(*cfg["pin_origin_rpy"]))

    # 先让连杆指向目标球心（球铰两轴，位置可精确满足）
    d6 = T_link6[:3, :3].T @ ((p9_target - p7[:3]) / rod_length)
    best = None
    for sign in (1.0, -1.0):
        alpha, beta, saturated = aim_rod(d_local, d6, sign)
        over_limit = abs(alpha) > ball_limit or abs(beta) > ball_limit
        alpha = max(-ball_limit, min(ball_limit, alpha))
        beta = max(-ball_limit, min(ball_limit, beta))
        rotation = T_link6[:3, :3] @ rot_y(alpha) @ rot_z(beta)
        pin_angle, pin_residual_deg = best_pin_angle(
            rotation @ rpy_to_R(*cfg["pin_origin_rpy"]), target_rotation)
        if best is None or pin_residual_deg < best[1]:
            best = ((alpha, beta, pin_angle), pin_residual_deg, saturated or over_limit)
    (alpha, beta, pin_angle), pin_residual_deg, degraded = best

    rotation = T_link6[:3, :3] @ rot_y(alpha) @ rot_z(beta)
    reached = rotation @ d_local * rod_length + p7[:3]
    position_error = float(np.linalg.norm(reached - p9_target))
    if position_error > 1e-6 or pin_residual_deg > 90.0:
        degraded = True
    return (
        {cfg["ball_joint"]: alpha, cfg["rod_joint"]: beta, cfg["pin_joint"]: pin_angle},
        {
            "phi": phi,
            "feasible": feasible and not degraded,
            "position_error_m": position_error,
            "orientation_residual_deg": pin_residual_deg,
            "radius_m": r,
        },
    )


# ---------------------------------------------------------------------------
# ROS 节点
# ---------------------------------------------------------------------------


class WristClosureSolver(Node):
    def __init__(self):
        super().__init__("wrist_closure_solver")
        model = self.declare_parameter("model", "zhoubuguanjie_youbi_001").value
        share = get_package_share_directory("robot_arm_gazebo")
        default_urdf = f"{share}/models/{model}/urdf/{model}.urdf"
        default_cfg = f"{share}/models/{model}/config/wrist_closure.yaml"
        urdf_path = self.declare_parameter("urdf_path", default_urdf).value
        config_path = self.declare_parameter("config_path", default_cfg).value
        input_topic = self.declare_parameter("input_topic", "/joint_states_raw").value
        output_topic = self.declare_parameter("output_topic", "/joint_states").value

        with open(urdf_path, encoding="utf-8") as handle:
            self._chain = Chain(handle.read())
        with open(config_path, encoding="utf-8") as handle:
            config = yaml.safe_load(handle)

        self._branches = []
        for key in ("branch_a", "branch_b"):
            cfg = dict(config[key])
            cfg["pin_origin_rpy"] = self._chain.joints[cfg["pin_joint"]]["origin_rpy"]
            cfg["wrist_ball_offset"] = np.array(cfg["wrist_ball_offset"])
            cfg["crank_pin_offset"] = np.array(cfg["crank_pin_offset"])
            cfg["crank_axis_direction"] = np.array(cfg["crank_axis_direction"])
            cfg["crank_axis_point"] = np.array(cfg["crank_axis_point"])
            rod_length = float(np.linalg.norm(cfg["crank_pin_offset"] - cfg["wrist_ball_offset"]))
            self._branches.append((cfg, rod_length))
        solver_cfg = config.get("solver", {})
        self._ball_limit = float(solver_cfg.get("ball_limit_rad", 1.0))
        self._warn_deg = float(solver_cfg.get("orientation_warn_deg", 5.0))
        self._phi = {cfg["rod_link"]: None for cfg, _ in self._branches}

        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self._on_joint_state, 10)
        self.get_logger().info(
            f"闭环求解器已启动：{input_topic} → {output_topic}；"
            f"摇臂轴参数来自 {config_path}（数值未标定，见文件头说明）")

    def _on_joint_state(self, message):
        angles = dict(zip(message.name, message.position))
        poses = self._chain.world_poses(angles)
        solved = dict(angles)
        for cfg, rod_length in self._branches:
            joints, diag = solve_branch(self._chain, poses, cfg, rod_length,
                                        self._phi[cfg["rod_link"]], self._ball_limit)
            self._phi[cfg["rod_link"]] = diag["phi"]
            solved.update(joints)
            level = self.get_logger().warn if (
                not diag["feasible"] or diag["orientation_residual_deg"] > self._warn_deg
            ) else self.get_logger().debug
            level(
                f"{cfg['rod_link']}: 摇臂角={diag['phi']:+.3f} rad r={diag['radius_m'] * 1000:.2f} mm "
                f"位置误差={diag['position_error_m'] * 1e6:.1f} µm "
                f"姿态残差={diag['orientation_residual_deg']:.2f}° "
                f"{'' if diag['feasible'] else '（闭环无解或超限，已夹紧）'}")

        out = JointState()
        out.header.stamp = message.header.stamp
        out.header.frame_id = message.header.frame_id
        out.name = sorted(solved)
        out.position = [float(solved[n]) for n in out.name]
        out.velocity = [0.0] * len(out.name)
        out.effort = [0.0] * len(out.name)
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WristClosureSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
