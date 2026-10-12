#!/usr/bin/env python3
"""MuJoCo 力矩直通桥：订阅 effort 命令 → `mj_step` → 发布 /joint_states（可选 viewer）。

为什么不用 `mujoco_ros2_control` 自带的 ros2_control 集成（本机 0.1.2 实测）：
  它在注册 actuator 时会为每个关节创建话题 `pid_gains/position/<joint>/<index>/pid_state`，
  而 ROS 2 的话题名不允许出现纯数字 token → 节点抛 `InvalidTopicNameError` 直接 abort
  （`Registering actuators` 之后必崩）。本桥只用 motor_sim 需要的两个话题
  （effort 命令 ↔ joint_states），与 `gazebo.launch.py` 那条路的**话题契约完全一致**，
  所以 `simulation.launch.py` / web_host / 手工 `ros2 topic pub` 都不用改。

约定：
  * 命令是 `Float64MultiArray`，长度必须等于关节数，顺序 = **MJCF 里的关节顺序**
    （由 tools/build_mujoco_model.py 生成，等于 URDF 物理关节顺序：joint_link1..9 → 1.1..9.9）。
  * 超过 `command_timeout_s` 没有新命令 → 力矩清零（与 motor_sim 的命令超时语义一致）。
  * `/joint_states` 的 effort 字段取 `qfrc_actuator`（执行器施加到关节的广义力），
    与 Gazebo 侧读到的"关节传递力矩"语义略有差别。

依赖：python 需能 `import mujoco`（mujoco_ros2_control 的 venv
`~/.ros/ros2_control/.venv` 里就有；launch 会自动挑它）。
"""

from __future__ import annotations

import os

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MujocoEffortBridge(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_effort_bridge")
        model_path = str(self.declare_parameter("mujoco_model", "").value)
        if not model_path:
            raise RuntimeError("需要 mujoco_model 参数（MJCF 路径）")
        effort_topic = str(
            self.declare_parameter("effort_topic", "/arm_effort_controller/commands").value
        )
        joint_states_topic = str(
            self.declare_parameter("joint_states_topic", "/joint_states").value
        )
        control_hz = float(self.declare_parameter("control_hz", 100.0).value)
        command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.5).value)
        headless = bool(self.declare_parameter("headless", False).value)
        speed = float(self.declare_parameter("sim_speed_factor", 1.0).value)

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(self.model.njnt)
        ]
        self.qpos_address = np.array(
            [self.model.jnt_qposadr[i] for i in range(self.model.njnt)]
        )
        self.dof_address = np.array(
            [self.model.jnt_dofadr[i] for i in range(self.model.njnt)]
        )
        # actuator 与关节的对应：按 actuator 的目标关节查，不依赖命名或顺序
        self.actuator_of_joint = np.full(self.model.njnt, -1, dtype=int)
        for actuator in range(self.model.nu):
            self.actuator_of_joint[self.model.actuator_trnid[actuator, 0]] = actuator
        passive = [name for index, name in enumerate(self.joint_names)
                   if self.actuator_of_joint[index] < 0]
        if passive:
            self.get_logger().warning(f"这些关节没有 actuator，命令不会生效：{passive}")

        self.command = np.zeros(len(self.joint_names))
        self.command_stamp = None
        self.command_timeout_ns = int(command_timeout_s * 1e9)
        self.steps_per_tick = max(
            1, int(round(speed / (control_hz * self.model.opt.timestep)))
        )

        self.create_subscription(Float64MultiArray, effort_topic, self._on_command, 10)
        self.publisher = self.create_publisher(JointState, joint_states_topic, 50)
        self.viewer = None
        if not headless:
            try:
                # pyglfw 在 Wayland 会话默认加载 wheel 里的 wayland 版库，本机实测该后端
                # 窗口无响应、sync() 自旋（主线程 87% CPU、3D 区全黑）；wheel 里同时带
                # X11 变体，强制用它走 XWayland。注意必须在 import glfw 之前设。
                if os.environ.get("XDG_SESSION_TYPE") == "wayland" and os.environ.get("DISPLAY"):
                    os.environ["PYGLFW_LIBRARY_VARIANT"] = "x11"
                    os.environ.pop("WAYLAND_DISPLAY", None)
                # mujoco 3.6 的 __init__ 不自动导入 viewer 子模块，必须显式导入
                # （用别名以免把 `mujoco` 变成局部名，影响本函数其它引用）
                from mujoco import viewer as mujoco_viewer
                self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
                # 默认自由相机经常没框住模型（开窗只见局部或近乎全黑），按包围盒对准
                self.viewer.cam.lookat[:] = self.model.stat.center
                self.viewer.cam.distance = self.model.stat.extent * 1.8
                self.viewer.cam.elevation = -20.0
                self.viewer.cam.azimuth = 135.0
            except Exception as error:      # 无显示环境时降级为无头
                self.get_logger().warning(f"打不开 MuJoCo viewer（{error}），改为无头运行")
        self.create_timer(1.0 / control_hz, self._tick)
        self.get_logger().info(
            f"MuJoCo 桥就绪：{model_path}\n"
            f"  关节 {len(self.joint_names)} 个（{', '.join(self.joint_names[:3])} …）\n"
            f"  订阅 {effort_topic}（顺序同关节顺序，超时 {command_timeout_s}s 清零）\n"
            f"  发布 {joint_states_topic}；控制 {control_hz} Hz × "
            f"{self.steps_per_tick} 步 = {self.model.opt.timestep * self.steps_per_tick * control_hz:.3f}× 实时"
        )

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(self.joint_names):
            self.get_logger().warning(
                f"命令长度 {len(message.data)} ≠ 关节数 {len(self.joint_names)}，忽略",
                throttle_duration_sec=5.0,
            )
            return
        self.command[:] = message.data
        self.command_stamp = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        timed_out = (
            self.command_stamp is None
            or (now - self.command_stamp).nanoseconds > self.command_timeout_ns
        )
        applied = np.zeros(len(self.joint_names)) if timed_out else self.command
        for index, actuator in enumerate(self.actuator_of_joint):
            if actuator >= 0:
                self.data.ctrl[actuator] = applied[index]
        for _ in range(self.steps_per_tick):
            mujoco.mj_step(self.model, self.data)

        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = list(self.joint_names)
        message.position = self.data.qpos[self.qpos_address].tolist()
        message.velocity = self.data.qvel[self.dof_address].tolist()
        message.effort = self.data.qfrc_actuator[self.dof_address].tolist()
        self.publisher.publish(message)
        if self.viewer is not None:
            if self.viewer.is_running():
                self.viewer.sync()
            else:                           # 用户关掉窗口后继续无头仿真，别卡在 sync 上
                self.get_logger().info("MuJoCo viewer 已关闭，改为无头继续仿真")
                self.viewer = None


def main() -> None:
    rclpy.init()
    try:
        rclpy.spin(MujocoEffortBridge())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
