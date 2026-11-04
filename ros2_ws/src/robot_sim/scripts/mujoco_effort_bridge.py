#!/usr/bin/env python3
"""MuJoCo 力矩直通桥：effort 命令 → mj_step → /joint_states，可选 viewer。

话题契约与 Gazebo 路径一致：
* Float64MultiArray 按 joint_names 的逗号分隔顺序映射 actuator，长度须一致；
  未提供名字时，数组须与 MJCF 关节等长且顺序一致。
* command_timeout_s 内无新命令则力矩清零。
* JointState.effort 取 qfrc_actuator，是执行器广义力，不是 Gazebo 的关节传递力矩。
* /clock 取物理时间 data.time；节点自身不用仿真时间，状态时间戳和超时取系统时钟。

需可导入 mujoco 的 Python；launch 自动选择 ~/.ros/ros2_control/.venv。
PYGLFW_LIBRARY_VARIANT 必须由 launch 在导入 mujoco/glfw 前注入，
否则 Wayland viewer 可能自旋并阻塞状态发布。
"""

from __future__ import annotations

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mujoco_command_mapping import (
    mapping_error,
    parse_joint_names,
    resolve_command_indices,
)


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
        # 逗号分隔的有序关节名；空 = 按 MJCF 关节顺序（见模块 docstring）
        requested_names = parse_joint_names(
            str(self.declare_parameter("joint_names", "").value)
        )

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        # MJCF 关节顺序：用于 /joint_states 的名字与 qpos/qvel/qfrc 的取址
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
        # actuator 驱动的关节名，顺序 = MJCF 声明顺序（生成器按 URDF 顺序写 <motor>）
        self.actuator_joint_names = [
            mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                self.model.actuator_trnid[actuator, 0],
            )
            for actuator in range(self.model.nu)
        ]
        # 启动期就 fail fast：名字重复/不在模型里不该等到运行期每帧丢命令
        # （长度按「命令关节数」占位，只为触发名字类校验；长度本身每帧再查）
        startup_error = mapping_error(
            requested_names,
            self.actuator_joint_names,
            len(requested_names) or len(self.actuator_joint_names),
        )
        if startup_error is not None:
            raise RuntimeError(f"joint_names 无法映射到 MuJoCo 模型：{startup_error}")
        self.command_joint_names = requested_names or list(self.actuator_joint_names)
        self.command_pairs = resolve_command_indices(
            requested_names, self.actuator_joint_names
        )
        passive = sorted(set(self.joint_names) - set(self.actuator_joint_names))
        if passive:
            self.get_logger().warning(f"这些关节没有 actuator，命令不会生效：{passive}")

        self.command = np.zeros(len(self.command_joint_names))
        self.command_stamp = None
        self.command_timeout_ns = int(command_timeout_s * 1e9)
        self.steps_per_tick = max(
            1, int(round(speed / (control_hz * self.model.opt.timestep)))
        )
        # 关节速度上限（URDF 的 `<limit velocity>` 落在执行器的 `user` 字段）**本节点
        # 不使用**：被控对象不做速度约束，见 _apply_effort。
        self.command_dofs = np.array([
            self.model.jnt_dofadr[self.model.actuator_trnid[actuator, 0]]
            for _, actuator in self.command_pairs
        ])
        # 只使用已启用的关节限幅；未启用的 [0, 0] 不能当作零力矩上限。
        # MuJoCo 仍执行模型自身的 actuator ctrlrange / forcerange 限幅。
        self.command_torque_limits = np.array([
            float(self.model.jnt_actfrcrange[self.model.actuator_trnid[actuator, 0]][1])
            if self.model.jnt_actfrclimited[self.model.actuator_trnid[actuator, 0]]
            else float('inf')
            for _, actuator in self.command_pairs
        ])

        self.create_subscription(Float64MultiArray, effort_topic, self._on_command, 10)
        self.publisher = self.create_publisher(JointState, joint_states_topic, 50)
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)
        self.viewer = None
        if not headless:
            try:
                # mujoco 3.6 的 __init__ 不自动导入 viewer 子模块，必须显式导入
                # （用别名以免把 `mujoco` 变成局部名，影响本函数其它引用）
                from mujoco import viewer as mujoco_viewer
                self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
                # 打印实际选中的 pyGLFW 平台：Wayland 后端会自旋（窗口黑、sync() 不返回、
                # /joint_states 一帧不发），只打窗口是否成功是不够的失败信号。
                try:
                    import glfw
                    self.get_logger().info(
                        "MuJoCo viewer 平台：%s / platform=%s"
                        % (glfw.get_version_string().decode(errors="replace"),
                           hex(glfw.get_platform()))
                    )
                except Exception:       # pyGLFW 没有该 API 时不该因此起不来
                    pass
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
            f"  命令关节 {len(self.command_joint_names)} 个，"
            f"{'按 joint_names 映射' if requested_names else '按 MJCF 顺序'}："
            f"{', '.join(self.command_joint_names[:3])} …\n"
            f"  订阅 {effort_topic}（超时 {command_timeout_s}s 清零）\n"
            f"  发布 {joint_states_topic} 与 /clock；控制 {control_hz} Hz × "
            f"{self.steps_per_tick} 步 = {self.model.opt.timestep * self.steps_per_tick * control_hz:.3f}× 实时"
        )

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(self.command_joint_names):
            self.get_logger().warning(
                f"命令长度 {len(message.data)} ≠ 命令关节数 "
                f"{len(self.command_joint_names)}，忽略",
                throttle_duration_sec=5.0,
            )
            return
        self.command[:] = message.data
        self.command_stamp = self.get_clock().now()

    def _apply_effort(self, applied) -> None:
        """写入 ctrl，未映射 actuator 清零，按物理步截断峰值力矩。

        不执行额外速度限制，避免删除阻尼或与假电机限速冲突；关节量程由 MJCF 约束。
        """
        self.data.ctrl[:] = 0.0
        for slot, (command_index, actuator) in enumerate(self.command_pairs):
            torque = float(np.clip(applied[command_index],
                                   -self.command_torque_limits[slot],
                                   self.command_torque_limits[slot]))
            self.data.ctrl[actuator] = torque

    def _tick(self) -> None:
        now = self.get_clock().now()
        timed_out = (
            self.command_stamp is None
            or (now - self.command_stamp).nanoseconds > self.command_timeout_ns
        )
        applied = (
            np.zeros(len(self.command_joint_names)) if timed_out else self.command
        )
        for _ in range(self.steps_per_tick):
            self._apply_effort(applied)
            mujoco.mj_step(self.model, self.data)

        message = JointState()
        message.header.stamp = now.to_msg()
        message.name = list(self.joint_names)
        message.position = self.data.qpos[self.qpos_address].tolist()
        message.velocity = self.data.qvel[self.dof_address].tolist()
        message.effort = self.data.qfrc_actuator[self.dof_address].tolist()
        self.publisher.publish(message)

        seconds = int(self.data.time)
        clock = Clock()
        clock.clock.sec = seconds
        clock.clock.nanosec = int((self.data.time - seconds) * 1e9)
        self.clock_publisher.publish(clock)

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
