#!/usr/bin/env python3
"""MuJoCo 力矩直通桥：订阅 effort 命令 → `mj_step` → 发布 /joint_states（可选 viewer）。

为什么不用 `mujoco_ros2_control` 自带的 ros2_control 集成（本机 0.1.2 实测）：
  它在注册 actuator 时会为每个关节创建话题 `pid_gains/position/<joint>/<index>/pid_state`，
  而 ROS 2 的话题名不允许出现纯数字 token → 节点抛 `InvalidTopicNameError` 直接 abort
  （`Registering actuators` 之后必崩）。本桥只用 motor_sim 需要的两个话题
  （effort 命令 ↔ joint_states），与 `gazebo.launch.py` 那条路的**话题契约完全一致**，
  所以 `simulation.launch.py` / web_host / 手工 `ros2 topic pub` 都不用改。

约定：
  * 命令是 `Float64MultiArray`，顺序由 `joint_names` 参数决定：
      - **给了** `joint_names`（逗号分隔的有序关节名）：数组第 i 个数对应第 i 个**名字**，
        再按名字落到 MuJoCo 的 actuator；长度必须等于名字个数。这是
        `simulation.launch.py` 的用法——它把拓扑顺序的关节名交给桥，因此**不依赖**
        「MJCF 关节顺序恰好等于拓扑顺序」这个巧合（Gazebo 侧同类顺序错位已经踩过一次）。
      - **没给**（默认）：保持旧的「数组顺序 = MJCF 关节顺序、长度 = MJCF 关节数」语义，
        手工 `ros2 topic pub` 那 18 个数的用法不变。
  * 超过 `command_timeout_s` 没有新命令 → 力矩清零（与 motor_sim 的命令超时语义一致）。
  * `/joint_states` 的 effort 字段取 `qfrc_actuator`（执行器施加到关节的广义力），
    与 Gazebo 侧读到的"关节传递力矩"语义略有差别。
  * 每拍按物理时间 `data.time` 发布 `/clock`，与 Gazebo 那条路的 `/clock` 桥对等
    （`mujoco.launch.py` 的 robot_state_publisher 用 `use_sim_time:=true`）。本节点自身
    **不**用仿真时间：`/joint_states` 时间戳与命令超时仍取系统时钟。

依赖：python 需能 `import mujoco`（mujoco_ros2_control 的 venv
`~/.ros/ros2_control/.venv` 里就有；launch 会自动挑它）。

pyGLFW 的 X11 变体**不在这里设**：`import mujoco` 会先载入 `glfw`，脚本内再改
`PYGLFW_LIBRARY_VARIANT` 已经太晚（实测仍走 Wayland 后端，窗口自旋、`sync()` 不返回、
joint_states 一帧不发）。该环境变量由 `mujoco.launch.py` 在**进程外**注入。
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
        # 关节速度上限。URDF 的 `<limit velocity>` 有这条，**MuJoCo 的 `<joint>` 没有
        # 对应属性**，所以 MJCF 靠执行器的 `user` 字段携带它（见 build_mujoco_model.py）。
        # 缺了它被控对象会跑到真机不可能达到的转速——2026-10-15 实测 40 rad/s，MIT 律的
        # `kd·vel` 项自己就把力矩顶进 ±peak 饱和 ⇒ 阻尼退化成 bang-bang，整臂陷入饱和
        # 极限环（速度不衰减、力矩在两个峰值间反复打满）。Gazebo 侧由 Dart 执行该限幅，
        # 所以只有 MuJoCo 这条路发作。
        self.command_dofs = np.array([
            self.model.jnt_dofadr[self.model.actuator_trnid[actuator, 0]]
            for _, actuator in self.command_pairs
        ])
        # 峰值力矩：MuJoCo 的 `actuatorfrcrange` 只**标注**关节合力上限，**不夹**
        # actuator 的 ctrl（2026-10-15 实测：给 30 N·m 就是 30 N·m），所以这里自己执行。
        # MJCF 的值由 build_mujoco_model.py 从拓扑的 `mit_torque_limit_nm` 逐台写入
        # （r58 45 / r48 11 / vaxor 兜底 10），与假电机侧 `--tau-limits` 同源。
        self.command_torque_limits = np.array([
            float(self.model.jnt_actfrcrange[self.model.actuator_trnid[actuator, 0]][1])
            for _, actuator in self.command_pairs
        ])
        if self.model.nuser_actuator > 0:
            self.command_velocity_limits = np.array([
                abs(float(self.model.actuator_user[actuator, 0]))
                for _, actuator in self.command_pairs
            ])
        else:
            self.command_velocity_limits = None
            self.get_logger().warning(
                "MJCF 未携带关节速度上限（执行器 user 字段），被控对象可以跑到任意转速"
            )

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
        """把力矩写进 ctrl（不在命令里的 actuator 清零），并按关节速度上限截断。

        限幅按**物理步**做而不是按控制拍：一拍 `steps_per_tick` 步，只在拍首限一次
        会让关节在拍内多冲好几步（实测冲到上限的 2.5 倍）。只挡"继续加速"、不挡反向
        制动，所以减速与回程不受影响。
        """
        self.data.ctrl[:] = 0.0
        velocity_limits = self.command_velocity_limits
        for slot, (command_index, actuator) in enumerate(self.command_pairs):
            torque = float(np.clip(applied[command_index],
                                   -self.command_torque_limits[slot],
                                   self.command_torque_limits[slot]))
            if velocity_limits is not None:
                limit = velocity_limits[slot]
                if limit > 0.0:
                    velocity = self.data.qvel[self.command_dofs[slot]]
                    if (velocity >= limit and torque > 0.0) or \
                            (velocity <= -limit and torque < 0.0):
                        torque = 0.0
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
