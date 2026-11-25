#!/usr/bin/env python3
"""MuJoCo 力矩直通桥：effort 命令 → mj_step → /joint_states，可选 viewer。

话题契约与 Gazebo 路径一致：
* Float64MultiArray 按 joint_names 的逗号分隔顺序映射 actuator，长度须一致；
  未提供名字时，数组须与 MJCF 关节等长且顺序一致。
* command_timeout_s 内无新命令则力矩清零。
* JointState.effort 取 qfrc_actuator，是执行器广义力，不是 Gazebo 的关节传递力矩。
* /clock 取物理时间 data.time；节点自身不用仿真时间，状态时间戳和超时取系统时钟。

声明 robot_sim_control_mode=viewer_position 的 MJCF 使用本地 Viewer 的位置滑块，
关节状态与 /clock 使用同一仿真时间。
position_input=true 时由外部位置数组驱动原位置执行器，用于 STM32 HIL。

需可导入 mujoco 的 Python；launch 自动选择 ~/.ros/ros2_control/.venv。
PYGLFW_LIBRARY_VARIANT 必须由 launch 在导入 mujoco/glfw 前注入，
否则 Wayland viewer 可能自旋并阻塞状态发布。
"""

from __future__ import annotations

import json
import threading
import time
from collections import Counter
from uuid import uuid4

import mujoco
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from mujoco_command_mapping import (
    mapping_error,
    parse_joint_names,
    resolve_command_indices,
)
from mujoco_joint_position import JointPositionTargets
from mujoco_unregistered_hold import UnregisteredInputHold


def model_text(model, name):
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, name)
    if index < 0:
        return None
    start = model.text_adr[index]
    return model.text_data[start:start + model.text_size[index]].rstrip(b"\0").decode()


def viewer_position_mode(model) -> bool:
    mode = model_text(model, "robot_sim_control_mode")
    if mode is None:
        return False
    if mode != "viewer_position":
        raise RuntimeError(f"未知的 robot_sim_control_mode：{mode!r}")
    return True


def initialize_data(model):
    data = mujoco.MjData(model)
    standing = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'Standing')
    if standing >= 0:
        mujoco.mj_resetDataKeyframe(model, data, standing)
        mujoco.mj_forward(model, data)
    return data


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
        viewer_hz = float(self.declare_parameter("viewer_hz", 20.0).value)
        self._viewer_sync_interval = (1.0 / viewer_hz) if viewer_hz > 0.0 else 0.0
        self._last_viewer_sync = 0.0
        command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.5).value)
        headless = bool(self.declare_parameter("headless", False).value)
        self.position_input = bool(self.declare_parameter('position_input', False).value)
        self.registered_topic = str(
            self.declare_parameter(
                'registered_input_topic', '/robot_sim/registered_input_joints').value)
        self.registry_timeout_s = float(
            self.declare_parameter('registry_timeout_s', 0.5).value)
        hold_groups_raw = str(
            self.declare_parameter('hold_groups', '').value)
        self.hold_groups = json.loads(hold_groups_raw) if hold_groups_raw.strip() else []
        speed = float(self.declare_parameter("sim_speed_factor", 1.0).value)
        # 逗号分隔的有序关节名；空 = 按 MJCF 关节顺序（见模块 docstring）
        requested_names = parse_joint_names(
            str(self.declare_parameter("joint_names", "").value)
        )

        if self.position_input:
            # 需要给每个输入轴预置 joint equality，故走 MjSpec 再编译。
            self._spec = mujoco.MjSpec.from_file(model_path)
            self.model = self._spec.compile()
        else:
            self._spec = None
            self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = initialize_data(self.model)
        self.viewer_position = viewer_position_mode(self.model)
        if self.position_input and not self.viewer_position:
            raise RuntimeError('position_input 要求模型使用位置执行器')
        self.position_targets = None
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
        self.steps_per_tick = max(
            1, int(round(speed / (control_hz * self.model.opt.timestep)))
        )
        self._speed = speed
        self._control_hz = control_hz
        if self.viewer_position:
            if requested_names and not self.position_input:
                raise RuntimeError("viewer_position 模型不接受力矩关节映射")
            position_config = model_text(self.model, "robot_sim_joint_position")
            if self.position_input:
                self._configure_command_input(requested_names, '/arm_position_controller/commands', command_timeout_s)
                self.position_holding = True
                self._configure_registry_hold()
            elif position_config is not None:
                self._configure_joint_position(json.loads(position_config))
        else:
            self._configure_command_input(requested_names, effort_topic, command_timeout_s)

        self.publisher = self.create_publisher(JointState, joint_states_topic, 50)
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)
        self.viewer = None
        self._viewer_decoupled = bool(self.position_input or not self.viewer_position)
        self._viewer_lock = threading.Lock()
        self._viewer_event = threading.Event()
        self._viewer_snapshot = None
        self._viewer_stop = False
        self._viewer_data = None
        self._viewer_thread = None
        if not headless:
            try:
                # mujoco 3.6 的 __init__ 不自动导入 viewer 子模块，必须显式导入
                # （用别名以免把 `mujoco` 变成局部名，影响本函数其它引用）
                from mujoco import viewer as mujoco_viewer
                if self._viewer_decoupled:
                    self._viewer_data = mujoco.MjData(self.model)
                    self._viewer_data.qpos[:] = self.data.qpos
                    self._viewer_data.qvel[:] = self.data.qvel
                    self._viewer_data.ctrl[:] = self.data.ctrl
                    self._viewer_data.time = self.data.time
                    mujoco.mj_kinematics(self.model, self._viewer_data)
                    mujoco.mj_comPos(self.model, self._viewer_data)
                    mujoco.mj_tendon(self.model, self._viewer_data)
                    self.viewer = mujoco_viewer.launch_passive(self.model, self._viewer_data)
                else:
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
                if self._viewer_decoupled:
                    self._viewer_thread = threading.Thread(
                        target=self._viewer_sync_loop,
                        name="mujoco-viewer-sync",
                        daemon=True,
                    )
                    self._viewer_thread.start()
            except Exception as error:      # 无显示环境时降级为无头
                self.get_logger().warning(f"打不开 MuJoCo viewer（{error}），改为无头运行")
        self.create_timer(1.0 / control_hz, self._tick)
        if self.viewer_position:
            control_info = (f"  HIL 位置输入 {len(self.command_joint_names)} 路\n"
                            if self.position_input else f"  Viewer 位置滑块 {self.model.nu} 路\n")
        else:
            control_info = (
                f"  命令关节 {len(self.command_joint_names)} 个，"
                f"{'按 joint_names 映射' if requested_names else '按 MJCF 顺序'}："
                f"{', '.join(self.command_joint_names[:3])} …\n"
                f"  订阅 {effort_topic}（超时 {command_timeout_s}s 清零）\n"
            )
        self.get_logger().info(
            f"MuJoCo 桥就绪：{model_path}\n"
            f"{control_info}"
            f"  发布 {joint_states_topic} 与 /clock；控制 {control_hz} Hz × "
            f"{self.steps_per_tick} 步 = {self.model.opt.timestep * self.steps_per_tick * control_hz:.3f}× 实时"
        )

    def _configure_joint_position(self, config):
        self.position_targets = JointPositionTargets(self.model, config)
        joints = self.position_targets.joints
        positions = [float(self.data.qpos[address]) for address in joints.values()]
        if positions != list(self.position_targets.targets.values()):
            self.position_targets.apply(self.data, list(joints), positions)
        self.position_status = {
            "model": config["model"],
            "session": uuid4().hex,
            "limits": self.position_targets.active_limits(),
            "reject": "",
        }
        self.position_publisher = self.create_publisher(String, "/robot_sim/position_status", 1)
        self.create_subscription(JointState, "/robot_sim/joint_position_target", self._on_position, 1)
        self.create_timer(0.5, self._publish_position_status)

    def _configure_registry_hold(self):
        """为未注册输入轴预置物理固定，并订阅后端注册表。

        输入轴 = 位置执行器驱动的关节名（MJCF 声明顺序）。启动时注册表尚未到达，
        全部输入轴固定；注册集合到达后只释放本次注册的轴。注册表失联/超时同样
        回到全固定。耦合组信息由 launch 显式传入（hold_groups）。
        """
        # 记录 base 模型的积分参数，重新编译后核对并恢复，避免 MjSpec 编译
        # 抹掉任何运行期已设的 gravity/timestep（MJCF 里 gravity=0）。
        base_gravity = np.array(self.model.opt.gravity, dtype=float)
        base_timestep = float(self.model.opt.timestep)
        input_names = list(dict.fromkeys(self.actuator_joint_names))
        self.hold = UnregisteredInputHold(self._spec, input_names, self.hold_groups)
        # 加等式后重新编译：关节/执行器顺序不变，命令下标仍然有效。
        self.model = self.hold.model
        self.model.opt.gravity[:] = base_gravity
        self.model.opt.timestep = base_timestep
        self.steps_per_tick = max(
            1, int(round(self._speed / (self._control_hz * self.model.opt.timestep))))
        self.data = initialize_data(self.model)
        for command_index, actuator in self.command_pairs:
            self.command[command_index] = self.data.ctrl[actuator]
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
        self.hold.attach(self.data)
        self._declare_position_gains()
        # 注册表到达前，把所有输入轴固定在模型启动姿态。
        self.hold.sync(set())
        self.registered_inputs = set()
        self.registry_stamp = None
        self._registry_last_stamp_ns = None
        self._registry_receipt_monotonic = None
        self.create_subscription(
            JointState, self.registered_topic, self._on_registry, 1)

    def _declare_position_gains(self):
        self._pending_position_gains = set()
        self.add_on_set_parameters_callback(self._validate_position_gains)
        actuator_counts = Counter(self.actuator_joint_names)
        for actuator, joint in enumerate(self.actuator_joint_names):
            kp = float(self.model.actuator_gainprm[actuator, 0])
            bias = self.model.actuator_biasprm[actuator]
            # MJCF position 编译为固定增益与仿射偏置，可带一阶滤波。
            # 多执行器共享的关节没有唯一的 [kp, kv]。
            if (actuator_counts[joint] != 1
                    or self.model.actuator_dyntype[actuator] not in (
                        mujoco.mjtDyn.mjDYN_NONE, mujoco.mjtDyn.mjDYN_FILTEREXACT)
                    or self.model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or self.model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_AFFINE
                    or bias[0] != 0.0 or bias[1] != -kp or kp < 0 or bias[2] > 0):
                continue
            self.declare_parameter(
                f'position_servo.{joint}', [kp, float(-bias[2])],
                ParameterDescriptor(description='MuJoCo 位置执行器 [kp, kv]，注册时生效'))

    def _validate_position_gains(self, parameters):
        for param in parameters:
            if not param.name.startswith('position_servo.'):
                continue
            if (param.type_ != Parameter.Type.DOUBLE_ARRAY or len(param.value) != 2
                    or not all(np.isfinite(v) and v >= 0 for v in param.value)):
                return SetParametersResult(
                    successful=False,
                    reason=f'{param.name} 需要两个非负、有限的浮点数 [kp, kv]')
            # Humble 的校验回调发生在提交之前。控制拍只读取最终提交的参数值。
            self._pending_position_gains.add(param.name)
        return SetParametersResult(successful=True)

    def _on_registry(self, message):
        """接收注册输入轴集合；拒绝未知/重复/过期/未来/回放帧。"""
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        names = list(message.name)
        known = set(self.actuator_joint_names)
        if len(names) != len(set(names)):
            self.get_logger().warning("注册表含重复输入轴名，忽略本帧")
            return
        unknown = [name for name in names if name not in known]
        if unknown:
            self.get_logger().warning(
                f"注册表含未知输入轴，忽略本帧：{unknown}", throttle_duration_sec=5.0)
            return
        timeout_ns = int(self.registry_timeout_s * 1e9)
        # 未来戳（超过 50ms 容差）不可信，按失联处理，绝不据此释放输入。
        if stamp_ns <= 0 or stamp_ns - now_ns > 50_000_000:
            self.get_logger().warning(
                "注册表时间戳非法/来自未来，回到全固定", throttle_duration_sec=5.0)
            self.registered_inputs = set()
            self.registry_stamp = None
            self._registry_receipt_monotonic = None
            return
        if now_ns - stamp_ns > timeout_ns:
            # 过期：视为失联，回到全固定。
            self.get_logger().warning(
                "注册表时间戳过期，回到全固定", throttle_duration_sec=5.0)
            self.registered_inputs = set()
            self.registry_stamp = None
            self._registry_receipt_monotonic = None
            return
        # 回放/乱序：不晚于上一帧的戳直接拒绝，防止旧注册复活。
        if (self._registry_last_stamp_ns is not None
                and stamp_ns <= self._registry_last_stamp_ns):
            self.get_logger().warning(
                "注册表时间戳未前进，忽略回放帧", throttle_duration_sec=5.0)
            return
        self.registered_inputs = set(names)
        self.registry_stamp = stamp_ns
        self._registry_last_stamp_ns = stamp_ns
        self._registry_receipt_monotonic = time.monotonic()

    def _sync_registry_hold(self) -> None:
        """把注册集合同步到固定逻辑；回执看门狗超时则全固定。

        看门狗用 receipt 单调时钟，防止发布端冻结但 header.stamp 仍看似新鲜，
        也防止回放旧帧维持释放状态。
        """
        now_ns = self.get_clock().now().nanoseconds
        receipt = getattr(self, "_registry_receipt_monotonic", None)
        stale = (
            self.registry_stamp is None
            or receipt is None
            or time.monotonic() - receipt > self.registry_timeout_s
            or now_ns - self.registry_stamp > int(self.registry_timeout_s * 1e9)
        )
        if stale:
            registered = set()
        else:
            registered = set(self.registered_inputs)
        self.hold.sync(registered)

    def _publish_position_status(self):
        self.position_status["limits"] = self.position_targets.active_limits()
        self.position_publisher.publish(String(data=json.dumps(self.position_status, ensure_ascii=False)))

    def _on_position(self, message):
        if message.header.frame_id != self.position_status["session"]:
            return
        try:
            self.position_targets.apply(self.data, message.name, message.position)
        except ValueError as error:
            self.position_status["reject"] = str(error)
        else:
            self.position_status["reject"] = ""

    def _configure_command_input(self, requested_names, command_topic, command_timeout_s) -> None:
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
        if passive and not self.position_input:
            self.get_logger().warning(f"这些关节没有 actuator，命令不会生效：{passive}")

        self.command = np.zeros(len(self.command_joint_names))
        self.command_stamp = None
        self.command_timeout_ns = int(command_timeout_s * 1e9)
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

        self.create_subscription(Float64MultiArray, command_topic, self._on_command, 1)

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(self.command_joint_names):
            self.get_logger().warning(
                f"命令长度 {len(message.data)} ≠ 命令关节数 "
                f"{len(self.command_joint_names)}，忽略",
                throttle_duration_sec=5.0,
            )
            return
        if not np.isfinite(message.data).all():
            self.get_logger().warning('命令包含非有限数，忽略', throttle_duration_sec=5.0)
            return
        self.command[:] = message.data
        self.command_stamp = self.get_clock().now()
        self.position_holding = False

    def _apply_position(self, applied) -> None:
        for command_index, actuator in self.command_pairs:
            target = applied[command_index]
            if self.model.actuator_ctrllimited[actuator]:
                target = np.clip(target, *self.model.actuator_ctrlrange[actuator])
            self.data.ctrl[actuator] = target

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

    def _position_step_count(self, now: float) -> int:
        previous = getattr(self, '_position_wall_at', None)
        self._position_wall_at = now
        if previous is None:
            self._position_remainder_s = 0.
            return self.steps_per_tick
        # Viewer 同步可能阻塞多个控制周期。按单调时钟补足物理时间，
        # 最多补一个命令超时窗口，避免恢复后长时间阻塞命令与注册表回调。
        elapsed = max(0., min(now - previous, self.command_timeout_ns / 1e9))
        simulated = elapsed * self._speed + self._position_remainder_s
        timestep = self.model.opt.timestep
        steps = int((simulated + 1e-12) / timestep)
        self._position_remainder_s = max(0., simulated - steps * timestep)
        return steps

    def _viewer_sync_loop(self) -> None:
        """Dedicated thread for passive MuJoCo viewer sync so X11/GLX never stalls _tick()."""
        while not self._viewer_stop:
            if not self._viewer_event.wait(timeout=0.1):
                continue
            self._viewer_event.clear()
            if self._viewer_stop or self.viewer is None:
                break
            if not self.viewer.is_running():
                self.get_logger().info("MuJoCo viewer 已关闭，改为无头继续仿真")
                self.viewer = None
                break
            with self._viewer_lock:
                snapshot = self._viewer_snapshot
                self._viewer_snapshot = None
            if snapshot is None:
                continue
            qpos, qvel, ctrl, sim_time = snapshot
            self._viewer_data.qpos[:] = qpos
            self._viewer_data.qvel[:] = qvel
            self._viewer_data.ctrl[:] = ctrl
            self._viewer_data.time = sim_time
            mujoco.mj_kinematics(self.model, self._viewer_data)
            mujoco.mj_comPos(self.model, self._viewer_data)
            mujoco.mj_tendon(self.model, self._viewer_data)
            self.viewer.sync()
            interval = getattr(self, "_viewer_sync_interval", 0.0)
            if interval > 0.0:
                time.sleep(interval)

    def destroy_node(self) -> bool:
        self._viewer_stop = True
        if hasattr(self, "_viewer_event"):
            self._viewer_event.set()
        return super().destroy_node()

    def _tick(self) -> None:
        if self.viewer_position:
            if self.position_input:
                steps = self._position_step_count(time.monotonic())
                for name in self._pending_position_gains:
                    self.hold.set_position_gains(
                        name.removeprefix('position_servo.'), *self.get_parameter(name).value)
                self._pending_position_gains.clear()
                now = self.get_clock().now()
                timed_out = (self.command_stamp is None or
                             (now - self.command_stamp).nanoseconds > self.command_timeout_ns)
                if timed_out and not self.position_holding:
                    # 位置电机在命令断流时锁住当前输入轴位置。
                    for command_index, actuator in self.command_pairs:
                        joint = self.model.actuator_trnid[actuator, 0]
                        self.command[command_index] = self.data.qpos[self.model.jnt_qposadr[joint]]
                    self.position_holding = True
                # 注册集合变化 → 释放/固定对应输入轴；未注册轴增益已清零。
                self._sync_registry_hold()
                self._apply_position(self.command)
                # 逐步投影：把未注册输入轴（及整组缺失的输出）钉回固定值，
                # 消除软等式在强执行器扰动下的残差。
                for _ in range(steps):
                    mujoco.mj_step(self.model, self.data)
                    self.hold.project()
            else:
                mujoco.mj_step(self.model, self.data, nstep=self.steps_per_tick)
        else:
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

        seconds = int(self.data.time)
        clock = Clock()
        clock.clock.sec = seconds
        clock.clock.nanosec = int((self.data.time - seconds) * 1e9)
        message = JointState()
        message.header.stamp = clock.clock if self.viewer_position else now.to_msg()
        message.name = list(self.joint_names)
        message.position = self.data.qpos[self.qpos_address].tolist()
        message.velocity = self.data.qvel[self.dof_address].tolist()
        message.effort = self.data.qfrc_actuator[self.dof_address].tolist()
        self.publisher.publish(message)

        self.clock_publisher.publish(clock)

        if self.viewer is not None:
            if getattr(self, "_viewer_decoupled", False) and getattr(self, "_viewer_thread", None) is not None:
                with self._viewer_lock:
                    self._viewer_snapshot = (
                        self.data.qpos.copy(),
                        self.data.qvel.copy(),
                        self.data.ctrl.copy(),
                        float(self.data.time),
                    )
                self._viewer_event.set()
            elif self.viewer.is_running():
                now_mono = time.monotonic()
                if now_mono - getattr(self, "_last_viewer_sync", 0.0) >= getattr(self, "_viewer_sync_interval", 0.0):
                    self.viewer.sync()
                    self._last_viewer_sync = time.monotonic()
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
