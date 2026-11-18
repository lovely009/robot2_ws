"""未注册输入轴物理固定与部分注册联动。

MuJoCo 的 MJCF 把输入轴（flange 轴）和输出关节（BN_* 轴）都建成普通铰链；
flange 轴由 position actuator 直接驱动，输出关节只经连杆/腱约束联动。HIL 中
后端动态注册电机集合，未注册电机的输入轴不能被主机目标驱动，但若放任自由
会因重力/扰动漂移，破坏被控对象姿态。

本模块给每个可能被固定的关节预置一个 joint equality（目标=固定值，active 由
运行期切换），并把未注册输入轴的 position actuator 增益与偏置整体清零，避免
主机残留命令/阻尼项产生幽灵驱动或残余速度。注册集合变化时：
* 新增注册：active=0 释放该轴并恢复执行器增益/偏置；
* 撤销注册：把当前 qpos 写入 eq_data 并 active=1，立即固定当前位置。

固定集合一次算好再统一设置：先取「未注册输入轴」，再并入「所有输入都未注册
的耦合组输出关节」，最后统一按该集合开/关等式。这样避免「某轴既是输入又是组
输出」时，组的部分释放把它重新放开的顺序错误。

耦合组的部分注册行为由物理本身给出：固定一个输入轴、驱动另一个输入轴时，输出
关节经连杆联动；整个组的输入轴都未注册时输出不漂移。组信息（哪个输入轴同属一
个耦合组、对应哪些输出关节）由 launch 从 motor_topology.yaml +
motor_driver.simulation.yaml 显式构造传入，本模块不猜。

MuJoCo 3.6 的 mjEQ_JOINT 常值约束：data 长度固定 11（qpos0 + 零向量 + solref
占位），目标存 model.eq_data[:,0]，运行期经 data.eq_active 开/关。
"""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np


def _equality_data(target: float) -> list[float]:
    """mjEQ_JOINT 的常值约束 data 数组：qpos0 + 零向量 + solref 占位。"""
    data = [0.0] * 11
    data[0] = float(target)
    data[-1] = 1.0
    return data


class UnregisteredInputHold:
    """对未注册输入轴施加物理固定；注册集合由外部每拍同步。"""

    def __init__(self, spec, input_joint_names: Iterable[str],
                 groups: Iterable[dict] | None = None) -> None:
        self._spec = spec
        self._input = list(dict.fromkeys(input_joint_names))
        self._groups = [dict(group) for group in (groups or ())]
        self._equality_id = {}       # {joint_name: int} 每个可固定关节一个等式
        self._qpos_address = {}
        self._dof_address = {}
        self._actuator_of_input = {}
        self._registered_gain = {}   # {input: (gainprm0,biasprm0,biasprm1,biasprm2)}
        self._fixed = {}
        self._active = set()         # 当前被固定（eq_active=1）的关节名
        self._build()

    def _build(self) -> None:
        base = self._spec.compile()
        held = list(self._input)
        for group in self._groups:
            held.extend(group.get("outputs", ()))
        for name in dict.fromkeys(held):
            joint = base.joint(name)
            self._qpos_address[name] = joint.qposadr[0]
            self._dof_address[name] = joint.dofadr[0]
            self._fixed[name] = float(base.qpos0[joint.qposadr[0]])
        for actuator in range(base.nu):
            joint = base.joint(base.actuator_trnid[actuator, 0])
            if joint.name in self._input:
                self._actuator_of_input[joint.name] = actuator
                self._registered_gain[joint.name] = (
                    float(base.actuator_gainprm[actuator, 0]),
                    float(base.actuator_biasprm[actuator, 0]),
                    float(base.actuator_biasprm[actuator, 1]),
                    float(base.actuator_biasprm[actuator, 2]),
                )
        # 预置全部等式；启动期 active=0，由 sync() 按固定集合打开。
        for name in self._qpos_address:
            eq = self._spec.add_equality(
                type=2, name1=name, data=_equality_data(self._fixed[name]), active=0)
            self._equality_id[name] = eq.id
        self.model = self._spec.compile()

    def attach(self, data) -> None:
        """绑定编译后的 MjData；初始用 qpos0 作为固定值（调用方随后 sync）。"""
        self.data = data
        for name, address in self._qpos_address.items():
            self._fixed[name] = float(data.qpos[address])
        self._active = set()

    def _set_active(self, name: str, active: bool) -> None:
        eq = self._equality_id[name]
        if active:
            # 新固定：以当前 qpos 为目标（撤销注册固定当前位置）。
            if name not in self._active:
                self._fixed[name] = float(self.data.qpos[self._qpos_address[name]])
            self.model.eq_data[eq, 0] = self._fixed[name]
            self.data.eq_active[eq] = 1
            self._active.add(name)
        else:
            self.data.eq_active[eq] = 0
            self._active.discard(name)

    def set_position_gains(self, name: str, kp: float, kv: float) -> None:
        """下一次 sync 按注册状态应用位置伺服参数。"""
        self._registered_gain[name] = (kp, 0.0, -kp, -kv)

    def _apply_gain(self, name: str, registered: bool) -> None:
        actuator = self._actuator_of_input[name]
        g0, b0, b1, b2 = self._registered_gain[name]
        if registered:
            self.model.actuator_gainprm[actuator, 0] = g0
            self.model.actuator_biasprm[actuator, 0] = b0
            self.model.actuator_biasprm[actuator, 1] = b1
            self.model.actuator_biasprm[actuator, 2] = b2
        else:
            # position 执行器力 = gain*ctrl + bias0 + bias1*q + bias2*v；
            # 四项全清才真正无出力（只清 gainprm 仍残留 -kp*q）。
            self.model.actuator_gainprm[actuator, 0] = 0.0
            self.model.actuator_biasprm[actuator, 0] = 0.0
            self.model.actuator_biasprm[actuator, 1] = 0.0
            self.model.actuator_biasprm[actuator, 2] = 0.0

    def sync(self, registered: set[str]) -> None:
        """按最新注册集合统一增删固定约束与执行器增益/偏置。"""
        registered = set(registered)
        # 1) 先算固定集合：未注册输入轴 ∪ 全输入缺失组的输出关节。
        to_fix = {name for name in self._input if name not in registered}
        for group in self._groups:
            inputs = group.get("inputs", ())
            outputs = group.get("outputs", ())
            if inputs and not (set(inputs) & registered):
                to_fix.update(outputs)
        # 2) 统一按该集合开/关（输入兼组输出时以固定集合为准，不会被释放）。
        for name in self._qpos_address:
            self._set_active(name, name in to_fix)
        # 3) 输入轴执行器增益/偏置按注册状态设置。
        for name in self._input:
            self._apply_gain(name, name in registered)

    def project(self) -> None:
        """把未注册输入轴钉回固定值并清零速度（消除软约束残差与残余速度）。"""
        if self.data is None:
            return
        for name in self._active:
            eq = self._equality_id[name]
            self.data.qpos[self._qpos_address[name]] = self.model.eq_data[eq, 0]
            self.data.qvel[self._dof_address[name]] = 0.0

    def fixed_value(self, name: str) -> float:
        return self._fixed[name]

    def is_active(self, name: str) -> bool:
        return name in self._active
