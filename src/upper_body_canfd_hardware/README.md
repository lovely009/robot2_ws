# upper_body_canfd_hardware

C++ 实现的真机 ros2_control 硬件层，只负责“ROS 关节模型 ↔ 两条私有电机协议”的转换，
控制器不需要接触 SocketCAN，因此可以不加修改地在仿真与真机之间复用。

## 架构

```text
ROS 控制器 -> position 命令接口 -> CanFdSystem -> can0/can1 -> 电机
ROS 控制器 <- position/velocity/effort 状态 <- CanFdSystem <- 电机反馈
```

- 单个 `CanFdSystem` 实例同时管理左右两条总线（`left_interface` / `right_interface`）；
- 每臂 7 个关节：RP ID 1..4（肩 yaw/roll/pitch + 肘 pitch）+ 25/30 mm 腕部节点 N 1..3；
- 14 个关节的**顺序在 URDF 里硬校验**，防止映射错位；
- RP 命令合并为一条 64 字节 MIT 广播（空槽补零），腕部每节点一条 12 字节 MIT 帧；
- 反馈解码：RP 状态帧（CAN ID = 电机 ID，12 字节）与腕部 MIT 回复（`0x701+4N`）/ 周期状态（`0x702+4N`）。

## 工作流程

- `on_init`：校验 14 个关节名称与接口（position 命令 + position/velocity/effort 状态），解析参数；
- `on_activate`：打开两条总线 → 电机仍处于失能状态时先发零指令并读取全部 14 关节当前反馈（不发明零位，避免使能瞬间跳变）→ 以当前姿态作为初始目标 → 再统一使能；
- `read()`：持续解码反馈，任一关节超过 `feedback_timeout_s` 无反馈即返回 ERROR；
- `write()`：把 14 路 position 命令映射为 RP 广播 + 腕部 MIT 帧下发；
- `on_deactivate`：先统一失能，再关闭总线。

## 参数（URDF 硬件块）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `left_interface` / `right_interface` | `can0` / `can1` | 左右臂 SocketCAN 接口 |
| `rp_kp` / `rp_kd` | `30.0` / `1.0` | RP 关节 MIT 刚度/阻尼 |
| `wrist_kp` / `wrist_kd` | `0.008` / `1.0` | 腕部 MIT 增益（手册建议初调值） |
| `wrist_torque_constant_nm_per_amp` | `0.35` | 腕部电流→扭矩常数（仿真假设，待真实型号标定） |
| `feedback_timeout_s` | `0.1` | 反馈超时阈值 |

## 真机使用

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up

cd ~/robot_ws
source install/setup.bash
ros2 launch upper_body_canfd_gazebo hardware.launch.py \
  left_interface:=can0 right_interface:=can1
```

按文档中的关节顺序向 `/position_controller/commands` 发布 14 路 position 指令。

## 测试

```bash
colcon build --symlink-install --packages-select upper_body_canfd_hardware
colcon test --packages-select upper_body_canfd_hardware
colcon test-result --verbose
```

当前为协议级 gtest（`test/test_protocol.cpp`）：对协议手册的已知字节序列断言
（RP 中性槽、腕部命令与零反馈）。尚未覆盖插件级/vcan 回环测试，真机冒烟前建议补齐。

## 安全提示

- 不要与 `canfd_actions`、`canfd_demo`、`canfd_keyboard` 等直连总线的旧工具同时运行，
  一条总线同一时刻只能有一个命令所有者；
- 首次通电前必须逐电机核对 ID、零位、方向、软/硬限位、急停与增益，并在机构卸载状态下验证；
- 默认参数是调试值，不是安全认证值；腕部扭矩常数等参数拿到真实规格后必须重新标定。
