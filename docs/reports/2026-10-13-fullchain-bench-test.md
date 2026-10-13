# 全链路台架测试（2026-10-13）

台架：`simulation.launch.py backend:=mujoco transport:=slcan controller_mode:=firmware
model:=half_body_robot`，18 台电机经 CANable2 由 `fdcan_motor_sim.py` 模拟，
STM32H723 跑真固件。链路 = 模拟器 ⇄ CAN ⇄ 固件 ⇄ micro-ROS ⇄ ROS 话题 ⇄ MuJoCo。

## 1. 通过的部分

### 上行链路（MCU → ROS）—— 全部达标

| 话题 | 实测 | 设计值 |
|---|---|---|
| `imu/data` | 100.12 Hz | 100 |
| `motion_states` | 50.00 Hz | 50 |
| `motor_status_array` | 20.00 Hz | 20 |

18/18 在线；`publish_fail = 0`、`decode_error_count = 0`、`rx_hw_lost`/`ring_drop` 全 0；
无 OFFLINE 抖动；逐电机反馈均龄 6–16 ms、最差 ≤54 ms（远低于 100 ms 离线门）。
（这些是同日修复后的稳态，见 `2026-10-13-motor-feedback-rate-diagnosis.md`。）

### 族间隔离（活体）

`motor_topology` 实测 vaxor = {8,9,17,18}（4 台，`protocol=0`）、CERB = 其余 14 台
（`protocol=1`），无混杂、无 `CONFIG_CONFLICT`。

## 2. 下行链路：能用，但有硬性节奏要求 [已 A/B 验证]

**客户端必须以 >33 Hz 发布命令快照。** 同一台、同一目标，只改节奏：

| 命令节奏 | `target_source` | 生命周期 | 事件 |
|---|---|---|---|
| 50 ms（20 Hz） | **0**（目标未采纳） | SOFT_STOP ↔ ACTIVE 反复 | 持续 SOFT_STOP/ACTIVE 抖动 |
| **10 ms（100 Hz）** | **1**（快照） | **ACTIVE** | 仅 1 条 ACTIVE |

机理：固件的新鲜度级联是 30 ms / 100 ms（`motor_control.c` 的先判定会话/100ms/30ms）。
间隔 50 ms > 30 ms → 每帧快照都被判过期 → 软停 → **目标永不落地**，且软停/激活
交替本身就是在扰动电机。

**据此发现 `tools/traj_play.py` 自己也踩这条线**：`hold()` 用的是
`publish(...); self.spin(0.05)`（_50 ms_），低于 33 Hz 门。该门是 2026-09-15
台架决策（「序号门移除，改由新鲜度门承担」）引入的，之后这个节奏就没再对齐过。

## 3. 找到并修复的崩溃缺陷：模拟器被关节越界打死 [已修]

武装后关节瞬时摆过 ±180°，模拟器进程抛

```
struct.error: 'h' format requires -32768 <= number <= 32767
  at fdcan_motor_sim.py  MotorModel.cerb_frame()
```

退出码 1，launch 随即关掉全部进程（含 MuJoCo 桥，exit **-11**），台架整体下线；
固件侧 18 台全 OFFLINE、`fault=5`、`state_age` 冻结在 ~69 s 并单调爬升。

**根因**：CERB 反馈帧把位置/转速/Iq/温度都塞进 int16，
`struct.pack(">hhh", round(self.pos_deg * 182.0), round(self.rpm), round(self.iq_ma))`
**没有任何范围防护**。而固件侧约定（`motor_protocol.c:217`）是
`position = read_be_i16(data)/182.0 * π/180`，即该字段只能表达 **±180°**——
关节一旦越过 ±180°，`pos_deg*182` 溢出 int16 直接抛异常。

不对称：`vaxor_pos_code`/`vaxor_vel_code`/`vaxor_cur_code` **一直有 clamp**，只有
CERB 这条路漏了。而它的失效方式是**崩进程**，不是丢一帧。

**修复**：新增 `clamp_i16(value, what)` 并用于 `cerb_frame` 的四个字段；越界夹取到
int16 端点并**首次**打印告警（静默夹取会把"关节真跑出量程"这个建模问题藏起来）。
回归测试见 `test/host/test_fdcan_motor_sim.py`（越界不再抛异常 + 量程内不变形）。

## 4. 台架本身的性质：没有"安全静止态" [重要]

- MJCF 里**没有 `<option>`**，即 MuJoCo 默认重力 `(0,0,-9.81)` **生效**；
  18 个关节都有 `damping`，但量级极小（0.0002–0.05）。
- 未受扰动时臂**完全静止**（957 帧样本、10 s、0 个关节在动、effort 全零）——
  说明初始姿态是靠约束/限位维持的，**不是靠任何力矩**。
- 一旦被推离该姿态：无外力矩 + 极小阻尼 + 重力 → 运动长时间不衰减，
  表现就是「整个手臂在乱动」。**恢复手段只有重启被控对象**（重置 qpos/qvel）。

实测：同一台架，测试扰动后臂持续乱动；`simulation.launch.py` 重启后 10 s 内
0 个关节在动，完全恢复。

**结论**：在这台架上做完任何驱动实验后，臂不会自己回到静止——要么重启台架，
要么保持电机武装用 MIT 持续保持某姿态。做全链路测试时必须把"收尾"当成一步。

## 5. 未定论

**命令被采纳后不跟随**：100 Hz 节奏下 `target_source=1`、`lifecycle=ACTIVE`
（目标已落地），但目标 +0.4342 rad、实测走到 −0.2776 rad，误差 0.71 rad。
候选原因（未验证）：
- 只武装部分关节时，其余关节无力矩，整臂在重力下的耦合运动主导了被控关节；
- 该姿态接近限位，限位约束力与命令力矩方向相反；
- 臂上的 `kp=30` 对该关节偏弱。

**已排除**：关节顺序错配——模拟器 `--joint-map` 的顺序
（`joint_link1..9, joint_link1.1..9.9`）与 MuJoCo 桥的 `joint_names` **逐字一致**，
且桥是按**名字**映射并带启动期校验（`mujoco_effort_bridge.py:85-105`）。

**下一步**：单台、单关节、目标设为其**限位内的任意小偏移**，同时记录
`/joint_states` 全部 18 个关节，看是否只有被命令的那台在动——这能把"耦合主导"
与"被控关节本身不跟"分开。

## 6. 过程记录（诚实起见）

本轮测试首次驱动时**打断了台架**：崩溃链见 §3。恢复流程为
停 launch → （若需）复位板子 → 起模拟器 → 复位板子；本轮只重启了台架，
板子未复位（SYNC 立即恢复 50 Hz，证明固件一直健在）。
`fdcan_motor_sim.py` 的崩溃修复在第二次重启后生效。
