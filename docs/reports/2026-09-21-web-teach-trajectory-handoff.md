# Web 示教下发轨迹无响应：根因与交接（2026-09-21）

承接 Codex 会话 `01a0be9f`「修复 Web 到 Gazebo 轨迹链路」——该会话验证了
WebSocket→rosbridge→micro-ROS→STM32→CAN2→仿真电机→Gazebo 整条链路，但在排查
「浏览器页面本身不可控」时被中断，交接报告没写完。本文续上，并给出**真实根因**。

> **订正**：本文初版（同日晚些时候）曾下结论「Web 页面代码不是故障点」。那是错的——
> 页面确实有一处会静默吃掉「武装」的缺陷（§二）。当时没测出来，是因为电机卡在
> `CONFIG_CONFLICT`，页面**正确地拒绝了播放**，缺陷被挡在上游没机会暴露。

## 结论摘要

「示教播放无响应」是三层叠起来的，必须全部拆掉才能动：

1. **台架层：四台电机卡 `CONFIG_CONFLICT`**，冲突态禁止使能 → 页面正确地拒绝播放
   并说明原因。**复位控制板即解**（模拟器须先在线当 ACK 节点）。
2. **页面层（真 bug）：`policyGeneration` 每次开页都从 0 重来**，而固件那边的代数门
   是**跨页面持久**的 → 刷新页面后的每一条「武装」都被静默拒（只有事件日志的
   `CONFIG_RESULT reason=2` 看得出），**而轨迹照旧受理、照旧 `completed`**。
   于是：播放"完成"、机构一动不动。这就是用户报的「无响应」。
3. **台架层：模拟器没加 `--gazebo`** → 它只跑自己的内部 plant，从不驱动 Gazebo
   （`/arm_effort_controller/commands` 的 publisher 数为 0）。加上即通。

另外：effort 模式下 `kp=0.2` 顶不住模型重力，**必须给足增益**（本次用 `kp=8 kd=0.5`）。

## 一、台架层：CONFIG_CONFLICT 与它的两个出口

### 症状

```
motor_topology      : 4 台 online=true, conflict=true
motor_status_array  : 4 台 lifecycle=7(CONFIG_CONFLICT), unified_fault_code=6
系统参数            : system.last_error=41 (UNVERIFIED)
页面点「播放」      : 播放前检查失败：电机未就绪：m1(故障6)…
```

页面**把原因说出来了**——这是 Codex 那轮 `playbackReadinessError` 的功劳，它按预期工作。
所以「无响应」在这一层不是页面装死，是它按契约拒绝。

### 来历

2026-09-21 01:46 控制板上电时模拟器不在线（`/dev/ttyACM0` 与 `ACM1` 同刻重建 = 一次
USB 重枚举）。上电归一（0a 阶段）无对象可写 → 判 `UNVERIFIED(41)` → `CONFIG_CONFLICT` 锁存。

### 为什么重启模拟器救不回来（设计如此）

固件 [motor_control.c:1127](../../micro_ros_firmware/lib/motor_control/src/motor_control.c#L1127)
显式豁免：`冲突锁存不降级 OFFLINE：冲突态本就禁运动，且 §9.3 禁止任何"离线→重连"
周期被用来自动解除冲突`。实测停模拟器 35 s：四台仍 `online: true`、lifecycle 恒为 7、
`motor_events` 零事件，模拟器重开后固件也不再重跑归一/回读（sim 日志零 SDO）。

早先"重连会重置 `boot_norm_attempts`"（[motor_control_discovery.c:156](../../micro_ros_firmware/lib/motor_control/src/motor_control_discovery.c#L156)）
的判断对**非冲突态**成立，在冲突态根本到不了。

### 出口

```bash
# 复位控制板（模拟器必须已在线当 ACK 节点）
~/.platformio/packages/tool-openocd/bin/openocd \
  -s ~/.platformio/packages/tool-openocd/openocd/scripts \
  -f interface/stlink.cfg -c 'transport select swd' \
  -f target/stm32h7x.cfg -c 'init' -c 'reset run' -c 'shutdown'
```

复位后实测：四台 `conflict=false`、`lifecycle=3(READY_DISABLED)`、`fault=0`、
`system.last_error=0`、页面参数页「共 4 台 已核对 4」。

（另一条出口是参数面写回，本次未用。）

## 二、页面层：`policyGeneration` 归零吞掉「武装」（真 bug，已修）

### 现象

把上面的台架问题清掉之后，播放**依然不动**：

```
页面  : ✓ 播放完成：5 段；末点保持
固件  : trajectory_accepted: 5, trajectory_rejected: 0
sim   : 收到 cerb_mit 帧 0 条        ← 总线上一帧 MIT 都没有
Gazebo: joint_link1/2 停在 1.5708（零力矩下挂在限位上），一动不动
```

`tx_frame_count` 在涨，但那几乎全是 20ms 的 SYNC：固件**受理了轨迹、却没为任何一台
生成运动帧**。

### 根因

固件 `entry_may_move()` 要求 `policy_enable`；而
[motor_control_config.c:944](../../micro_ros_firmware/lib/motor_control/src/motor_control_config.c#L944)：

```c
if (request->enable && !entry->policy_enable &&
    request->policy_generation <= entry->policy_generation) {
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_STALE_GENERATION;
  return reject_invalid(ctx, entry, request, now_ms);   // 静默拒
}
```

`MotorPolicyArray.msg` 自己写明了这道门：**「内核在表项进入 FAULT / OFFLINE /
CONFIG_CONFLICT 时锁存当时的代数，enabled=true 且代数 <= 锁存值的一律拒绝」**。

而页面 [app.js:1457](../../ros2_ws/src/web_host/web/app.js#L1457) 是 `let policyGeneration = 0;`
——**每次开页/刷新都归零**。固件记得的是上一轮的值（本例 4），页面重发的却是
1、2、3、4，**四条全被拒**。`policy_accepted` 计的是数组级受理，与逐台落地无关，
所以计数器看着正常，只有 `CONFIG_RESULT` 事件的 `reason=2` 能看出真相。

Codex 已经用同款手法修过 `seq`（`Date.now() & 0x7fffffff`，治刷新页面后复用旧
`trajectory_id`），但**漏了 `policyGeneration`**。

**任何一次刷新页面都会触发**——这是日常最容易踩到的那个。

### 修法

```js
// 固件的代数门是跨页面持久的：刷新只是把我们这侧归零，固件记得的仍是上一轮的值。
// 与 seq 同法，用启动时间把起点垫高。
let policyGeneration = (Date.now() & 0x7fffffff) >>> 0;
```

回归验证：全新浏览器（代数种子 `1077178375`）→ 读参数 → 逐台武装 → 四台
`CONFIG_RESULT reason=0`、`lifecycle=4(ACTIVE)`，**不再被拒**。

## 三、台架层：模拟器漏了 `--gazebo`

```bash
ros2 topic info /arm_effort_controller/commands
# 修前：Publisher count: 0      ← 没有任何东西在驱动 Gazebo
# 修后：Publisher count: 1
```

不加 `--gazebo` 时模拟器只跑自己的内部 plant（`joint_map` 只决定"谁接 Gazebo"），
Gazebo 里的臂与整条链路完全脱钩。正确启动：

```bash
python3 micro_ros_agent/tools/fdcan_motor_sim.py --tty /dev/ttyACM1 --gazebo \
  --log /tmp/fdcan-motor-sim.jsonl
# --joint-map 默认就是 1:joint_link1,2:joint_link2,3:joint_link3,4:joint_link4
# --gazebo-control-mode 默认 effort（真实 MIT 动力学）
```

### effort 模式必须给足增益

`kp=0.2 kd=0.8` 时力矩只有零点几 N·m，顶不住模型重力，臂明显滞后（实测停在
-1.53 rad，而目标 -0.025）。换 `kp=8 kd=0.5` 后正常。模拟器自己的注释就写了这点：
「使用它时必须自己提供足够的增益/前馈来对抗模型重力」。

## 四、端到端验收（真实浏览器驱动，2026-09-21）

全部经**真实 Firefox 页面**（WebDriver BiDi 驱动按钮，非裸 WebSocket 脚本）：

```
导入  : 页面原生导入路径 → "导入 270 个点位（轨迹载荷 → 按时刻合成行）"
        ；ℹ 1080 点超单条上限（256），播放时自动分段
分段  : 256 / 256 / 256 / 256 / 72   时长 2100/2100/2100/2100/567 ms
播放  : ▸ 轨迹段 1/5 … → ✓ 播放完成：5 段；末点保持
固件  : trajectory_accepted: 5, rejected: 0；policy_accepted: 4, rejected: 0
总线  : 模拟器收到 cerb_mit 26987 帧（0x111–0x114，四台各 ~6.5k）
```

Gazebo 关节跟踪（`kp=8 kd=0.5`）：

| 关节 | 播放前 | 播放后 | 轨迹末点要求 | 误差 |
|---|---|---|---|---|
| joint_link1 | -0.2835 | **0.0173** | 0.0183 | 0.001 |
| joint_link2 | -1.5289 | **0.0186** | -0.0246 | 0.043 |
| joint_link3 | -0.0562 | **0.0100** | -0.00002 | 0.010 |
| joint_link4 | -0.6800 | **0.0144** | -0.0379 | 0.052 |

1.5 rad 的行程跟到 0.05 rad 以内；四台 `unified_fault_code=0`、`lifecycle=4`，
播完保持末点（两次播放之间实测不回坠）。

## 五、本次改动

| 文件 | 改动 | 状态 |
|---|---|---|
| `ros2_ws/src/web_host/web/app.js` | `policyGeneration` 初值 `0` → `Date.now() & 0x7fffffff` | **已修**，全新浏览器回归通过 |
| `ros2_ws/src/web_host/web/app.js` | `init()` 末尾补 `connect()`：开页即连 | 已在真实页面验证 |

`init()` 改动理由：`rosUrl` 已按 `location.hostname` 填好，没有"先改地址再连"的场景；
而页面打开恒为「未连接」、必须手点「连接」——这正是"上位机不可控"的第一印象。
「断开」仍会立起 `manualDisconnect`，之后不再自动重连。

> Codex 那轮**未提交**的 app.js 改动（播放改回轨迹通道 + 分段 + `trajectory_state`
> 回执串行 + 播放前就绪门 + `primeTrajectorySnapshot`）予以保留，未回退：
> 分段与就绪门两件事都是对的，本次问题出在更上游的授权代数。

## 六、未决

1. **归一为什么不推进到 DONE**（模拟器在线时跑过一轮，`0x2650` 写/读回两侧值一致
   却仍判 41）。**复位后不再复现**，故暂列为"上电时模拟器不在线"的后果；若再遇到，
   从 `boot_norm_stage` 的推进条件查，不要先怀疑总线。
2. **台架易碎性**：控制板任何一次"模拟器不在线"的上电都会把四台锁进冲突，而 §9.3
   又堵死自动恢复，只能人工复位。是否给一个需人工确认的"重做上电归一"入口，
   属设计决策，未动。
3. `drive_mode` 在 `motor_status_array` 里始终为 0，与"四台 ACTIVE 且 MIT 帧在流"
   不一致——未查，可能是该字段语义（`MotorStatusTarget.msg`）而非故障。

## 复现要点

```bash
# 1) 顺序不可颠倒：先让 CANable 成为 ACK 节点（务必带 --gazebo）
python3 micro_ros_agent/tools/fdcan_motor_sim.py --tty /dev/ttyACM1 --gazebo \
  --log /tmp/fdcan-motor-sim.jsonl
# 2) micro-ROS agent（CDC = /dev/ttyACM0，按 by-id 认）
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
# 3) Web 上位机
ros2 launch web_host web_host.launch.py
```

```bash
# 驱动真实页面：Firefox 155 只有 BiDi，没有 CDP；/tmp 对 snap 版 firefox 是私有
# 命名空间，profile 与输出必须放在 $HOME 可见路径下
firefox --headless --no-remote -profile "$(mktemp -d /home/ly/robot_ws/.ff-ctl.XXXXXX)" \
        --remote-debugging-port 9222 --window-size=1600,2000 http://127.0.0.1:8080/

# app.js 是 IIFE → 脚本级 const 在 BiDi 沙箱里不可见，只经 DOM 驱动：
#   document.getElementById('btnTeachPlay').click()，再读 #teachStatus
# 导入文件同理走 DOM：钩住 document.createElement 拿到它自建的那个 <input type=file>，
# 用 DataTransfer 塞 File 后 dispatchEvent(new Event('change'))——真实 onchange/FileReader 路径。
```

**诊断三个坑**（都踩过）：

- `pkill -f "<自己在命令行里写过的字符串>"` 会**杀掉自己的 shell**。改用 `pgrep`
  拿 PID 再 `kill`，且别让启动命令与匹配模式出现在同一条命令行里。
- 总线侧真相看模拟器日志的 `dir`/`tag`：`cerb_mit` 有没有出现，直接决定"固件到底
  发没发运动帧"。**只看 `tx_frame_count` 会被 SYNC 骗过去**（20ms 一条，长期累计上万）。
- 策略的受理**不是**整体的：`policy_accepted` 是数组级计数，逐台落地要看
  `motor_events` 的 `CONFIG_RESULT`（`reason`：0=OK、2=非法）。
