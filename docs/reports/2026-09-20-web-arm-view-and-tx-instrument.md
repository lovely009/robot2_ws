# Web 臂视图与发送侧仪表（2026-09-20）

> 环境：STM32H723VGT6 + ST-LINK/V2 烧录（OpenOCD `reset run` 复位）+ CANable2
> SLCAN（`/dev/ttyACM1`，`b158aa7`，`S8/Y5/O`）+ **九节点模拟器**（`fdcan_motor_sim.py`，
> 不是真机）+ Ignition Gazebo 作为被控对象；micro-ROS agent 走板载 USB CDC。
> 固件基线 `cb89e3e`（本轮在其上改工作树）。
> 判据：模拟器的**逐帧 JSONL 收发日志**（`--log`）＋ 固件诊断计数，两者独立对照。

## 结论摘要

1. **下行控制链已打通并留下端到端证据**：上位机命令 → 固件内核受理 → MIT 帧上总线
   → 被控对象运动，每一跳都有数；模拟器实收 MIT 帧的 CAN ID 与目标电机逐一对上。
2. **"板子到底发了多少帧"这个仪表此前是坏的，本轮修好并交叉验证。**
   `rx_hw_lost` / `tx_frame_count` 两个序列从未接线（`.data == NULL`），每次发布
   往地址 `0x0/0x4/0x8` 写 12 字节并互相覆盖——读出来的两个字段逐位相同。
   修前报 50 帧/s、真值 208 帧/s；修后 145.8 vs 158.4，同窗对账闭合。
3. **实测 MIT 命令帧是每台 ~39.4 Hz，不是设计意图的 200 Hz。** 控制边界确实按
   5 ms 跑，但没有变成总线上的 200 Hz。**未解**，见「未决」。
4. 模拟器有两个独立的健壮性缺陷，都会**打死整个进程**进而让总线失去 ACK 节点、
   把控制板打进 bus-off——一个 10 字节的非法 CAN FD 长度，一个未处理的串口
   `EAGAIN`。都已修并带可运行自检。
5. Web 上位机改为**以臂为中心的单页**，臂由 URDF 经 three.js 实时渲染。

## 一、发送侧仪表：一个会骗人的数字

### 症状

`tx_frame_count`（每总线"成功提交给硬件"的发送帧数）在**有命令**和**没命令**时
都稳定报 ~50 帧/s，恰好等于 SYNC 的速率（20 ms 周期）。这个数字太合理，所以
一直没被怀疑——直到拿模拟器的逐帧日志对账：

```
                     修复前          修复后
tx_frame_count[0]    50 帧/s    →   145.8 帧/s   ┐ 同一窗口
模拟器实收           208 帧/s   →   158.4 帧/s   ┘ 量级一致
rx_hw_lost          与 tx 逐位相同 → [0,0,0]（独立）
```

修后的差额来自窗口边界：模拟器的 6.07 s 全是武装期（MIT 帧满跑），而诊断的
10.05 s 里约 4 s 没在发命令。算式闭合：
`958 MIT + 50 × 10.05 SYNC = 1460 帧 / 10.05 s = 145.3 帧/s`。

### 根因

`MotorDiagnostics` 里三个 `uint32[<=3]` 序列各有独立静态缓冲，但
`motor_bridge_create()` 只接了 `ring_drop` 一条：

```c
g_diagnostics_msg.ring_drop.capacity = MOTOR_CONTROL_MAX_BUSES;
g_diagnostics_msg.ring_drop.data = g_diagnostics_ring_buf;
// rx_hw_lost / tx_frame_count —— 没有这两行
```

漏接的序列 `.data` 为 `NULL`，而写入侧照写 `.data[i]`：地址 `0x0/0x4/0x8`
（STM32H7 的 ITCM 在 0 处，**写不报错**）。两个漏接的序列落在同一组地址上，
后写的 `tx_frame_count` 覆盖先写的 `rx_hw_lost`，序列化时又都从这组地址读，
于是两个字段必然逐位相同、且读出来的是后写者的值。

- `rx_hw_lost` 此前"看起来是对的"纯属侥幸：那时只有它一个在往 NULL 写。
- 本轮加入 `tx_frame_count` 后两只手同时伸进同一块地址，问题才暴露。
- **代价**：整整一轮"发送频率"的判断建立在错数上（见 `cb89e3e` 的动机说明）。

### 修法

1. 补两条缓冲与接线（`g_diagnostics_rx_lost_buf` / `g_diagnostics_tx_count_buf`）。
2. 在 `motor_bridge_create()` 里加**启动期硬失败守卫**：任一诊断序列
   `.data == NULL` 或 `.capacity` 不足即返回 `false`。这类"漏接线就静默写 NULL"
   的坑不该靠下次读数值去发现。

固件体量：RAM 261,020 B / 577,536 B（45.2%，其中 +24 B 即两个 3×4 缓冲），
Flash 243,840 B（23.3%）。

## 二、台架：让发现结果与物理接线一致

### 复位前后的差别

换线（CANable 从 FDCAN2 移回 FDCAN1）之后板子没复位，发现结果是旧的：

| | 复位前 | 复位后 |
|---|---|---|
| `bus_index` | 9 台全 = 1 | 9 台全 = 0 |
| `online` | m1–m4 在线，m5–m9 离线 | 9 台全在线 |
| `conflict` | m5–m9 也冲突 | m5–m9 干净；m1–m4 仍冲突 |

复位用 OpenOCD `reset run`（**ACK 节点在先**的台架顺序：模拟器先开通道，再复位）。

### 为什么不动：一道明确的安全门

发命令后 `snapshot_accepted` 全数受理、`policy_accepted` 全数受理，但
`config_rejected` 逐条对应、事件流给出
`CONFIG_RESULT motor=1 reason=2`。`reason=2` = `MOTOR_PROTOCOL_CONFIG_INVALID`，
落在 `motor_control_config.c` 的 `SET_ENABLE`：

```c
// 设备参数初始化未完成（未回读/回读失败/与记录不一致）：使能请求直接
// 拒绝并说明原因，不静默吞掉——页面要能立刻看到"为什么按不下去"。
if (!motor_control_dev_params_allow_motion(entry)) {
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
  return reject_invalid(ctx, entry, request, now_ms);
}
```

即 **`CONFIG_CONFLICT` 的台被固件故意拒绝使能**，不是缺陷。m1–m4 的冲突源自
W25 旧记录，恢复路径是参数页「读取」→ 把限值写回驱动板。本轮绕开它，改用
**m5–m8**——模拟器的 Gazebo 桥接的正好是这四台（`--joint-map 5:joint_link1,...`），
复位后四台全在线且无冲突。

### 端到端证据

```
cerb_mit  id=0x115(m5) 239 帧   cerb_mit  id=0x116(m6) 239 帧
vaxor_mit id=0x51D(m7) 240 帧   vaxor_mit id=0x521(m8) 240 帧
config_rejected: 0 · snapshot_accepted: 120 · tx_dropped: 0 · 模拟器零崩溃
```

`target_source` 由 0 → **1（快照）**，Gazebo 侧 `/joint_states` 随之运动。

### MIT 命令帧率（实测，未解）

```
m5 39.4 帧/s · m6 39.4 · m7 40.0 · m8 40.0     中位间隔 26.1 ms
四台合计 157.8 帧/s，加 SYNC 50 帧/s → 总线 ~208 帧/s
```

**目标不变时该速率会掉到 ~1 帧/s**（早期误判的来源）：用常量目标测总线，
只看到 SYNC 的 50 帧/s。

## 三、模拟器：两个会打死进程的缺陷

两者后果相同且严重：模拟器一死总线就失去 ACK 节点，控制板随即 bus-off，
**半秒的抖动毁掉整轮台架**。都只在特定路径上触发，此前所有轮次都没走到。

| 缺陷 | 触发路径 | 症状 | 修法 |
|---|---|---|---|
| `vaxor_reply_frame()` 组 **10 字节** | 收到 VAXOR（m7/m8）MIT 命令后回显 | CAN FD 没有 10 这个合法长度 → `FD_LEN_CODE[10]` → `KeyError: 10` → 进程死 | 补满 **12 字节**（固件 `motor_protocol_pack_vaxor_reply` 写的是 `uint8_t[12]`，解码 `unpack_vaxor_feedback` 认 size==12 或 20、只读字节 2..9，尾 2 字节不使用）；非法长度改报带 CAN id 的明确错误 |
| `send()` 不处理 `EAGAIN` | USB CDC 写缓冲瞬时满 | `BlockingIOError` 直接抛出 → 进程死 | 有界重试（250 ms）；持续背压则**丢这一帧并计数**，绝不退出；开通道的配置命令仍 fail-fast |

自检（可运行，在 `tools/` 下按需重跑）：12 字节 + SLCAN 行合法；短暂背压重试成功、
持续背压丢帧不退出、配置命令写不进去仍抛错。

另：九节点 × 50 Hz 的反馈会把 CANable 的 CDC 打满，表现为持续背压。台架只需
这条臂时用 `--cerb 5,6 --vaxor 7,8` 收窄节点集，负载降到 4/9，收帧 200/s 零丢帧。

## 四、Web 上位机：以臂为中心

### 结构

取消 Tab 分页（此前"单页布局"的注释配"分页"的代码，自相矛盾），改为：左栏关节索引
／中栏臂视图（主角）／右栏选中关节就地 ／常驻下发条 ／底部抽屉（曲线·事件日志·
命令历史·示教·IMU）。

- **点关节选中**：左栏点一行、或点臂上的关节小球，右栏随之切换。
- **逐台参数与策略**：限值（驱动板真值）+ 使能/kp/kd 合成一张逐台表，每台一行、
  各自一个「策略」按钮。原先独立的「逐电机策略」表单已删除。
- **所有面板可折叠**，折叠状态记进 `localStorage`；**收起后标题栏带实时摘要**
  （`4/4 在线 · 冲突 0`、`m5 · 已下发 2 台`、`37 条` 等），所以收起不等于丢信息。

### 为什么使能/kp/kd 不走参数面

`PARAM_FIELD_ENABLE / KP / KD` 确实是固件 `param_field_t` 的头三项，但**没进
`kParamMotorFields[]` 注册表**（每电机只注册 5 项限值类），而参数面总容量
`PARAM_BRIDGE_MAX_PARAMS = 64`——再加 3 项会把可支持台数从 ~11 压到 ~8。
策略话题这条路还带「代数严格前进」的防复权门。**固件不回读 kp/kd**，所以表格里
显示的是页面最后一次下发的值，界面上已注明。

### 臂视图

three.js 0.160.1 + urdf-loader 0.13.1，vendor 在 `web/vendor/`（847 KB），
用浏览器**原生 import map** 解析——`web/` 依然只有静态文件、无 Node 构建链。
vendor 是整体：urdf-loader 顶层就 import STLLoader 与 ColladaLoader（后者又拉
TGALoader），换版本要整体换。

几何（URDF + 约 20 MB STL）不放 `web/`：`setup.py` 的 data_files 不能收目录，
symlink-install 也处理不了**目录**符号链接。改由 `static_server` 把
`robot_arm_gazebo` 的 share 挂到 `/robot_arm_gazebo`，URDF 里的
`package://robot_arm_gazebo/...` 因此原样可用。

两个踩过的坑：`os.walk` 默认**不下钻目录符号链接**，而 colcon 在 `build/` 里构建时
`web/vendor` 正是指回 src 的链接——整个 vendor 被静默跳过，构建"成功"但页面 404；
以及 URDF 是 **Z-up**、three.js 默认 Y-up（urdf-loader 只转图元、不转网格），
不设 `Object3D.DEFAULT_UP` 整条臂会躺着。

### 明确写在界面上的两条限制

1. 幽灵与「跟随误差」用的是**页面自己最后一次发出去的目标**，不是固件设定点——
   固件侧的钳位、软停、整包拒收画不出来；轨迹在播期间（`target_source == 2`）
   目标由固件插值，页面无从得知，此时宁可不画。
2. 臂视图与曲线都受 `motion_states` 的 50 Hz 限制。

### 本机未能目视验证

这台机器只有 snap Firefox（拒绝独立 profile），Chrome 未安装。**WebGL 实际渲染与
交互未经目视确认**；已静态验证的是：8 个模块全部解析（把 import map 应用一遍逐个
HTTP 检查）、76/76 DOM id 契约、`node --check` 语法、无死引用。

## 未决

| # | 事项 | 现状 |
|---|---|---|
| 1 | **MIT 帧 ~39.4 Hz vs 目标 200 Hz** | 已测准、已可复现；成因未查。控制边界按 5 ms 跑但没有变成总线上的 200 Hz |
| 2 | m1–m4 `CONFIG_CONFLICT` | 固件故意拒绝使能；恢复路径是参数页「读取」→ 写回驱动板限值。本轮用 m5–m8 绕开 |
| 3 | Web 渲染目视验证 | 需在有 Chrome 的机器上开 `http://<host>:8080` 确认 |
| 4 | 目标不变时 MIT 速率掉到 ~1 帧/s | 已观测两次；是设计取舍还是缺陷未定 |

## 变更清单

```
micro_ros_agent/src/can_bridge_msgs/msg/MotorDiagnostics.msg   +5    tx_frame_count 字段
micro_ros_agent/tools/fdcan_motor_sim.py                       +53/-4 VAXOR 回显 12B、EAGAIN 兜底
micro_ros_firmware/include/motor_rt_task.h                     +1    tx_frame_count 快照字段
micro_ros_firmware/lib/fdcan/{include/can_bsp.h,src/can_bsp.c} +14   每总线发送帧计数
micro_ros_firmware/src/motor_rt_task.cpp                       +1    填充发送帧计数
micro_ros_firmware/src/motor_bridge.cpp                        +28   序列接线 + 启动期守卫
ros2_ws/src/web_host/web/{index.html,app.js,style.css}         大幅  以臂为中心的单页
ros2_ws/src/web_host/web/arm3d.js                             新增  URDF → three.js 臂视图
ros2_ws/src/web_host/web/vendor/                              新增  three 0.160.1 + urdf-loader 0.13.1
ros2_ws/src/web_host/web_host/static_server.py                 +33   模型目录挂载
ros2_ws/src/web_host/{launch/web_host.launch.py,package.xml,setup.py} 挂载参数 / 依赖 / data_files 逐目录枚举
ros2_ws/src/web_host/README.md                                 重写  页面对应章节 + 臂视图一节
```

## 复现要点

```bash
# 台架（顺序不可颠倒：ACK 节点在先）
cd micro_ros_agent && python3 -u tools/fdcan_motor_sim.py \
  --cerb 5,6 --vaxor 7,8 --gazebo \
  --joint-map 5:joint_link1,6:joint_link2,7:joint_link3,8:joint_link4 \
  --log /tmp/sim_rx.jsonl
~/.platformio/packages/tool-openocd/bin/openocd \
  -s ~/.platformio/packages/tool-openocd/openocd/scripts \
  -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset run; exit"

# 对账：模拟器逐帧日志 vs 固件诊断
#   模拟器侧按 tag 计数、用日志自带 monotonic 时间戳算速率
#   固件侧用诊断消息自带的 sync_time_ns 计时（**不要**用外部 date 打点：
#   ros2 topic echo --once 的启动延迟 1–3 s 会被算进间隔，速率系统性偏低）
```

仿真绝不用来验收真机行为；本报告所有"运动"结论都来自模拟器与 Gazebo。
