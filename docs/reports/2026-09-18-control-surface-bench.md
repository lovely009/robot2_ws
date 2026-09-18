# 控制面重构台架记录（2026-09-18）

> 环境：STM32H723VGT6 + ST-LINK/V2 烧录 + CANable2 SLCAN（`/dev/ttyACM1`）+
> 九节点模拟器（CERB 1–6 + VAXOR 7–9）；micro-ROS agent 走 `/dev/ttyACM0`。
> 本次只做到「发现 + 只读观察 + 参数写」；使能/轨迹/运动测试未做（中途暂停）。

## 固件与环境

- `pio run`：RAM 220,308 B / 577,536 B（PlatformIO 口径 38.1%）、Flash 242,044 B（23.1%）。
  **RAM_D1 口径**（`.data+.bss` 全部落在 320 KiB 上）≈ **67%**。
- 九节点全部发现、型号绑定正确：1 = `cerb_r58_101`、2–6 = `cerb_r48_101`、
  7–9 = `vaxor_2550`；`bus_index` 全为 1。
- 发现完成后稳态：`decode_error=0`、`config_rejected=0`、`event_dropped=0`、
  `tx_dropped=0`、三条 `ring_drop` 全 0；收帧速率 ≈ 452 帧/秒（9 节点 × 50Hz）。

## 新接口的台架验证（只读面，全部通过）

| 项 | 结果 |
|---|---|
| `micro_ros/diagnostics`（新） | 1Hz 正常发布；`heap_free` 与全部计数器可读 |
| `micro_ros/trajectory_state`（新） | 正常发布；空闲态 `trajectory_id=0 / active=false / target_count=0` |
| `MotorTopology.schema_version` | `2` ✓ |
| `MotorMotionStateTarget.target_source` | 字段存在，未下命令时为 `0` ✓ |
| 上行三分去重 | `state_age_ms` 已不在 status；`lifecycle` 在 status ✓ |
| 参数面退役 | `ros2 param list` 只剩 `m<id>.pos_min/pos_max/vel_limit/torque_limit`，`enable`/`kp`/`kd` 已不存在 ✓ |
| `/motor_policy`（新） | 话题已建（publisher 0 / subscription 1）✓ |

## 发现 1（严重，已修）：凡是 W25 落盘，板子必然复位

**现象**：任何触发落盘的操作都把板子打重启。判据用 `/micro_ros/diagnostics` 的
`rx_frame_count`——它是单调计数，写入后从数万掉到几百即说明固件重启。

**定位过程**（三条现象收敛到同一点）：

| 操作 | 触发 W25 落盘 | 现象 |
|---|---|---|
| `m1.pos_min`（位置双限**成对暂存**，事务未发起） | 否 | 正常 |
| `system.refresh`（设备只读回读，不写记录） | 否 | 正常 |
| `m1.vel_limit` / `m5.vel_limit`（写成功 → `record_dirty` 去抖落盘） | **是** | **复位** |
| `system.save`（纯全局布尔，直接落盘） | **是** | **复位** |

**根因**：`src/storage_task.cpp` 的 `perform_save()` 里

```c
motor_control_param_view_t views[MOTOR_CONTROL_MAX_MOTORS];  // 64 × 80 B = 5,120 B
```

是**栈上**局部数组，而该任务栈只有 `kStorageTaskStack = 1024` words = **4,096 B**
（注释自称"4KB（HAL OSPI AutoPolling 深栈）"）。**超出至少 1,024 B**，每次落盘必然溢出。

溢出的后果由 `src/main.cpp` 的钩子决定：

```c
void vApplicationStackOverflowHook(TaskHandle_t, char *pcTaskName) {
  g_stack_overflow_task = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) { }        // 关中断死循环
}
```

`motor_rt` 是唯一喂 IWDG 的任务（超时 500ms），中断一关就没人喂狗 → **约 500ms 后看门狗复位**。
所以表观现象是"一落盘就重启"，而不是任何显式报错。

**归属**：这是**既有缺陷，不是本次重构引入的**——`git diff` 证明 `src/storage_task.cpp`
与 `motor_control_param_view_t` 在本会话均未被修改。

**修复**：该数组改为 `static`（与同文件 `g_write_buf`/`g_verify_buf` 同类做法；`perform_save`
只由 `storage_task_fn` 调用，本任务独占该缓冲）。修复后 RAM 增加 5,120 B——正好是数组从栈
挪到 `.bss` 的量。**已验证 `system.save` 不再复位**（`rx_frame_count` 14,526 → 15,879 正常增长）。

**影响面（修复前）**：`system.save`、设备参数写回、设零、清错——**所有会落盘的操作在真机上都会复位**。

## 发现 2（严重，已修）：参数槽预算把 DTCM 堆吃到只剩 1.8 KB

`PARAM_BRIDGE_MAX_PARAMS` 原按计划设 208（覆盖 48 台）。台架实测 `system.heap_free`：

| `MAX_PARAMS` | `heap_free` |
|---|---|
| 208 | **1,776 B** |
| 176 | **12,272 B** |

→ **实测每槽 328 B**（(12,272−1,776)/32）。比按结构估算的 269 B 高约 59 B，多出的是
分配器块开销——容量审计当时把它标为"未确认"，这次量到了。计划里"208 槽 ≈ 56.6 KB、
池内剩约 26 KB"的估算**低估了约 21.5 KB**。

已降为 **176 槽**（覆盖 (176−10)/4 = 41 台，实际部署约 40 台）。**再往上调之前必须先在
台架上量 `heap_free` 的稳态最低位**（该口径已写进 `include/param_bridge.h` 的注释）。

## 发现 3：motor_id 1–4 处于 DEV_PARAM 冲突

`unified_fault_code = 6`（`MOTOR_CONTROL_FAULT_DEV_PARAM`）、`lifecycle = 7`
（`CONFIG_CONFLICT`），即 W25 记录与设备回读不一致——文档过的台架常态；5–9 为
`lifecycle = 3`（READY_DISABLED）、故障码 0。

按文档的修复法写回四条静态量：**1 号已修好**（写完后 `conflict: false`）。
**2–4 号未处理**（中途暂停）。

> 注：`system.last_error` 是**最后写入者胜**的单值对，会被后续的 OK 覆盖成 `(0, id)`，
> 排查多台冲突时不能只看它——要结合 `MotorStatusTarget.unified_fault_code`。

## 未做

- **使能 / 轨迹 / 运动测试全部未做**（用户已授权但暂停）：策略通道的代数门与稀疏语义、
  轨迹下发与插值执行、发快照使其运动、`event_seq`（稳态无事件，未观察到跳号行为）。
- motor_id 2–4 的 DEV_PARAM 冲突未修。
- 发现 1 的修复**只用 `system.save` 验证过**；设备参数写回同样触发落盘，1 号写回成功并
  脱离冲突是其间接旁证，但没有单独做"写→落盘→复位"的对照实验。

## 复现命令

```bash
# 烧录（必须先停模拟器：MCU 挂起期间无人 ACK，会把 CANable 打进错误态）
~/.platformio/penv/bin/pio run -t upload
# 模拟器（先于复位控制板成为 ACK 节点）
python3 micro_ros_agent/tools/fdcan_motor_sim.py --tty /dev/ttyACM1 --log /tmp/sim.jsonl
# 观察
ros2 topic echo --once /micro_ros/diagnostics     # heap_free 与全部计数器
ros2 param set /micro_ros_node system.save true   # 修复前必然复位；修复后正常
```
