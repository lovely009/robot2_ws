# 固件控制安全与可观测性整改设计

日期：2026-08-22

状态：已讨论并批准，待实施计划

目标分支：`dev`；无负载真机验证在 `dev-real`

## 1. 背景

`micro_ros_firmware` 是 STM32H723VGT6 固件，使用 STM32duino、FreeRTOS、
micro-ROS、BMI088 和三路 FDCAN。当前实现可以构建并完成基本联调，但存在以下
结构性问题：

- ROS 订阅回调直接访问 FDCAN，控制执行时间受 micro-ROS executor 调度影响；
- 电机命令没有 freshness timeout，Agent 断连后不会主动进入安全状态；
- CAN1 只保存最新一帧，反馈频率高于 ROS 轮询频率时会确定性覆盖旧帧；
- IMU 样本跨任务读写没有一致性保护；
- 缺少统一任务心跳、故障位和关键错误计数；
- 控制状态机与 HAL、ROS 桥接耦合，难以通过 host-side 测试验证。

本设计面向开发调试阶段，优先建立确定、可测试、可观察的控制闭环。它不宣称
达到带载生产安全等级，也不代替硬件急停、机械限位或真机标定。

## 2. 已确认约束

| 项目 | 约束 |
|---|---|
| MCU | STM32H723VGT6，Cortex-M7，当前 SYSCLK 480 MHz |
| 存储 | 320 KB RAM，1 MB Flash |
| RTOS | STM32FreeRTOS |
| ROS | ROS 2 Humble，micro-ROS serial over USB CDC |
| 控制命令 | 正常生产路径使用 MIT，100 Hz，周期/deadline 10 ms |
| 调试模式 | position-only 和 velocity-only 模式继续保留 |
| Timeout | 100 ms soft-stop，300 ms hard-stop |
| 电机 | 当前两个默认测试电机：RP + vaxor |
| 机械约束 | 关节没有机械抱闸，失能后可能因重力运动 |
| CAN | 控制侧保留有界完整帧流；ROS 侧发布每关节最新状态 |
| 验证能力 | 自动构建/host测试 + 烧录、串口、无负载 CAN 测试；暂无台架 |
| 分支 | 协作开发在 `dev`；真机相关验证和适配在 `dev-real` |

## 3. 范围

### 3.1 本轮包含

- 把 ROS 命令接收与 FDCAN 硬件发送解耦；
- 新增可 host 测试的纯逻辑电机控制模块；
- 建立每关节命令 freshness、soft-stop、hard-stop 和恢复状态机；
- 将 RP 电机命令按总线聚合为一帧 64 B 广播，独立维护各 slot；
- 将 CAN RX 单帧快照改为静态有界队列；
- 修复 IMU 跨任务样本一致性，BMI088 故障时允许降级调试；
- 新增 software supervisor、任务心跳、fault bitmask 和关键计数；
- 增加协议、状态机、命令校验、RP 聚合和队列测试；
- 更新固件 README 和 AGENTS 文档。

### 3.2 本轮不包含

- 完整电机拓扑动态配置；
- 完整 diagnostics topic 或自定义诊断消息；
- `dev`/`prod` 两套 PlatformIO 环境；
- rclc executor 或 Agent 的完整 host 仿真；
- 带载机械臂安全认证或性能结论；
- 自动真机烧录或带载测试；
- 硬件 IWDG 启用。IWDG 留待 `dev-real` 基于同一健康判定验证。

## 4. 方案选择

采用“纯逻辑控制内核 + RTOS/HAL Adapter”方案。

不采用把逻辑继续堆入 `control_task.cpp` 的最小补丁方案，因为它无法自然测试
timeout、故障恢复和 RP 聚合。不采用统一事件总线方案，因为当前两台测试电机不
需要该复杂度。

## 5. 总体架构

```text
micro_ros task, P2
  JointState callback
    -> 名称/finite/数组边界校验
    -> 每关节 overwrite mailbox
    -> 不访问 HAL，不编码协议

control task, P3, 100 Hz
  -> 复制 mailbox 快照
  -> motor_control 状态机
  -> RP 每总线聚合一帧 / vaxor 单电机帧
  -> 非阻塞写 FDCAN TX FIFO
  -> 消费 CAN RX 队列并更新最新关节状态

FDCAN ISR
  -> HAL 取帧
  -> 静态 FreeRTOS Queue
  -> 必要时唤醒 control task

sensor task, P4
  -> BMI088 初始化/重试
  -> DRDY 采样、Mahony、温控
  -> 发布一致的双缓冲快照

supervisor task, P1, 100 ms
  -> 检查任务 heartbeat
  -> 更新 fault bitmask 和计数
  -> dev 阶段不复位、不喂 IWDG
```

## 6. 模块设计

### 6.1 `motor_control` 纯逻辑模块

新增 `micro_ros_firmware/lib/motor_control/`。该模块不依赖 Arduino、FreeRTOS、
HAL 或 rclc；只使用固定容量数据结构，不进行动态内存分配。

建议接口表达以下能力，最终函数名可以在实施计划中按现有 C 风格微调：

```c
motor_control_init(config, motors, count);
motor_control_accept_command(command, now_ms);
motor_control_step(now_ms, outputs);
motor_control_accept_feedback(can_frame, now_ms);
motor_control_set_authorization(enabled);
motor_control_set_fault(fault, active);
motor_control_get_joint_states(...);
motor_control_get_diagnostics(...);
```

模块隐藏每关节命令年龄、状态、sequence、RP slot 聚合、vaxor 模式切换、恢复
授权及诊断计数。调用方只提交输入和消费确定的输出动作。

### 6.2 `control_task` 硬件 Adapter

`control_task` 使用 `vTaskDelayUntil()` 以 10 ms 周期运行，并成为正常控制路径中
唯一的 FDCAN 发送方和 CAN RX 队列消费者。ROS 回调不再调用
`control_motor_command()` 直接发送。

每个周期：

1. 复制所有有更新的命令 mailbox；
2. 将命令提交给 `motor_control`；
3. 消费各 CAN RX 队列，提交反馈并更新最新关节状态；
4. 执行一次 `motor_control_step()`；
5. 每条 RP 总线最多发送一帧聚合命令；
6. 为需要更新的 vaxor 电机发送命令；
7. 在状态切换时发送必要的 enable/disable 控制帧；
8. 更新 control heartbeat 和发送诊断计数。

`HAL_FDCAN_AddMessageToTxFifoQ()` 失败只增加计数，不在控制任务中阻塞重试。

### 6.3 `motor_bridge` ROS Adapter

`motor_bridge` 继续管理 `/joint_commands` 和 `micro_ros/JointState`，新增
`micro_ros/motor_enable`（`std_msgs/Bool`）。

- `true`：在严重故障条件已消失后清除锁存并授权新命令重新使能；
- `false`：请求总线级 safe-stop，并锁存 manual-disable；
- `true` 不直接运行电机，也不重放旧命令。

命令 mailbox 使用固定容量静态内存。写入和读取快照使用短 critical section，
临界区内只复制结构，不解析协议、不调用 ROS、不访问 HAL。

JointState 可以只包含部分关节。每个关节独立更新时间戳；未出现的关节保持原命令
及原时间戳，并按自身 freshness 进入 timeout。

### 6.4 CAN RX 队列

使用 `xQueueCreateStatic()`：

| 总线 | 初始容量 | 满队列策略 |
|---|---:|---|
| CAN1 | 32 帧 | 丢最旧帧，保留最新帧，增加 overflow 计数 |
| CAN2 | 8 帧 | 同上 |
| CAN3 | 8 帧 | 同上 |

ISR 只完成取帧和入队。协议解析、状态更新和 ROS 发布均在任务上下文完成。

当前 FDCAN IRQ priority 为 0，不能调用 FreeRTOS `FromISR` 接口。实施时必须将
FDCAN IRQ 设置为满足 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 的 library priority，
初始采用 5，并通过当前 STM32FreeRTOS 配置和 `configASSERT` 验证。

队列分别记录 `rx_count`、`overflow_count` 和 `hal_receive_error_count`。开发阶段
overflow 只报告和计数，不自动停机。

### 6.5 IMU 快照与降级

BMI088 是板载设备，但初始化失败在开发阶段不阻止 CAN 和 micro-ROS 调试：

- 初始化失败后每 1 s 重试；
- 更新 `imu_init_error_count` 和 IMU health；
- sensor heartbeat 必须继续更新，不能把重试误判为任务卡死；
- 使用双缓冲保存 `imu_data_t`；
- producer 完整写 inactive buffer 后，在短临界区切换 active index；
- ROS 侧先复制完整 active snapshot，再填充和发布消息。

### 6.6 Software supervisor

新增优先级 1、周期 100 ms 的 supervisor task，检查：

- control task 是否按窗口推进；
- sensor task 是否推进，以及 IMU 是否处于降级状态；
- micro-ROS 状态机是否推进；
- FDCAN 是否已初始化及是否报告总线严重故障。

开发阶段只更新 `fault_bitmask`、miss 计数和连续异常次数，不复位 MCU，不启用
硬件 IWDG。Agent 尚未连接是合法状态；状态机停止推进才是 heartbeat 异常。

## 7. 控制状态机

### 7.1 普通命令超时

```text
DISABLED --新鲜有效命令且已授权--> ACTIVE
ACTIVE --age >= 100 ms--> SOFT_STOP
SOFT_STOP --新鲜有效命令--> ACTIVE
SOFT_STOP --age >= 300 ms--> DISABLED
```

SOFT_STOP 使用 MIT 阻尼命令：`kp=0`，保留该电机配置的 `kd`，目标速度和前馈
均为 0。位置字段不产生控制作用。300 ms 后执行协议失能；由于没有机械抱闸，
该动作不被声明为带载安全，必须在未来台架上重新评估。

普通 command timeout 不锁存严重故障。进入 DISABLED 后，下一条新鲜有效命令
可以自动使能，方便开发调试。

### 7.2 严重故障与恢复

以下事件锁存严重故障：

- 曾连接后的 micro-ROS Agent 断连；
- CAN bus-off；
- 电机自身 fault；
- `motor_enable=false` 手动停止。

故障发生时清除旧命令有效性，禁止后续重放。恢复必须满足：

1. 原故障条件已经消失；
2. 收到 `motor_enable=true`；
3. 收到一条故障之后产生的新鲜有效命令。

Agent 从未连接不算故障。FDCAN 尚未初始化时可以更新 mailbox，但禁止发送；完成
初始化后只接受当时仍新鲜的命令。

单台 RP 电机 fault 或 hard-timeout 只改变对应 slot，其他 slot 继续运行。Agent
断连、bus-off、manual-disable 等总线级故障才发送 RP 广播失能。

## 8. RP 聚合语义

RP 的 64 B 命令帧是总线广播，但 8 个 slot 独立对应电机 ID。每个 100 Hz 控制
周期按总线构造一次完整帧：

- ACTIVE：写该电机最新 MIT 命令；
- SOFT_STOP：写该电机阻尼命令；
- DISABLED 或单电机 fault：写该 slot 的 neutral 命令；
- 未配置 slot：写 neutral。

禁止为单台 RP 电机分别发送一帧并把其他 slot 临时清为 neutral，因为这会使同
总线电机在同一周期互相覆盖。

RP enable/disable 是总线级广播，和可独立控制的命令 slot 是不同语义。

## 9. 命令校验

第一版校验：

- 关节名存在、非空且已登记；
- 所有实际提供的 position、velocity、effort 均为 finite；
- 各数组长度独立判断，禁止越界；
- position-only 选择位置调试模式；
- velocity-only 选择速度调试模式；
- 两者同时提供或都未提供时保持 MIT 语义；
- 只使用协议库已有数值限幅，不虚构机械位置、速度或力矩限位。

非法输入只拒绝对应关节，增加 `invalid_command_count`，不更新时间戳，因此坏消息
不能给旧命令续命。同一 JointState 中其他有效关节继续处理。

## 10. 诊断与故障升级

第一版通过 GDB 可读全局结构观察，不新增 diagnostics topic，也不向承载
micro-ROS 的 USB CDC 混入文本日志。

至少记录：

- 每任务 heartbeat/miss；
- valid/invalid command；
- soft-stop/hard-stop/fault/recovery；
- CAN RX/overflow/HAL receive error；
- CAN TX success/failure/连续失败；
- ROS publish return code和失败次数；
- IMU初始化失败次数和health。

单次或连续 CAN TX failure 在开发阶段先计数，不预设停机阈值；无负载测试后再以
测量结果决定是否升级为严重故障。

## 11. 测试设计

### 11.1 Host-side 自动测试

| 测试域 | 验收行为 |
|---|---|
| 协议 | RP/vaxor pack/unpack、极值、非法长度 |
| 校验 | NaN/Inf 拒绝；坏命令不刷新 freshness |
| 独立关节 | 部分 JointState 只更新出现的关节 |
| Timeout | 99 ms ACTIVE；100 ms SOFT_STOP；299 ms SOFT_STOP；300 ms DISABLED |
| 恢复 | 普通超时由新命令恢复；严重故障需要重新授权和新命令 |
| 旧命令 | Agent断连及恢复后不重放 |
| RP聚合 | 更新一个 slot 不清空其他有效 slot |
| RP故障 | 单电机故障只改变自己的 slot |
| 总线故障 | Agent断连/bus-off/manual-disable产生总线级失能动作 |
| 队列 | FIFO顺序、容量、满时丢最旧、overflow计数 |
| IMU快照 | 消费者只能看到某一完整采样 |
| 模式 | MIT正常路径和position/velocity调试路径保持兼容 |

纯逻辑在 host 测试；不为本轮搭建 HAL、FreeRTOS 或 rclc 的完整模拟环境。

### 11.2 固件自动验收

- `~/.platformio/penv/bin/pio run` 成功；
- 没有新增编译 warning；
- 记录 Flash/RAM 修改前后差异；
- 新任务栈初值写入 README；
- 新队列、mailbox 和核心状态使用静态有界存储；
- ROS callback 中不存在 FDCAN HAL 调用；
- control task 使用 `vTaskDelayUntil()`；
- FDCAN IRQ priority 满足 FreeRTOS `FromISR` 约束。

### 11.3 `dev-real` 无负载人工验证

1. 烧录并确认 USB CDC、BMI088 和 FDCAN 初始化；
2. 连续发送 100 Hz MIT 命令，观察 TX 周期和反馈；
3. 停止命令，验证 100 ms 阻尼、300 ms 失能；
4. 断开 Agent，验证旧命令被清除；
5. 重连后确认必须重新授权并发送新命令；
6. 注入非法数值、CAN 无 ACK、RX burst 和 queue overflow；
7. 验证 RP 多 slot 聚合不会互相覆盖；
8. 检查 heartbeat、overflow、invalid、timeout 和 TX failure 计数；
9. 制造 BMI088 初始化异常，确认 CAN 与 micro-ROS 仍可调试。

没有台架和机械抱闸，本轮验收不包含带载安全结论。

## 12. 预计文件范围

- 新增 `micro_ros_firmware/lib/motor_control/`；
- 新增 `micro_ros_firmware/include/supervisor_task.h`；
- 新增 `micro_ros_firmware/src/supervisor_task.cpp`；
- 修改 `main.cpp`、`ros_bridge.cpp/.h`；
- 修改 `motor_bridge.cpp/.h`；
- 重构 `control_task.cpp/.h`；
- 修改 `lib/fdcan/src/can_bsp.c`、`include/can_bsp.h`、`src/fdcan.c`；
- 修改 `sensor_task.cpp/.h`、`imu_bridge.cpp/.h`；
- 新增 host-side 测试入口和测试文件；
- 更新 `micro_ros_firmware/README.md` 和 `AGENTS.md`。

不修改 `ros2_ws` 电机栈，不在本轮实现完整拓扑动态配置。

## 13. 实施顺序

1. 建立 host 测试入口并固定现有协议行为；
2. 以测试驱动实现 `motor_control`；
3. 将 ROS callback 改为 mailbox，将 control task 改为 100 Hz 唯一发送方；
4. 实现 RP 按总线聚合；
5. 将 CAN snapshot 替换为静态队列并修正 IRQ priority；
6. 修复 IMU 快照一致性和初始化降级；
7. 增加 software supervisor 与诊断结构；
8. 更新文档，执行 host 测试和固件构建；
9. 输出 `dev-real` 人工验证记录，不自动执行带载测试。

## 14. 完成定义

本设计的实现完成，是指自动测试和固件构建通过，模块职责及并发接口符合本文，
并已提供无负载真机验证步骤。它不意味着已经完成 `dev-real` 真机验证，也不意味
着系统已具备无抱闸机械臂的带载生产安全性。
