# 电机运行时交接

> 固件侧 ID 驱动运行时已经完成。本文只记录当前实现边界、剩余上层迁移与
> 台架验收项；设计依据以 [ARD-0007](../architecture/ARD-0007-micro-ros-motor-runtime.md)、
> [ADR 0007](../adr/0007-micro-ros-discovery-driven-motor-runtime.md) 和
> [架构开发基线](../architecture/motor-runtime-decision-log.md) 为准。开放架构验收统一维护在
> [运行时验证计划](../plans/active/motor-runtime-validation.md)。

## 当前状态

固件不保存安装电机表或 `joint_name`。设备由反馈帧发现，以全局唯一
`uint16_t motor_id` 寻址；型号由上层绑定 Profile，命令使用最多 64 个目标的
完整 ID 快照。

`motor_rt_task` 已整体替代旧 `control_task`，并拥有以下资源：

- FDCAN RX ring 的唯一消费者；
- `motor_control` 运行时状态的唯一写者；
- 正常 CAN TX 的唯一所有者；
- 每 1ms 执行 RX drain、内核服务、SYNC、TX 路由与 pump；
- 每 10ms 处理会话/授权、完整命令快照、配置 FIFO 与绑定 FIFO。

ROS 桥只通过 mailbox、只读状态副本和事件中继交互，不接触厂家协议字节或
原始 CAN RX。host CTest 当前包含 12 个目标；固件目标代码改动合入前还必须执行
PlatformIO build。

## 协议与运行时事实

两种协议均将命令回显与总线心跳分开：

- vaxor：12B reply 帧（`0x701 + 4n`）回显控制状态；20B status 帧
  （`0x702 + 4n`）约每 20ms 自主上报并承担在线检测。
- CERB：`0x300 + node` 反馈帧回显控制状态；`0x080` SYNC 维持约 20ms
  状态上报。SYNC 每 10ms 发送，只用于发现、重连和心跳，不作为 100Hz 控制触发。
- 两种协议的 OFFLINE 阈值均为心跳丢失 100ms。

每总线 TX 调度分为：

- P0：安全、使能、软停、失能；不得静默丢弃，队列满或硬件拒收时回告内核重试；
- P1：100Hz MIT，按运行时槽位 latest-wins；
- P2：发现、配置和管理，有界配额，不得挤占 P0/P1。

逐电机 `motion_authorized` 是真实门禁。离线、故障、冲突、全局撤权或
`SET_ENABLE(false)` 会清除授权；恢复必须显式 `SET_ENABLE(true)`，新命令快照
不能自动恢复授权。SET_MODE 事务结束后也必须重新执行使能序列。

快照安全规则：

- 缺席的受控电机立即进入阻尼软停；
- 超过 30ms 没有新快照时全部受控电机软停；
- 超过 100ms 时协议级失能并清授权；
- Agent 断链立即执行失能路径，不等待 100ms；
- NaN/Inf、未知或重复 `motor_id` 使整份快照被拒绝，且不刷新 freshness。

## 资源所有权

- FDCAN ISR 只盖单调时戳并写每总线 SPSC RX ring；环满丢新帧并计数，ISR
  不解码也不回收消费者可能正在读取的槽位。
- `lib/dma` 管理 DMA1、BDMA/DMAMUX2 控制器级时钟、NVIC、占用登记及
  cache/MPU 约束；设备句柄和传输配置仍归设备库。
- BMI088 使用阻塞 SPI2，不占用 DMA；设备层须在 `HAL_SPI_Init` 前显式调用
  `imu_spi2_board_init()`。
- WS2812 使用 SPI6 + BDMA Channel0；DMA 缓冲位于 `.bdma_noinit/RAM_D3`。
  BDMA 完成缓冲搬运后，仍需 SPI6 EOT IRQ 将 HAL 状态恢复为 READY。
- W25Q64 当前不执行启动探测；后续运行期访问须有独立低优先级所有者，不得阻塞
  1ms/10ms 电机路径。

## 剩余工作

### 上层迁移

`motor_hardware_interface` 仍需维护 `joint_name ↔ motor_id`、方向、零位和关节语义，
发送 `MotorCommandArray` 并接收 `MotorMotionStateArray`。如需调整消息定义，先修改
同仓库 `../micro_ros_agent/src/can_bridge_msgs` 的权威消息包，再同步 ROS 工作区；
不要在固件中恢复 name-to-ID 兼容层。

### 未闭环项

- CERB 设备侧限位管理对象尚未确认，目前请求返回 `UNSUPPORTED`；
- Vaxor 80 速比型号缺少完整 Profile，不得凭猜测开放控制；
- FDCAN BUS_OFF 当前可观测但恢复策略须依据台架数据确定；
- 最大规模三总线、约 40 台设备时的 USB CDC 带宽、RX 容量和实时余量尚待实测；
- `motor_rt_task` 的按名称绑定转发采用尽力而为；失败已有结果事件，上层负责重试策略。

## 自动验证

在 `micro_ros_firmware` 目录执行：

```bash
cmake -E remove_directory .host-build
cmake -S test/host -B .host-build
cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
git -C .. diff --check
```

修改 `platformio.ini` 编译宏后才需要先执行 `pio run -t clean`，避免 HAL SrcWrapper 沿用旧对象。常规验证
不得运行 `clean_microros`，以免联网重建依赖。自动测试不得启动 Gazebo、烧录固件
或驱动真实电机。

## 真机验收

只可在明确授权的真机验收环境中、机构无负载、硬件急停和供电隔离均已验证时人工执行：

1. 记录三总线约 40 台最坏反馈流量下的 RX overflow/backlog、P0/P1 延迟、
   motor_rt WCET、周期 jitter、总线负载和 USB CDC 上行余量。
2. 实测完整快照 30ms 软停、100ms 失能，以及单电机从快照缺席时的隔离行为。
3. 断开并恢复 Agent，确认失能清授权，且恢复需要全局授权、逐电机 Enable 和新快照。
4. 注入未知/重复 ID、NaN/Inf、非法 mode、厂家故障、心跳丢失和跨总线 ID 冲突，
   检查拒绝、锁存、P0 失能和结果事件。
5. 制造 CAN 无 ACK、RX burst 和 BUS_OFF，记录调度器及 ring 计数，验证 P0 不静默丢失。
6. 验证 CERB 每总线 10ms 单帧 SYNC、约 20ms 状态上报和 100ms OFFLINE 边界。
7. 断开 BMI088、阻塞 SPI 或停止 DRDY，验证传输超时、降级重试、heater off 与
   双缓冲不撕裂。

软件停机不能替代硬件急停、机械限位、供电隔离和真机标定。取得上述台架证据前，
ADR 0007 保持 Proposed。
