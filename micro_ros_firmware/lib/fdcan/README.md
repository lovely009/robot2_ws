# fdcan

## 审查结论

**结构和职责符合；测试覆盖集中在纯逻辑层，硬件入口仍需台架验证。** 库没有向协议层泄漏策略，也没有反向依赖任务层。

## 职责与边界

- 三路 FDCAN1/2/3 的 HAL 初始化、GPIO/时钟、过滤器和 IRQ 回调。
- 每总线一个 SPSC RX ring：ISR 是唯一生产者，`motor_rt_task` 是唯一消费者。
- 每总线一个 P0/P1/P2 TX 调度器：P0 安全帧、P1 latest-wins 控制帧、P2 有界管理队列。
- 不做协议解码、型号发现、ROS 发布或安全策略裁决。

## 文件与接口

| 文件 | 作用 |
|---|---|
| `include/fdcan.h` / `src/fdcan.c` | 三路 HAL 句柄、初始化、MSP 和 IRQ |
| `include/can_bsp.h` / `src/can_bsp.c` | 总线初始化、RX/TX 访问和硬件统计 |
| `include/can_rx_ring.h` / `src/can_rx_ring.c` | ISR→任务的固定容量 RX 环 |
| `include/can_tx_scheduler.h` / `src/can_tx_scheduler.c` | P0/P1/P2 软件调度和拒收记账 |
| `library.json` | PlatformIO 元数据 |

关键接口包括 `bsp_can_init()`、`can_bsp_rx_ring()`、`can_bsp_submit_tx()`、`can_rx_ring_push_from_isr()`、`can_rx_ring_pop()` 和 `can_tx_scheduler_pump()`。

## 实时与所有权契约

- RX ring 满时丢新帧并计数，ISR 不回收消费者可能仍在读取的槽位。
- TX 调度器有界，不阻塞、不分配内存；P0 不应被 P2 积压挤占。
- FDCAN ISR 只取硬件帧、盖 MCU tick、写 ring；正常 CAN TX 只有 `motor_rt_task` 提交。
- 板级 ISR 不包含 FreeRTOS，优先级约束来自 `lib/board`。

## 验证与缺口

- `test_can_rx_ring.cpp`：环容量、溢出和批量弹出。
- `test_can_tx_scheduler.cpp`：P0/P1/P2 优先级、latest-wins、配额和硬件拒收记账。
- `test_fdcan_motor_sim.py`：电机运行路径仿真。
- PlatformIO 固件构建通过。

host 未编译 `can_bsp.c`、`fdcan.c` 的 HAL/IRQ 路径；这属于硬件测试缺口，不是目录越界。当前还保留 `TBD-CAN-01` 的 P2 配额和 `TBD-CAN-02` 的过滤器收敛项，不能在文档中标成已冻结规格。

## 后续建议

台架验证应记录 RX FIFO 丢失、TX FIFO 拒收、实际仲裁/数据段时序；若将来需要脱机验证硬件包装层，可注入 HAL seam，不要把 HAL stub 混进纯 ring/scheduler 测试。
