# `micro_ros_firmware/lib` 库审查

## 审查范围与方法

审查对象是 `micro_ros_firmware/lib` 下的 14 个 PlatformIO 库，核对：

1. `library.json` 名称、目录和默认/显式构建目录。
2. `include/` 与 `src/` 的公共/私有边界、依赖方向和所有权。
3. 固件实际构建、host CTest 的源文件覆盖和已知缺口。
4. 库定义中写明的实时、静态容量、硬件独占和协议分层约束。

验证基线：`~/.platformio/penv/bin/pio run` 成功；host CMake/CTest 有 22 个测试目标，当前实现测试全部通过。硬件中断、HAL 和 Flash/SPI 电气时序不由 host 测试代替。

## 结论矩阵

| 库 | 目录/元数据 | 边界与所有权 | 测试覆盖 | 结论 |
|---|---|---|---|---|
| `board` | ✅ header-only，PlatformIO 可发现 | ✅ 只提供 ISR 常量 | 无可执行逻辑 | 符合 |
| `control_mailbox` | ✅ | ✅ 纯逻辑、调用方临界区 | ✅ 共享锁并发、NULL、clear 与诊断计数 | 符合 |
| `dma` | ✅ 显式构建与 `board` 依赖 | ✅ 只管控制器级 DMA 资源 | ⚠️ 目标硬件路径 | 符合，需台架 |
| `fdcan` | ✅ 显式构建与 `board` 依赖 | ✅ HAL、SPSC ring、P0/P1/P2 调度分层 | ⚠️ HAL/IRQ 未在 host 编译 | 符合，部分待冻结 |
| `imu` | ✅ 显式构建 | ✅ BMI088/SPI2/融合/snapshot 集中 | ⚠️ SPI/FreeRTOS/EXTI | 符合，需台架 |
| `motor_control` | ✅ 显式构建，依赖 `motor_protocol` | ✅ 纯 C 内核，唯一调用者明确 | ✅ 核心路径覆盖较全 | 符合，含产品 TBD |
| `motor_protocol` | ✅ 显式构建 | ✅ 公共/CERB/VAXOR 三块，厂家字节集中 | ✅ codec 与 adapter | 符合，含协议 TBD |
| `motor_tx_router` | ✅ 显式构建，依赖 `fdcan` 与 `motor_protocol` | ✅ 有界队列选择、物理帧预算和 HAL submit seam | ✅ P0 拒收顺序、P1 聚合、P2 配额 | 符合 |
| `param_store` | ✅ 显式构建 | ✅ 纯布局/CRC/迁移，无 Flash 访问 | ✅ v3/迁移/值域 | 符合，预留字段未实现 |
| `save_transaction` | ✅ 显式构建，无硬件依赖 | ✅ SAVE 阶段与外部端口分离 | ✅ 快照、关联、失败、超时、合并 | 符合 |
| `session_sync` | ✅ 显式构建 | ✅ 纯派生，依赖方向正确 | ✅ 有真实内核联测 | 符合 |
| `storage_projection` | ✅ 显式构建与本地依赖 | ✅ 纯投影，静态断言锁跨库序 | ✅ CERB/VAXOR 投影与空指针边界 | 符合 |
| `w25q64` | ✅ 显式构建 | ✅ OCTOSPI/命令/geometry 分层，storage 独占 | ⚠️ 仅 geometry | 符合，需 Flash 台架 |
| `ws2812` | ✅ 显式构建 | ✅ SPI6/BDMA 物理发送，策略在任务层 | ⚠️ 无 host 波形测试 | 符合，需 LED 台架 |

## 已确认的结构性检查

- 未发现库 include `src/`、ROS/micro-ROS 头或其他任务实现。
- `lib/fdcan`、`lib/dma`、`lib/board` 未 include FreeRTOS；`lib/imu` 的 FreeRTOS/HAL 依赖位于其明确的硬件 middleware，符合其职责。
- `motor_control -> motor_protocol`、`storage_projection -> motor_control + param_store`、`session_sync -> control_mailbox + motor_control` 均为单向依赖，没有反向环。
- PlatformIO 依赖图能发现全部 14 个库；固件构建成功。
- `board` 没有 `src/` 是 header-only 设计，不是缺文件；其余库的源文件都位于声明的 `src/` 目录。

## 需要保留在文档中的问题

### 测试覆盖

1. `fdcan` 的 `can_bsp.c`/`fdcan.c`、`imu` 的 SPI/HAL middleware、`w25q64` 的 OCTOSPI 命令层、`dma` 和 `ws2812` 仍依赖目标板验证。
2. `control_mailbox` 的共享锁并发、NULL 提交、clear 和计数持久性已有 host 用例；目标板上的临界区调度时序仍需台架确认。
3. `storage_projection` 对空指针与容量组合已安全返回并有 host 用例；调用方仍负责提供有效缓冲区及先行 CRC 校验。

### 元数据和待决规格

1. 所有库的 `library.json` 已显式标注构建目录（`board` 为纯头文件库）；直接跨库 include 的依赖已列出。
2. 本地依赖仍需通过 PlatformIO 目标构建验证，避免元数据调整改变链接范围。
3. `param_store` v3 的型号身份、identity hash、MIT 标度和 profile version 是预留字段，当前不应宣称已持久化。
4. `motor_protocol` 的 CAN FD 属性、配置确认对象、软停和离线阈值仍有明确 TBD；`fdcan` 的过滤器和 P2 配额也有待冻结项。

## 整改优先级

| 优先级 | 动作 |
|---|---|
| P0 | 保持现有所有权：FDCAN ISR→RX ring→`motor_rt_task`，Flash 只由 `storage_task`，WS2812 只由 `led_task`。本次未发现需要立即修复的反向依赖。 |
| P1 | 对硬件库建立台架验收记录，并验证 FreeRTOS 临界区及 FDCAN/Flash/SPI 实际时序。 |
| P2 | 关闭协议/调度 TBD 后同步更新对应库 README 和测试。 |

## 最终判断

按当前仓库定义，14 个库的目录归属、主要职责和依赖方向**整体符合**。本次审查发现的是文档缺失、硬件路径测试边界和已明确标注的待决规格，没有发现需要因“库定义违规”立即重构的文件。每个库的具体证据和限制见对应 `lib/*/README.md`。
