# motor_protocol

## 审查结论

**符合协议库定义，host 覆盖完整度最高。** 公共类型、CERB/VAXOR 编解码和 adapter 接缝都集中在本库，未发现 HAL、ROS、任务层或注册表反向依赖。

## 三块结构

| 区域 | 内容 |
|---|---|
| 公共 | SI 侧命令/状态、通用量化、adapter 公共类型和传输帧描述 |
| CERB | CAN-ID、MIT/PDO、CANopen SDO、SYNC、设备参数换算和 `motor_protocol_cerb_adapter` |
| VAXOR | 25/30 mm MIT、控制/反馈/状态帧和 `motor_protocol_vaxor_adapter` |

## 文件与接口

| 文件 | 作用 |
|---|---|
| `include/motor_protocol.h` | 公共 codec、ID 推导、CERB/VAXOR 线协议结构 |
| `include/motor_protocol_adapter.h` | 内核注入的协议 adapter 接口、Profile 和配置阶段契约 |
| `src/motor_protocol.c` | 公共/CERB/VAXOR 基础 codec 与 ID 函数 |
| `src/cerb_adapter.c` | CERB 帧归属、解码、MIT、停机、SYNC、SDO 阶段 |
| `src/vaxor_adapter.c` | VAXOR 帧归属、解码、MIT、停机和配置阶段 |

内核只调用 `motor_protocol_adapter_t`；新增协议应增加 adapter 和测试，不改 `motor_control` 的收发循环。

## 验证与已知待决项

- `test_motor_protocol.cpp`：基础 codec、ID、边界和单位。
- `test_motor_protocol_adapter.cpp`：两种 adapter 的解码、MIT、停机、SYNC 和配置事务。
- `test_motor_control.cpp`、`test_storage_projection.cpp`：跨库的槽序和 Profile 契约。
- PlatformIO 固件构建通过。

`TBD-CAN-03`（各帧 FD/BRS/DLC 的最终确认）、`TBD-CONFIG-01`（厂家确认对象）和 `TBD-SAFETY-01/03`（软停/离线阈值）仍在代码注释中明确标记；当前实现按占位语义运行，不能在接口文档中当作厂家最终规格。
