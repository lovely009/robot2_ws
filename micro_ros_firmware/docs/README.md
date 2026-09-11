# micro_ros_firmware 文档

固件**开发资料**按文件夹归类；厂商参考资料平铺在本目录根部。台架活动计划、
交接与实验报告在根工程 [docs/reports](../../docs/reports/README.md)，台架操作
指南在 [docs/guides](../../docs/guides/README.md)。

## 目录职责

- `adr/`：固件架构决策，不存操作步骤；
- `architecture/`：架构需求、设计与开发基线；
- `handoffs/`：当前实现边界、风险和下一步；
- `plans/active/`：未完成架构验收项；
- 根部平铺文件：厂商板卡与电机协议原件，不改内文。

## 开发文档

| 文档 | 内容 | 状态 |
|---|---|---|
| [ARD-0007](architecture/ARD-0007-micro-ros-motor-runtime.md) | 架构需求与设计（55 章规范 + 实现状态矩阵） | Proposed / 开发基线 |
| [ADR 0007](adr/0007-micro-ros-discovery-driven-motor-runtime.md) | 发现驱动的聚合式电机运行时决策 | Proposed |
| [motor-runtime-decision-log](architecture/motor-runtime-decision-log.md) | 架构开发基线（2026-09-04 收敛） | 基线，部分历史 |
| [HANDOFF-motor-runtime-refactor](handoffs/HANDOFF-motor-runtime-refactor.md) | 当前实现边界、剩余上层迁移与台架验收 | 当前 |
| [motor-runtime-validation](plans/active/motor-runtime-validation.md) | ADR 0007 转 accepted 前的架构级开放验收 | 活动 |

## 厂商参考资料（原件不改内文）

- 控制板：[CtrBoard-H7 V1.0 原理图](CtrBoard-H7_V1.0-240124.pdf)、[H7 管脚标注图](H7_管脚标注图.png)
- VAXOR / 达妙：[16–30 mm 外置驱动器完整协议](16-30mm外置驱动器使用说明手册_20260720%28完整协议%29.pdf)、[25–30 mm 电机参数规格](25-30mm电机参数规格书.pdf)、[DM-MC-Board02 开发板说明书](达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf)
- CERB / 巨蟹：[关节模组使用协议简易说明书](巨蟹智能-关节模组使用协议简易说明书.pdf)

协议文档中的电机轴/输出轴、角度、转速、电流和力矩单位不可直接渗透到上层；
统一换算边界在 `lib/motor_protocol` 和型号 Profile。实现代码必须把原始协议
单位集中转换为工程统一单位。
