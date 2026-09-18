# micro_ros_firmware 文档

固件**开发资料**按文件夹归类；厂商参考资料平铺在本目录根部。台架活动计划与
实验报告在根工程 [docs/reports](../../docs/reports/README.md)，台架操作
指南在 [docs/guides](../../docs/guides/README.md)。

## 目录职责

- `adr/`：固件架构决策（含勘误），不存操作步骤；
- `architecture/`：架构开发基线（decision-log 承载设计理由）；
- `plans/active/`：未完成真机验收项（唯一开放清单）；
- `archive/`：冻结的完整规范与历史交接，保留追溯、不再维护；
- 根部平铺文件：厂商板卡与电机协议原件，不改内文；
- `dm-mc02例程/`：厂商例程整树（仅本地参考，已 gitignore，勿入库）。

## 开发文档

| 文档 | 内容 | 状态 |
|---|---|---|
| [README](../../README.md) | 工程事实：硬件/时钟/任务/模块/安全语义 | 权威 |
| [AGENTS.md](../../AGENTS.md) | AI/人工操作手册：构建、台架顺序、坑 | 权威 |
| [ADR 0007](adr/0007-micro-ros-discovery-driven-motor-runtime.md) | 发现驱动聚合式运行时决策（顶部含 2026-09-18 勘误） | Proposed + 勘误 |
| [motor-runtime-decision-log](architecture/motor-runtime-decision-log.md) | 设计理由（参数链/一问一答/设零等） | 基线，含历史 |
| [motor-runtime-validation](plans/active/motor-runtime-validation.md) | 开放台架项唯一清单与验收记录要求 | 活动 |
| [w25-blob-v3-design](plans/active/w25-blob-v3-design.md) | W25 参数 blob v3 布局（每槽 2 扇区、设备侧事实、迁移链、零点代被否的理由） | 设计已定，待实现 |
| [w25-blob-v3-plan](plans/active/w25-blob-v3-plan.md) | 上者的实现计划：8 个任务，host 可测部分在前、台架部分在后 | 待执行 |
| [ARD-0007 归档](archive/ARD-0007-micro-ros-motor-runtime.md) | 冻结的 55 章完整规范（10ms/旧话题等已被取代） | 历史 |
| [HANDOFF 归档](archive/HANDOFF-motor-runtime-refactor.md) | 2026-09 重构交接快照（开放项已并入 validation plan） | 历史 |

## 厂商参考资料（原件不改内文）

- 控制板：[CtrBoard-H7 V1.0 原理图](CtrBoard-H7_V1.0-240124.pdf)、[H7 管脚标注图](H7_管脚标注图.png)
- VAXOR / 达妙：[16–30 mm 外置驱动器完整协议](16-30mm外置驱动器使用说明手册_20260720%28完整协议%29.pdf)、[25–30 mm 电机参数规格](25-30mm电机参数规格书.pdf)、[DM-MC-Board02 开发板说明书](达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf)
- CERB / 巨蟹：[关节模组使用协议简易说明书](巨蟹智能-关节模组使用协议简易说明书.pdf)、
  [CANopen-V0.9 字典表](CERB巨蟹智能-CANopen-V0.9字典表.xlsx)（4 个 sheet：字典表 /
  字典操作示例 / SDO协议内容详解 / CANFD 自定义 PDO 控制协议）、
  [R48-101-Lite 参数规格](CE-RB-R48-101-Lite-CO-I-参数规格.pdf)、
  [R58-101-Lite 参数规格](CE-RB-R58-101-Lite-CO-I-参数规格.pdf)

协议文档中的电机轴/输出轴、角度、转速、电流和力矩单位不可直接渗透到上层；
统一换算边界在 `lib/motor_protocol` 和型号 Profile。实现代码必须把原始协议
单位集中转换为工程统一单位。
