# CERB Lite CAN FD / MIT 协议摘要

本文记录 `motor_drive` 使用的协议子集。设备侧帧格式与缩放依据见
[设备侧参考 §6](../../../../docs/research/cerb-device-reference.md#6-控制报文入口)，
以厂家《CANFD MIT 快控档位与缩放使用指南》为准。此控制路径不通过读写寄存器设置 MIT 目标或量程。

## 通讯和帧格式

- 标准 CAN FD 帧，BRS；仲裁段 1 Mbit/s、数据段 5 Mbit/s。
- 通用节点范围为 1–127；单轴快控限 1–15，ID 必须是 `0x110 | Dev_ID`，不能用加法。
- 单轴发送 12 Byte（DLC=9），前 9 Byte 有效，尾部 3 Byte 补零。
- 多轴使用 `0x210`、64 Byte（DLC=15），最多六个 9 Byte 槽位；NodeID 表位于 Byte56–61。
  主机驱动的高节点使用单槽多轴帧；固件 P1 路径可将同总线命令合并为六槽帧。
- 状态反馈 ID 为 `0x300 + Dev_ID`、12 Byte；其位置/速度/Iq 刻度独立于 MIT 命令量程。

MIT 控制字 bit7 为使能、bit6 为抱闸释放、bit5 为一次性清错事件，bit4:1 为模式
（MIT=`0x06`）。普通使能命令使用 `0xCC`，失能命令使用 `0x0C`，不持续置清错位。
Byte1–8 按大端位序打包 `P(16) + V(12) + Kp(12) + Kd(12) + T(12)`。
物理量先按配置范围饱和，再四舍五入到对应 raw；五个 raw 全零为无效命令，不得用作中性目标。

反馈 Byte0–5 分别为大端 `int16`：输出端位置（182 count/deg）、电机端速度（rpm）
和 Iq（20 mA/码，见设备侧参考 §10.5）。策略层将速度除以减速比，
电流按 `Iq × Kt × gear_ratio × gear_efficiency` 换算为输出端转矩。

## 型号参数与安全边界

型号库唯一来源是
[`motor_hardware_interface/config/motor.yaml`](../../motor_hardware_interface/config/motor.yaml)。
CERB 的五项 MIT 线协议满量程分别由 `mit_position_limit_rad`、`mit_velocity_limit_rad_s`、
`mit_kp_limit`、`mit_kd_limit`、`mit_torque_limit_nm` 提供；必须为有限正数。
拓扑中单电机的 `parameters` 可覆盖型号默认值，但必须与该台设备的实际设置一致。

线协议满量程不是物理保护阈值：`peak_torque_nm`、`peak_current_amp` 等仍独立限制出力；
固件的软件 Kp/Kd 上限也不会因为量化范围扩大而自动放宽。VAXOR 不使用这些 CERB 缩放参数。

软件使能帧使用零位置、零速度、零增益、零前馈对应的量化中点，不是全零 raw。
真机首次接入仍须逐关节确认零位、方向、量程、机械限位和硬件急停；协议测试通过不等于运动验收。
