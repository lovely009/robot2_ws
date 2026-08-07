### CAN-FD 关节通信协议方案 V1.0

文档版本：V1.0 最后修订：2026.03.30 修订人：刘灼

### 修订记录

序号 修订时间 版本号 修订人 修订说明

1 2026.03.30 V1.0 刘灼 初版协议

### 1. 前言

本文档描述基于 CANFD 接口的电机通信协议，包括基础通信帧、数据定义、故障上报等。

主机采用广播方式单次发送 64 字节，即可完成单条 CANFD 指令最多对 8 个电机的实时控制。

### 1.1 默认配置说明

###  关节方向：正向旋转方向为正视输出轴时的逆时针旋转。

###  关节默认运动范围：±π（即断电重启首次开机随机位置处于 ±180° 范围内）。

### 2. 通信接口

###  接口类型：CANFD

###  默认速率：1Mbps（仲裁域）+ 5Mbps（数据域），采样点 75%

###  帧格式：扩展标识符（29bit）数据帧 / 标准帧，禁用远程帧

###  通信机制：一问一答，应答超时 50ms

###  发送标识符与应答标识符相同

### 扩展帧定义（29bit）

位域 含义

Bit0–Bit6 设备 ID

Bit7 保留

Bit8–Bit14 产品 ID

Bit15 读写成功标志（0=成功，1=异常）

Bit16–Bit23 寄存器地址

Bit24–Bit27 子寄存器地址

Bit28 读写标志（0=读，1=写）

设备 ID 定义：默认地址 0x01，范围 0x01–0x08 产品 ID 定义：默认地址 0x01 数据格式：超过 1 字节的数据以小端格式发送

### 3. 寄存器地址及子地址定义

###  读寄存器：Bit28 = 0

###  写寄存器：Bit28 = 1

###  配置寄存器写入后自动保存，重启生效

###  读寄存器无长度限制，写寄存器必须匹配寄存器长度

###  float 类型采用 IEEE-754 标准

### 3.1 通用寄存器定义（扩展帧，单关节通信）

以下参数修改后具备掉电存储能力。

Pn1：厂家信息（地址 0x01）

子地址 名称 内容 长度 读写 备注

0x00 子寄存器操作 读写所有子寄存器 48Byte 读写

0x01 产品型号 ASCII 字符串 16Byte 读写

0x02 产品序列号 15Byte 读写

Byte 0：Major 主版本号； Byte 1：Minor 次版本号； 0x03 硬件版本信息4Byte 读写 Byte 2：Patch 修订版本号； Byte 3：Reserve 保留；

子地址 名称 内容 长度 读写 备注

Software Version： 'Major.Minor.Patch'；

Byte 0：Major 主版本号； Byte 1：Minor 次版本号； 0x04 软件版本信息Byte 2：Patch 修订版本号；4Byte 读写 Byte 3：Reserve 保留； Software Version：'Major.Minor.Patch'；

Pn2：设备信息（地址 0x02）

子地址 名称 内容 长度 读写 备注

0x00 子寄存器操作 读写所有子寄存器 5Byte 读写

默认设备 ID：0x01；设备 0x01 ID 寄存器 1Byte 读写 重启后生效 ID 地址：0x01 ~ 0x7F；

Byte[0] - CAN FD 仲裁域 通讯波特率设置： 8 位无 符号整型数据，默认值： 0x02 (1Mbps)； 0：125Kbps； 1：500Kbps； 2：1Mbps； Byte[1] - CAN FD 仲裁域 采样点设置： 8 位无符号 整型数据，默认值： 0x01(80%); 0：75.0%； 1：80.0% 0x02 通信参数配置4Byte 读写 重启后生效 2：87.5%； Byte[2] - CAN FD 数据域 通讯波特率设置： 8 位无 符号整型数据，默认值： 0x03(5Mbps); 0 - 125Kbps； 1 - 500Kbps； 2 - 1Mbps； 3 - 5Mbps； Byte[3] - CAN FD 数据域 采样点设置： 8 位无符号 整型数据，默认值：

子地址 名称 内容 长度 读写 备注

0x00(75%); 0：75.0%； 1：80.0% 2：87.5%；

Pn3：MIT 阈值（地址 0x03）

子地址 名称 默认值 长度 读写 备注

0x00 子寄存器惭怍 读写所有子寄存器 48Byte 读写

0x01 位置最小值 Float 类型 默认-10.0f 4Byte 读写

0x02 位置最大值 Float 类型 默认 10.0f 4Byte 读写

0x03 速度最小值 Float 类型 默认-60.0f 4Byte 读写

0x04 速度最大值 Float 类型 默认 60.0f 4Byte 读写

0x05 扭矩最小值 Float 类型 默认-60.0f 4Byte 读写

0x06 扭矩最大值 Float 类型 默认 60.0f 4Byte 读写

0x07 KP 最小值 Float 类型 默认 0.0f 4Byte 读写

0x08 KP 最大值 Float 类型 默认 500.0f 4Byte 读写

0x09 KD 最小值 Float 类型 默认 0.0f 4Byte 读写

0x0A KD 最大值 Float 类型 默认 10.0f 4Byte 读写

Pn4：Level1 保护阈值（地址 0x04）

子地址 名称 默认值 长度 读写 备注

0x00 子寄存器操作 读写所有子寄存器 24Byte 读写 Level1 保 护 阈 值 触 发 后 上 报 0x01 MOS 过温告警阈值 float，如 90℃ 4Byte 读写 故障，电机维持 当 前 运 行 的 状 0x02 MOS 过温恢复阈值 float，如 75℃ 4Byte 读写 态

0x03 过流告警阈值 float 4Byte 读写

子地址 名称 默认值 长度 读写 备注

0x04 过流恢复阈值 float 4Byte 读写

0x05 过压告警阈值 float 4Byte 读写

0x06 过压恢复阈值 float 4Byte 读写

Pn5：Level2 保护阈值（地址 0x05）

子地址 名称 长度 读写 备注

0x00 子寄存器操作 24Byte 读写

0x01 MOS 过温告警阈值 4Byte 读写 Level2 保护阈 0x02MOS过温恢复阈值4Byte读写值触发后上报 故障，电机主动 0x03 过流告警阈值 4Byte 读写停止当前运行 的状态 0x04 过流恢复阈值 4Byte 读写

0x05 过压告警阈值 4Byte 读写

0x06 过压恢复阈值 4Byte 读写

Pn6：位置模式参数配置（地址 0x06）

子地址 名称 类型 长度 读写 备注

0x00 子寄存器操作 读写所有子寄存器 16Byte 读写

0x01 梯形加速度 rad/s² float 类型 4Byte 读写

0x02 梯形匀速速度 rad/s float 4Byte 读写

0x03 梯形减速度 rad/s² float 4Byte 读写

0x04 位置增益 P float 4Byte 读写

Pn7：速度模式参数配置（地址 0x07）

子地址 名称 类型 长度 读写 备注

读写所有子 0x00 子寄存器操作 8Byte 读写 寄存器

子地址 名称 类型 长度 读写 备注

0x01 速度增益 P float 4Byte 读写

0x02 速度积分 I float 4Byte 读写

Pn08：功能设置（地址 0xA0）

子地址 名称 说明 长度 读写 备注

0x00 子寄存器操作 读写所有子寄存器 4Byte 读写

0x01 模式配置 00=MIT，01=PP，02=PV 1Byte 读写

应答： 0x02 零位配置 0x01 = 设置当前位置为零位 1Byte 读写0x00 : 设置异常 0x01 : 设置成功

写 0x0000 清除所有错误 bit0-----使能状态 bit1----- 1 级过压 bit2----- 2 级过压 bit3----- 1 级过流 bit4-----2 级过流 bit5-----MOS1 级过温 bit6-----MOS2 级过温 bit7-----超速 0x03 故障寄存器bit8-----过载2Byte 读写 bit9-----超差 bit10-----堵转 bit11-----预驱故障 bit12-----编码器故障 bit13 ----- 欠压 bit14 ---- 线圈 1 级过温 bit15 ---- 线圈 2 级过温

其他寄存器

地址 子地址 名称 说明 长度 读写 备注

0xB0 0x00 子寄存器操作 读写所有子寄存器 12Byte 读写

地址 子地址 名称 说明 长度 读写 备注

0x01 速度滤波截至频率 float， 4Byte 读写 单位: Hz

0x02 电流限制 float， 4Byte 读写 单位 0.1A

0x03 输出力矩限制 float， 4Byte 读写 单位 Nm

### 3.2 控制寄存器定义（标准帧通信）

0x80：MIT 协议控制，广播帧，单次控制 8 个关节 0x80 + N：经典协议控制，单节点控制，N = 设备 ID 8 个电机 × 8 字节 = 64 字节



主站下发（广播）

字节 功能

Byte0–Byte7 第 1 个关节指令数据

Byte8–Byte15 第 2 个关节指令数据

Byte16–Byte23 第 3 个关节指令数据

Byte24–Byte31 第 4 个关节指令数据

Byte32–Byte39 第 5 个关节指令数据

Byte40–Byte47 第 6 个关节指令数据

Byte48–Byte55 第 7 个关节指令数据

Byte56–Byte63 第 8 个关节指令数据

关节应答（CAN ID = 电机 ID）

字节 功能 说明

Byte0–Byte1 关节当前位置 16bit

Byte2–Byte3 关节当前速度 12bit

字节 功能 说明

Byte3–Byte4 关节当前扭矩 12bit

Byte5 MOS 温度 uint8，-40~215℃

Byte6–Byte7 电机状态 见故障寄存器位定义

Byte8 当前电压 uint8，0–255V

Byte9 线圈温度 uint8，-40~215℃

Byte10–Byte11 预留 —

### 4. 控制协议格式

### 4.1.1 主机发送模式切换数据

###  CAN ID：0x00

###  帧类型：标准帧

###  数据域：高字节在前，低字节在后

字节 功能 备注

0x00=失能， Byte0 使能 0x01=使能

Byte1–Byte7 预留

### 4.1.2 主机发送期望关节数据

4.1.2.1 MIT 模式说明

控制公式 力矩指令 = Kp × (位置指令 – 反馈位置) + Kd × (速度指令 – 反馈速度) + 期望力矩

CAN ID：0x80 帧类型：标准帧

字节 内容 备注

字节 内容 备注

Byte0-Byte1 16bit） 0x0000~0xFFFF 代表 [-10~ 10] rad; 期望关节位置 0x7FFF = 0rad;

Byte2-Byte3 （12bit） 0x000~0xFFF 代表 [-60~ 60] rad/s; 期望关节速度 0x7FF = 0rad/s;

Byte3-Byte4 （12bit）0x000~0xFFF 代表 [0~ 500] ; 期望关节 Kp 0x000 = 0;

Byte5-Byte6 （12bit） 0x000~0xFFF 代表 [0~ 10] ; 期望关节 Kd 0x000 = 0;

Byte6-Byte7 （12bit） 0x000~0xFFF 代表 [-60~ 60] Nm; 期望关节扭矩 0x7FF = 0Nm;

示例：给 8 个关节发送控制指令：（0 位置、0 速度、0 扭矩、0kp、0kd）

字节 内容 备注（低字节在后）

Byte0-Byte7 第 1 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte8-Byte15 第 2 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte16-Byte23 第 3 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte24-Byte31 第 4 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte32-Byte39 第 5 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte40-Byte47 第 6 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte48-Byte55 第 7 个关节指令数据 7F FF 7F F0 00 00 07 FF

Byte56-Byte63 第 8 个关节指令数据 7F FF 7F F0 00 00 07 FF

4.1.2.2 规划位置模式（PP）

CAN ID：0x80 + N， N=设备 ID ，标准帧

字节 功能 备注

Byte0-Byte3 期望当前位置 Float

4.1.2.3 规划速度模式（PV）

CAN ID：0x80 + N， N=设备 ID ，标准帧

字节 功能 备注

Byte0-Byte3 期望关节速度 Float

### 4.1.3 关节应答状态数据（MIT 格式）

字节 内容 范围

Byte0–Byte1 位置 0x000–0xFFFF → -10~10 rad

Byte2–Byte3 速度 0x000–0xFFF → -60~60 rad/s

Byte3–Byte4 扭矩 0x000–0xFFF → -60~60 Nm

Byte5 MOS 温度 0x00–0xFF → -40~215℃

Byte6–Byte7 状态 见故障寄存器位

Byte8 电压 0–255 → 0–255V

Byte9 线圈温度 0x00–0xFF → -40~215℃

Byte10–Byte11 预留 —

### 4.1.4 MIT 转换示例

转换公式

int FloatToUint(float x, float x_min, float x_max, int bits)

{

float span = x_max - x_min;

float offset = x_min;

return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);

}

float UintToFloat(int x_int, float x_min, float x_max, int bits)

{

float span = x_max - x_min;

float offset = x_min;

return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;

}

下行转换（主站 → 从站）

pos_tmp = (_data[data_offset] << 8) | _data[data_offset + 1];

vel_tmp = (_data[data_offset + 2] << 4) | (_data[data_offset + 3] >> 4);

kp_tmp = ((_data[data_offset + 3] & 0xF) << 8) | _data[data_offset + 4];

kd_tmp = (_data[data_offset + 5] << 4) | (_data[data_offset + 6] >> 4);

tor_tmp = ((_data[data_offset + 6] & 0xF) << 8) | _data[data_offset + 7];

mit_pos = UintToFloat(pos_tmp, posMin, posMax, 16);

mit_vel = UintToFloat(vel_tmp, velMin, velMax, 12);

mit_toq = UintToFloat(tor_tmp, torMin, torMax, 12);

mit_kp = UintToFloat(kp_tmp, kpMin, kpMax, 12);

mit_kd = UintToFloat(kd_tmp, kdMin, kdMax, 12);

上行转换（从站 → 主站）

pos_tmp = FloatToUint(motor.GetPosition(), posMin, posMax, 16);

vel_tmp = FloatToUint(motor.GetVelocity(), velMin, velMax, 12);

tor_tmp = FloatToUint(motor.GetTorque(), torMin, torMax, 12);

_data[0] = pos_tmp >> 8;

_data[1] = pos_tmp;

_data[2] = vel_tmp >> 4

;_data[3] = ((vel_tmp & 0xF) << 4) | (tor_tmp >> 8);

_data[4] = tor_tmp;

_data[5] = motor.GetTemperature();

_data[6] = motor.GetStatus() & 0xFF;

_data[7] = motor.GetStatus() >> 8;

_data[8] = motor.GetVoltage();

### 附录：故障寄存器补充说明

级别 说明 恢复方式

Level 1 告警，不需要干涉，不掉使能 自动恢复（条件满足后），但标志位需手动清除

Level 2 故障，电机执行保护措施 不可自动恢复，需手动清除

BIT6 之后 故障 不可自动恢复，需手动清除

清除方式：对应位写 0 或整个寄存器写 0。

示例： MOS 一级过温阈值 70℃，恢复阈值 50℃。 触发过温 → 告警标志置位 → 温度低于 50℃ 后告警状态自动恢复 → 主机清除标志，若周期内仍 满足条件则标志再次置位。