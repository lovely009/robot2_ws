# micro-ROS 数值参数电机拓扑设计

## 目标

继续使用 ROS 2 标准参数服务，在 Humble `rclc_parameter` 仅支持
bool/int/double 的限制下，从主机 YAML 向固件下发完整电机拓扑。固件启动时仍以
内置电机表运行；只有收到并校验通过一整份新配置后，才原子替换运行时电机表。

## 配置来源与优先级

- 主机侧 `motor_driver.yaml` 是电机拓扑唯一来源。
- 主机侧 `motor.yaml` 提供协议和型号默认参数。
- 固件 `kDefaultMotors` 仅是上电及下发失败时的回退，不是主机拓扑来源。
- 每次启动先加载 `kDefaultMotors`；完整参数配置应用成功后覆盖运行时表。
- 不完整、非法或超时的参数批次不得改变当前运行时表。

## 标准参数编码

所有参数值仅使用 `int64` 或 `double`。

### 批次控制

参数逐项通过标准 `ros2 param set` 下发。`config_generation` 是提交标志：上层先
下发所有配置字段，最后下发一个严格递增的正整数 generation。固件只在 generation
变化时校验并尝试应用整批配置。

### 总线

主机根据 `motor_driver.yaml` 中每条 bus 的 `micro_can` 计算零基 FDCAN 索引。
固件直接接收 `motor_{i}_bus_index`，范围为 0..2，不再接收总线名字符串或维护
额外的名称映射。

### 协议

`motor_{i}_protocol` 使用稳定整数枚举：

- `0`：RP
- `1`：VAXOR

未知枚举值拒绝整批配置。

### 关节名

关节名按 UTF-8 原始字节编码。每个 chunk 最多携带 7 字节，按 little-endian
打包进非负 `int64`，避免使用符号位：

- `motor_{i}_joint_name_length`：字节长度
- `motor_{i}_joint_name_chunk_0`
- `motor_{i}_joint_name_chunk_1`
- 依此类推，直到覆盖 length

固件容量仍由 `MOTOR_CONTROL_JOINT_NAME_CAPACITY` 决定。长度必须大于 0 且小于
容量；未使用 chunk 必须清零。解码结果末尾由固件补 `\0`。

### 其余字段

- `motor_count`：1..`MOTOR_CONTROL_MAX_MOTORS`
- `motor_{i}_id`：1..60
- `motor_{i}_gear_ratio`：正 double
- `motor_{i}_gear_efficiency`：(0, 1] double
- `motor_{i}_torque_constant`：非负 double
- `motor_{i}_default_kp`：非负 double
- `motor_{i}_default_kd`：非负 double

## 固件收集与原子应用

固件使用独立 pending 缓冲区及逐字段 presence 位图。普通字段回调只更新 pending，
不触发停机或重配置。收到新的 `config_generation` 后提交异步应用请求。参数服务器
另外公开两个整数状态参数：`config_applied_generation` 和 `config_error_code`。
control task 处理请求时：

1. 检查 generation 严格大于最近成功应用的 generation。
2. 检查 motor_count、每台电机的必需字段、数值范围和全部关节名 chunk。
3. 检查同一总线内 `(protocol, id)` 不重复，关节名全局不重复。
4. 生成完整的 `motor_control_motor_config_t` 临时数组。
5. 请求撤销运动授权并清除旧命令。
6. 在 control task 所有权边界内替换 motor_control 和 mailbox 配置。
7. 保持未授权状态；必须重新发送 `motor_enable=true` 和新命令才能运动。
8. 成功时更新 `config_applied_generation`、清零 `config_error_code`，并清空 pending
   presence 状态；失败时只更新非零 `config_error_code`。

校验失败时当前运行时表和授权状态均保持不变，pending 字段保留以便上层修正后
重新提交 generation。由于应用发生在 control task，generation 参数设置成功仅表示
请求已入队；上层必须等待 `config_applied_generation` 等于目标 generation 才能报告
成功。

## 动态数量修复

`control_task` 增加受临界区保护的当前电机数量，初始化为默认数量，重配置成功后
更新。以下路径不再使用 `kDefaultMotorCount`：

- JointState 反馈索引边界
- 命令 mailbox drain 数量
- 临时 pending 命令数组遍历
- motor bridge 发布的反馈数量和名称

固定容量数组仍按 `MOTOR_CONTROL_MAX_MOTORS` 分配，循环只遍历当前数量。

## 上层下发

`param_publish.launch.py` 继续读取两份 YAML，但输出新的纯数值参数名。它必须先
完成本地 YAML 校验和 joint-name chunk 编码，再顺序执行参数设置，最后设置
`config_generation`。任一字段命令失败时不得提交 generation；提交后轮询
`config_applied_generation` 和 `config_error_code`，超时或错误均返回非零状态，不能
只打印“参数下发完成”。

## 错误与恢复

- agent 断连不恢复旧命令，现有重新授权语义不变。
- 参数批次不完整或非法时继续使用当前运行时表；首次配置失败即继续使用默认表。
- 重配置期间不在 ROS executor timer 回调内等待 500ms；timer 只提交请求，control
  task 在自己的周期内执行停机和替换，避免阻塞 micro-ROS executor。
- 参数服务器关闭编译开关时，所有新增参数路径均不创建实体，默认电机表照常工作。

## 测试

### 主机固件测试

- 关节名 0、1、7、8、最大合法长度的编码/解码。
- 非法长度、缺 chunk、符号位、非法 UTF-8 原始字节处理的一致性。
- 缺字段、非法协议/总线/ID/浮点值、重复关节名和重复地址拒绝。
- generation 未变化不应用；完整新 generation 只应用一次。
- 异步成功/失败分别更新 applied generation 和 error code。
- 重配置后少于、等于和多于默认数量时，命令与反馈循环使用当前数量。
- 配置失败不改变当前表、授权或 mailbox。

### ROS 2 侧测试

- YAML 到纯数值参数的确定性转换。
- 7-byte chunk 边界及 UTF-8 字节往返。
- 任一 `ros2 param set` 失败时不提交 generation。
- generation 最后提交。
- 只有 applied generation 匹配时报告成功，错误或超时返回失败。

### 构建验证

- 参数服务器开、关两种固件构建均成功。
- 固件全部主机测试通过。
- `motor_hardware_interface` 相关 pytest 通过。

真机只做无负载验证：应用配置后仍须重新授权和发送新命令，硬件急停必须可用。
