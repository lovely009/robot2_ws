# 系统命令（上层 → MCU，reliable）。
#
# 这四条此前是参数树的 system.* 写项——它们是**命令不是参数**：写下去 = 一个
# 动作，完成判据是 /micro_ros_up/system_status 里对应序号的推进。语义没变，
# 只是回到它本来该待的通道。
#
# 完成判据（**唯一**可靠的"等它做完"谓词）：
#   SAVE        → system_status.save_seq 前进（且 save_in_progress 已转 false）
#   REFRESH     → system_status.refresh_seq 前进
#   SET_ZERO    → system_status.set_zero_seq 前进
#   CLEAR_FAULT → system_status.clear_fault_seq 前进
#
# 非法 command / 越界 motor_id → 本地拒绝（system_status.last_error = 10）。
# 不新增错误码：操作员码表零迁移。
uint64 sync_time_ns
uint32 request_id
# 命令码
uint8 COMMAND_SAVE=0
uint8 COMMAND_REFRESH=1
uint8 COMMAND_SET_ZERO=2
uint8 COMMAND_CLEAR_FAULT=3
uint8 command
# 目标电机（0..63）；MOTOR_ID_NONE = 不适用。
# SAVE 只接受 MOTOR_ID_NONE——本轮只实现整机保存（W25 那半必须整体落盘，按台
# 保存只能省设备侧一半，语义含混）。其余三条命令必须给有效 id。
uint16 MOTOR_ID_NONE=65535
uint16 motor_id
