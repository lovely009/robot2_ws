# 单台设备的参数核对双行快照（MCU → 上层，低频）。
#
# 两行必须同单位、同槽位顺序才可比，故都走关节侧 SI：位置 rad、速度 rad/s、
# 力矩 A（设备对象 0x2538 存的就是电流，力矩列按 A 呈现与参数面一致）。换算在
# 固件侧完成——页面不碰 cnt / mA / RPM。
#
# 缺失量一律 NaN（不填 0：0 在页面上是个能被人按下去的值）。
uint16 motor_id
# motor_control_dev_init_t：0=NA（本协议无设备对象）/1=PENDING/2=OK/
# 3=MISMATCH/4=UNVERIFIED/5=NO_RECORD（本机对该台无记录 → 固件列整行「无记录」）
uint8 dev_init_state
# 已写、未保存：设备侧 0x2539 与 W25 都没落盘。含保护开关/方向两槽——它们没有
# 设备保存码，但记录要落 W25 才生效（方向尤其靠记录在下次上电重建）。
bool record_dirty
# 记录与设备对不上的槽位位图（口径同 MotorEvent 的 DEV_INIT detail）：
# bit0=位置下限 bit1=位置上限 bit2=最大转速 bit3=峰值电流
# bit4=保护开关 bit5=方向。仅在 dev_init_state==MISMATCH 时非零。
uint8 mismatch_mask
# 固件记录侧（本机权威值）
float32 rec_pos_min
float32 rec_pos_max
float32 rec_vel_limit
float32 rec_torque_limit
float32 rec_safety
float32 rec_direction
# 驱动板回读侧（影子）
float32 dev_pos_min
float32 dev_pos_max
float32 dev_vel_limit
float32 dev_torque_limit
float32 dev_safety
float32 dev_direction
