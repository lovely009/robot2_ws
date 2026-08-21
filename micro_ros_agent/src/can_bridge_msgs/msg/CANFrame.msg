# 单条 CAN/CAN-FD 帧（桥接 micro_ros_firmware 的 FDCAN1 总线）
# header.stamp 由固件在时间同步后写入（rmw_uros_epoch_nanos）
std_msgs/Header header
uint16 id          # 标准/扩展 ID（11/29 位）
uint8 dlc          # 数据长度（字节数，0..64）
uint8[64] data     # 帧数据（FD 帧最多 64 字节）
bool is_extended   # true=扩展帧（29 位 ID）
bool is_fd         # true=CAN-FD 帧（BRS）