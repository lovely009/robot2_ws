# 线上契约的机器可读唯一来源：话题名、定点量程、哨兵、schema 版本、QoS 类别码。
# 固件与主机 bridge 都只引用这里生成出来的常量，不各自写字符串字面量。
#
# 方向：do = 主机→固件，up = 固件→主机。
#
# 本消息**从不发布**，只作常量载体；placeholder 的唯一作用是避免空结构体。
#
# 定点：q = clamp(round(x * 32767 / SCALE), -32767, 32767)；-32768 留给 SENTINEL_MISSING。
# 单位边界：换算只发生在两侧 codec 里，控制逻辑仍用 rad / rad/s / A / N·m。

string TOPIC_LOW_CMD=/micro_ros_do/low_cmd
string TOPIC_POLICY=/micro_ros_do/policy
string TOPIC_TRAJECTORY=/micro_ros_do/trajectory
string TOPIC_CONFIG=/micro_ros_do/config
string TOPIC_LOW_STATE=/micro_ros_up/low_state
string TOPIC_MOTOR_STATUS=/micro_ros_up/motor_status
string TOPIC_TOPOLOGY=/micro_ros_up/topology
string TOPIC_EVENTS=/micro_ros_up/events
string TOPIC_DEVICE_PARAM=/micro_ros_up/device_param
string TOPIC_IMU_DATA=/micro_ros_up/imu/data
string TOPIC_IMU_TEMP=/micro_ros_up/imu/temp

# 消息面契约版本：随字段**语义**变更递增（字段只增不改序）。
uint16 SCHEMA_VERSION=1
# 单条消息可携带的最大电机数（= 内核 MOTOR_CONTROL_MAX_MOTORS）。
uint8 MOTOR_COUNT_MAX=64
# QoS 类别码：0 = 高频流（best-effort depth 1）；1 = 低频状态声明（reliable）。
uint8 QOS_STREAM=0
uint8 QOS_RELIABLE=1
# 定点哨兵：本拍无有效测量。
int16 SENTINEL_MISSING=-32768
# 定点量程（full-scale，双极性）。
float32 SCALE_POSITION_RAD=12.5
float32 SCALE_VELOCITY_RAD_S=64.0
float32 SCALE_EFFORT_NM=64.0

uint8 placeholder
