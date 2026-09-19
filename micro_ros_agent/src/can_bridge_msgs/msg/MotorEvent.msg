# 拓扑/安全事件（ARD-0007 §18.2）：即时通知；MCU 单调时戳仅用于事件排序。
# event_type 与 firmware motor_control_event_type_t 对齐：
#   0=DISCOVERED 1=PROFILE_BOUND 2=PROFILE_REJECTED 3=READY 4=ACTIVE
#   5=SOFT_STOP 6=DISABLE 7=FAULT 8=CONFIG_CONFLICT 9=OFFLINE
#   10=RECONNECTED 11=AUTHORIZATION_LOST 12=SNAPSHOT_REJECTED
#   13=CONFIG_RESULT 14=REGISTRY_FULL 15=DEV_INIT（设备参数上电比对定论，
#   reason = 0=NA 1=PENDING 2=OK 3=MISMATCH（未初始化，该台不可运动）
#   4=UNVERIFIED（回读失败，不拦运动但结论可见））
# reason：事件相关的原因/结果码（故障码或配置结果码）；request_id 仅
# CONFIG_RESULT 携带；EVENT_DEV_INIT 的 request_id 携带槽位位图
#   （1u<<slot：1=位置下限 2=位置上限 4=最大转速 8=峰值电流
#     16=保护开关(0x2650:00) 32=方向(0x2541)；0=无此项信息）。
#   位图两态语义不同：
#     reason==3 (MISMATCH)：位图 = **对不上**的槽位（记录有、设备读到的值不同）；
#     reason==4 (UNVERIFIED)：位图 = **没读到**的槽位（回读未到齐，非值不符）。
#   槽位号与 firmware 的 device_raw 槽索引一一对应；新增槽位必须同时改这里与
#   web_host 的 DEV_SLOT_NAMES（web/app.js）。
uint8 EVENT_DISCOVERED=0
uint8 EVENT_PROFILE_BOUND=1
uint8 EVENT_PROFILE_REJECTED=2
uint8 EVENT_READY=3
uint8 EVENT_ACTIVE=4
uint8 EVENT_SOFT_STOP=5
uint8 EVENT_DISABLE=6
uint8 EVENT_FAULT=7
uint8 EVENT_CONFIG_CONFLICT=8
uint8 EVENT_OFFLINE=9
uint8 EVENT_RECONNECTED=10
uint8 EVENT_AUTHORIZATION_LOST=11
uint8 EVENT_SNAPSHOT_REJECTED=12
uint8 EVENT_CONFIG_RESULT=13
uint8 EVENT_REGISTRY_FULL=14
uint8 EVENT_DEV_INIT=15
# 轨迹被整条拒绝。与其它事件不同，本事件的字段含义重载：
#   request_id = 被拒的 trajectory_id（并发多条时才分得清是谁）
#   reason     = 首个非法路点下标；0xFFFF = 与路点无关（计数越界/
#                声明集合重复/未注册电机/声明了归属却无路点）
#   motor_id   = 0（一条轨迹可含多台，逐台归属无意义）
uint8 EVENT_TRAJECTORY_REJECTED=16
uint32 tick_ms
uint8 event_type
uint16 motor_id
uint8 bus_index
uint16 reason
uint32 request_id
# 单调递增（内核分配，跨事件连续）。消费者据此判丢帧：相邻两条之间跳号即
# 说明事件环满过；累计丢弃数见 MotorDiagnostics.event_dropped。
uint32 event_seq
