# 轨迹控制入口；ACK/CANCEL 不夹带路点。
uint8 LOAD=0
uint8 COMMIT=1
uint8 CANCEL=2
uint8 ACK=3
uint64 session_id
uint64 issued_stamp
uint32 request_id
uint32 trajectory_id
uint32 ack_request_id
uint32 ack_terminal_trajectory_id
uint16 ttl_ms
uint16 schema_version 3
uint8 operation
uint16 segment_index
uint16 segment_count
uint16[<=48] target_ids
MotorTrajectoryPoint[<=32] points
