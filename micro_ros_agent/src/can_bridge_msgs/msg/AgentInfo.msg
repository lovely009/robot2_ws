uint8 DEVICE_PAGE=0
uint8 OPERATION_DETAILS=1
uint8 OK=0
uint8 RETRY=1
uint8 BUSY=2
uint8 RETIRED=3
uint64 session_id
uint64 snapshot_stamp
uint32 snapshot_tick_ms
uint16 schema_version 3
bool clock_valid
uint32 query_id
uint8 kind
uint8 result
uint32 revision
uint16 cursor
uint16 next_cursor
bool complete
uint16 motor_capacity 48
uint16 trajectory_point_capacity 256
AgentDeviceInfo[<=16] devices
uint32 detail_request_id
AgentOperationDetail[<=16] details
