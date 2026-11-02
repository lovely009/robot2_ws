# 配置终态与会话关联；同会话同 request_id/相同内容重试仅重发结果，不重复动作。
uint64 session_id
uint32 request_id
uint8 kind
uint8 command
uint8 STATUS_PENDING=0
uint8 STATUS_SUCCEEDED=1
uint8 STATUS_FAILED=2
uint8 STATUS_BUSY=3
uint8 STATUS_CONFLICT=4
uint8 status
uint16 error_code
uint16 completed_targets
uint16 failed_targets
# Host-issued policy identity; empty for legacy configuration callers. Not sent to MCU.
string intent_id
