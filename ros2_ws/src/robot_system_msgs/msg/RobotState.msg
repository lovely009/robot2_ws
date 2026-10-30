# 主机整机监督状态。armed 是主机授权，不替代 MCU 权威控制状态或驱动器回读。
std_msgs/Header header
uint8 BOOTING=0
uint8 NOT_READY=1
uint8 READY=2
uint8 ARMED=3
uint8 RUNNING=4
uint8 HOLD=5
uint8 FAULT=6
uint8 state
bool ready
bool armed
# Fresh MCU acknowledgement of this armed generation; enable/trajectory inputs
# must wait for this, not merely the host's SetArmed service response.
bool mcu_authorized
bool fault_latched
# supervisor 实例代数；租约不跨实例复用。
uint64 epoch
# MCU 当前会话，沿用 can_bridge_msgs 的 uint64 epoch 毫秒契约。
uint64 session_id
string owner
# 取值见 AcquireControl 请求中的 MANUAL / TRAJECTORY / TASK。
uint8 mode
# 仅作显示；租约过期裁决使用 supervisor 的本地单调时钟。
uint32 lease_remaining_ms
string calibration_id
string config_hash
uint16[] motor_ids
string[] blockers
