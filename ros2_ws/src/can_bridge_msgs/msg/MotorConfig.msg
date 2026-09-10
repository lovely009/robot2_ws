# 设备配置事务请求（ARD-0007 §20，上层 → MCU）。
# 每电机同时至多一个未决事务；新请求立即 BUSY（不排队、不覆盖，ARD §20.3）。
# request_id 由上层生成（非 0），结果经 MotorConfigResult 以同 id 关联。
# kind 与取值：
#   KIND_SET_MODE  : mode（0=MIT 1=velocity 2=position；CREB 仅支持 MIT）
#   KIND_SET_LIMITS: min/max_position_rad（设备侧限位，rad；写后必读回核验）
# 厂家管理对象与阶段超时未决项：TBD-CONFIG-01/02。
uint8 KIND_SET_MODE=1
uint8 KIND_SET_LIMITS=3
uint32 request_id
uint16 motor_id
uint8 kind
uint8 mode               # KIND_SET_MODE
float32 min_position_rad # KIND_SET_LIMITS
float32 max_position_rad # KIND_SET_LIMITS
