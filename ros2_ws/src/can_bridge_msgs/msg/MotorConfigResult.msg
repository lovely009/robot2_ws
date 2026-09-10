# 配置事务结果（ARD-0007 §20.5，MCU → 上层；TBD-ROS-04=A 结果话题）。
# 以 request_id 关联请求；读回不一致不得返回 OK（禁止项 9）。
# result（统一七码）：
#   0=OK 操作完成且已确认
#   1=BUSY 该电机已有未决事务
#   2=INVALID 请求非法
#   3=UNSUPPORTED 型号/协议不支持
#   4=DEVICE_ERROR 设备拒绝/协议执行错误
#   5=TIMEOUT 阶段超时且重试耗尽
#   6=UNCONFIRMED 写/读回无法确认或不一致（vaxor 无确认对象时模式切换收敛于此）
uint8 RESULT_OK=0
uint8 RESULT_BUSY=1
uint8 RESULT_INVALID=2
uint8 RESULT_UNSUPPORTED=3
uint8 RESULT_DEVICE_ERROR=4
uint8 RESULT_TIMEOUT=5
uint8 RESULT_UNCONFIRMED=6
uint32 request_id
uint16 motor_id
uint8 result
