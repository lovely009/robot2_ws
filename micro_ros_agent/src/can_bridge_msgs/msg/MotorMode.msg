# DEPRECATED（Phase C）：joint_name/raw-CAN 依赖面，固件已停用；上层迁移完成后删除。

# 关节工作模式请求。模式切换不会复用此前的控制目标。
string joint_name

uint8 MODE_MIT=0
uint8 MODE_VELOCITY=1
uint8 MODE_POSITION=2
uint8 mode
