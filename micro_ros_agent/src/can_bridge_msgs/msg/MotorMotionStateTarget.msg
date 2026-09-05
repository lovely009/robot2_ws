# 运动状态元素（ARD-0007 §16.3）：缺失浮点 = NaN（ARD §38.3）。
# 数值均为关节侧 SI；state_age_ms 为 MCU 单调时基下最近有效运动测量的年龄，
# 上层据此判断测量新鲜度（100Hz 发布 ≠ 100Hz 新测量，ARD §11.4/禁止项 17）。
uint16 motor_id
float32 position      # rad，关节侧（缺失=NaN）
float32 velocity      # rad/s，关节侧（缺失=NaN）
float32 effort        # N·m，关节侧（缺失=NaN）
uint32 state_age_ms   # 0xFFFFFFFF = 从未收到运动测量
