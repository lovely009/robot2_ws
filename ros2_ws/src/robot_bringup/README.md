# 工作台启动

```bash
source /opt/ros/humble/setup.bash
source micro_ros_agent/install/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robot_bringup robot.launch.py
```

打开 http://localhost:8080 。在工作台选择模型，使用 Agent、仿真、使能和目标操作。
启动不会自动使能电机；Agent 串口遵循工作台原有选择逻辑。`bench_manager` 按模型选择管理主机控制会话。
`bench.launch.py` 单独启动工作台管理；`firmware_tf.launch.py` 发布实测关节与 TF。

执行路径为 `工作台 → workbench_control → ros2_control → MicroRosSystem → micro_ros_bridge → MCU`。
手动、轨迹和重力补偿由主机控制器执行。MCU 保留限位、故障、命令过期、授权期限与停止保护。
桥只在显式使能请求后建立 MCU 传输授权，等待确认后转发原请求；停止或连接过期后不会自动恢复。
模型切换会停止原主机控制会话，新模型仍需重新使能。
会话按新鲜在线电机的完整传动组建立：直连单电机可独立控制，差动组必须全组接入。
电机集合或 MCU 会话变化会先停止旧控制，再重建未使能的新会话；未接入关节不生成反馈。
关节区统一提供目标、使能和限位编辑；使能仅作用于当前控制会话的完整传动组，勾选状态以 MCU 回执为准。失能会先停止主机控制器及待发目标。限位修改需主机确认后生效。

仿真的结果不代表真实机构验收。构建不执行烧录或电机动作。
