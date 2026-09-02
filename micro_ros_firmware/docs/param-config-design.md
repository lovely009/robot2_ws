# micro-ROS 参数下发电机拓扑设计

## 概述

固件不再维护硬编码的电机拓扑表，改为通过 micro-ROS 参数服务从上层（ROS 2）动态下发配置。两侧共用同一份 `motor_driver.yaml`。

## 数据流

```
motor_driver.yaml (ROS 2)
    ↓ motor_config_publisher (launch)
    ↓ ros2 param set /motor_config ...
micro_ros_agent (参数桥接)
    ↓ parameter callback
micro_ros_node (固件 ros_bridge)
    ↓ write config
control_task (重建电机表)
    ↓ motor_control_reconfigure
motor_control (运行时重建)
```

## 参数定义

### 总线参数 (buses)

```
/motor_config/buses/left_arm/transport : string ("serial_slcan")
/motor_config/buses/left_arm/device    : string ("/dev/ttyACM0")
/motor_config/buses/right_arm/transport : string ("serial_slcan")
/motor_config/buses/right_arm/device    : string ("/dev/ttyACM1")
```

### 电机参数 (motors)

```
/motor_config/motors/count : integer (电机数量)
/motor_config/motors/joint_name_0 : string ("left_shoulder_yaw_joint")
/motor_config/motors/protocol_0    : string ("rp")
/motor_config/motors/id_0          : integer (1)
/motor_config/motors/bus_0         : string ("left_arm")
/motor_config/motors/gear_ratio_0  : double (1.0)
/motor_config/motors/gear_efficiency_0 : double (1.0)
/motor_config/motors/torque_constant_0 : double (0.0)
/motor_config/motors/default_kp_0  : double (30.0)
/motor_config/motors/default_kd_0  : double (1.0)
...
```

### 总线映射参数

```
/motor_config/bus_mapping/left_arm  : integer (0 = CAN1)
/motor_config/bus_mapping/right_arm : integer (1 = CAN2)
```

## 修改文件清单

### micro_ros_firmware 侧

1. **lib/motor_control/include/motor_control.h**
   - 新增 `motor_control_reconfigure()` 接口

2. **lib/motor_control/src/motor_control.c**
   - 实现运行时重建电机表（先停机，再替换配置）

3. **include/control_task.h**
   - 新增 `control_task_reconfigure()` 接口

4. **src/control_task.cpp**
   - 实现重建入口（停机 → 等待 → 重建 → 重启）

5. **include/ros_bridge.h**
   - 无需修改

6. **src/ros_bridge.cpp**
   - 注册 micro-ROS 参数
   - 实现参数回调

7. **src/motor_bridge.cpp**
   - 新增 `motor_bridge_apply_config()` 实现参数解析和应用

### ros2_ws 侧

8. **motor_hardware_interface/launch/param_publish.launch.py** (新文件)
   - 读取 motor_driver.yaml 并通过 ros2 param set 下发

## 安全约束

1. **停机优先**：收到参数后必须先停止所有电机，等待反馈超时
2. **原子重建**：电机表替换必须是原子操作（单次临界区）
3. **授权重置**：重建后必须重新授权（`micro_ros/motor_enable` 发 false）
4. **无重放**：重建后不保留旧命令，不重放历史命令

## 测试验证

1. 固件编译通过
2. 启动后无默认电机（等待参数下发）
3. 上层下发参数后电机表生效
4. 运行中重新下发参数，电机安全停止并重建
