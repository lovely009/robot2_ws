# robot_arm_gazebo

UR5e 机械臂 Gazebo 仿真集成包，提供完整的仿真环境、控制器配置与启动入口。

## 定位

作为 `ros2_ws` 仿真栈的顶层集成包，将 `motor_sim` 仿真库与 Gazebo 仿真器集成，
提供开箱即用的 UR5e 机械臂仿真环境。

## 核心功能

### 仿真环境

- **UR5e 模型**：URDF/Xacro 格式定义，包含几何、惯性、关节限位
- **Gazebo 世界**：预配置仿真环境（光照、地面、可选物体）
- **物理引擎**：Gazebo 默认物理参数（可调）

### 控制器

| 控制器 | 类型 | 用途 |
|---|---|---|
| `joint_trajectory_controller` | 位置控制 | 轨迹跟踪 |
| `forward_command_controller` | 直接力矩 | 力矩控制 |
| `gz_ros2_control` | Gazebo 接口 | 硬件抽象层 |

### 电机仿真

- **MotorSimNode**：基于 `motor_sim` 库的电机仿真节点
- **协议兼容**：支持 RP、VAXOR、CERB 协议
- **安全监控**：超限检测与锁存保护

### 辅助功能

- **BrakeCoordinator**：抱闸协调器（可选）
- **拓扑管理**：从 YAML 配置动态生成电机拓扑
- **状态发布**：关节状态、电机状态、诊断信息

## 目录结构

```
robot_arm_gazebo/
├── config/          # 控制器配置、电机拓扑
├── include/         # C++ 头文件
├── launch/          # 启动文件
├── scripts/         # 辅助脚本
├── src/             # C++ 源文件
├── test/            # 测试文件
├── urdf/            # URDF/Xacro 模型
├── vendor/          # 第三方模型（UR5e）
├── worlds/          # Gazebo 世界文件
└── README.md        # 本文件
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_arm_gazebo
colcon test --packages-select robot_arm_gazebo && colcon test-result --verbose
```

## 依赖关系

```
robot_arm_gazebo
├── motor_sim（电机仿真库）
├── ament_index_cpp（包索引）
├── rclcpp（ROS 2 客户端）
├── sensor_msgs（传感器消息）
├── std_msgs（标准消息）
├── controller_manager_msgs（控制器管理）
├── diagnostic_msgs（诊断消息）
└── Gazebo 相关（运行时依赖）
    ├── gz_ros2_control
    ├── controller_manager
    ├── forward_command_controller
    ├── joint_trajectory_controller
    ├── ros_gz_bridge
    ├── ros_gz_sim
    └── xacro
```

## 使用方式

### 启动仿真

```bash
# 默认启动（vcan 总线）
ros2 launch robot_arm_gazebo gazebo.launch.py

# 覆盖配置
ros2 launch robot_arm_gazebo gazebo.launch.py \
  motor_config_file:=/path/to/custom/motor_driver.yaml
```

### 控制机器人

```bash
# 轨迹控制
ros2 topic pub /joint_commands sensor_msgs/msg/JointState "{name: ['shoulder_yaw_joint'], position: [0.5]}"

# 读取状态
ros2 topic echo /joint_states
```

## 配置说明

### 电机拓扑

默认配置：`config/motors.yaml`（与真机栈格式兼容）

```yaml
motor_hardware:
  motor_specs_file: motor.yaml
  buses:
    vcan0: {transport: socketcan, interface: vcan0}
  motors:
    left_shoulder_yaw: {joint: shoulder_yaw_joint, bus: vcan0, type: rp-50h, id: 1}
```

### 控制器参数

- `config/ur5e_controllers.yaml`：控制器配置
- `config/gazebo_control.yaml`：Gazebo 控制器参数

## 与真机栈对比

| 特性 | robot_arm_gazebo（仿真） | motor_hardware_interface（真机） |
|---|---|---|
| 电机模型 | 数学模型（motor_sim） | 真实硬件 |
| 传输 | 虚拟 CAN（vcan） | 真实 CAN/串口 |
| 控制器 | 相同接口 | 相同接口 |
| 安全 | 软件模拟 | 硬件+软件 |
| 启动 | gazebo.launch.py | hardware.launch.py |

## 扩展指南

### 新增电机类型

1. 在 `motor_sim` 中实现协议
2. 在 `config/motor.yaml` 声明型号
3. 在 `config/motors.yaml` 添加电机实例
4. 测试验证

### 新增控制器

1. 在 `config/ur5e_controllers.yaml` 添加配置
2. 在 `launch/gazebo.launch.py` 启动控制器
3. 测试控制接口

## 测试覆盖

- Launch 静态检查（launch 测试）
- 拓扑一致性测试（YAML ↔ URDF ↔ 控制器）
- 控制器接口测试（话题/服务）
- Gazebo 集成测试（可选，需 GPU）

## 注意事项

1. **Gazebo 版本**：需要 Gazebo Fortress 或更新版本
2. **GPU 要求**：图形界面需要 GPU 支持（或使用 headless 模式）
3. **vcan 配置**：仿真使用虚拟 CAN 总线，需提前配置
4. **性能**：复杂场景可能需要较高计算资源
5. **与真机一致**：控制器接口与真机栈完全兼容，便于切换
