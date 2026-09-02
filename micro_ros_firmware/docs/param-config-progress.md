# micro-ROS 参数下发电机拓扑 —— 工作进度

## 当前状态

**参数服务器现由 `ENABLE_PARAMETER_SERVER=0` 在预处理阶段关闭**，固件继续使用
内置电机表；相关实现和 5-service 资源配置保留，供后期参数拓扑方案重新启用。

此前 agent 连接失败的根因是 micro_ros_platformio 默认只为
`rmw_microxrcedds` 预留 1 个 service，而 `rclc` 参数服务器固定需要 5 个；第 2 个
service 创建时初始化失败，固件随即销毁节点，所以 agent 侧能看到建实体流量，
`ros2 node list` 却看不到 `/micro_ros_node`。

项目通过 `microros.meta` 将 `RMW_UXRCE_MAX_SERVICES` 配为 5，并由
`platformio.ini` 的 `board_microros_user_meta` 显式启用。生成配置及固件构建已验证，
但默认构建不会创建参数实体。后期将 `ENABLE_PARAMETER_SERVER` 改为 `1` 后即可恢复。

## 已完成的工作

### 1. 固件侧 (micro_ros_firmware)

#### motor_control 层
- `lib/motor_control/include/motor_control.h`：新增 `motor_control_reconfigure()` 接口
- `lib/motor_control/src/motor_control.c`：实现运行时重建电机表（停机 → 清除状态 → 复制新配置 → 重置运行时）

#### control_task 层
- `include/control_task.h`：新增 `control_task_reconfigure()` 接口
- `src/control_task.cpp`：实现重建入口（停机 → 等待 500ms → 临界区重建 → 重置 mailbox）

#### ros_bridge 层
- `src/ros_bridge.cpp`：
  - 注册 micro-ROS 参数服务器（`rclc_parameter_server_t`）
  - 添加 `rclc_executor_add_parameter_server_with_context` 调用
  - 添加参数回调和配置应用逻辑

#### motor_bridge 层
- `src/motor_bridge.cpp`：
  - 实现参数解析回调 `motor_bridge_param_callback()`
  - 实现配置应用函数 `motor_bridge_apply_pending_config()`
- `include/motor_bridge.h`：声明参数回调和配置应用函数

### 2. ROS 2 侧 (ros2_ws)

#### 配置文件
- `motor_hardware_interface/config/motor_driver.yaml`：已简化，当前只保留 2 台 vaxor 电机
- `motor_hardware_interface/config/motor.yaml`：添加 `protocol_defaults`（rp/vaxor 的默认 kp/kd）

#### Launch 文件
- `motor_hardware_interface/launch/param_publish.launch.py`：只下发参数（一次性）
- `motor_hardware_interface/launch/bringup.launch.py`：agent + 参数下发 + 授权（一键启动）

### 3. 构建验证
- ✅ 固件编译通过（RAM 14.6%, Flash 18.7%）
- ✅ 7 个固件主机测试全部通过
- ✅ ros2_ws 235 个测试通过（1 个 xacro 测试因配置简化失败，预期行为）

## 当前问题

### Agent 连接失败（代码侧已修复，待烧录确认）
- 症状：指示灯不绿闪，`ros2 node list` 看不到 `/micro_ros_node`
- 已确认根因：参数服务器需要 5 个 service，已编译的 libmicroros 仅配置 1 个
- 修复：新增项目级 `microros.meta`，将 `RMW_UXRCE_MAX_SERVICES` 提升到 5
- 回归保护：主机测试编译实际生成的 libmicroros `config.h`，容量不足会直接失败
- executor handles 计算正确：参数服务器固定占 5 个 handle

### 需要排查的点

1. **烧录已修复固件**：运行 `pio run -t upload` 并重启板子
2. **确认连接**：检查指示灯绿闪及 `ros2 node list` 中的 `/micro_ros_node`
3. **验证参数类型兼容性**：Humble 的 `rclc_parameter` 仅支持 bool/int/double，
   当前字符串字段（joint/protocol/bus）仍需重新编码或改用专用配置消息/服务

## 参数下发机制

### 参数命名约定
```
bus_mapping_{bus_name}          : integer (0/1/2 对应 FDCAN1/2/3)
motor_{i}_joint_name            : string
motor_{i}_protocol              : string ("rp" 或 "vaxor")
motor_{i}_id                    : integer (1-60)
motor_{i}_bus                   : string (总线名)
motor_{i}_gear_ratio            : double
motor_{i}_gear_efficiency       : double
motor_{i}_torque_constant       : double
motor_{i}_default_kp            : double
motor_{i}_default_kd            : double
motor_count                     : integer
```

### 参数优先级
```
motor_driver.yaml command.kp/kd  (最高，单电机覆盖)
        ↓ 未设置时
motor.yaml protocol_defaults     (协议级默认)
        ↓ 未设置时
硬编码默认值 0.008 / 1.0         (最低)
```

### 参数下发流程
```
motor_driver.yaml + motor.yaml
    ↓ launch 解析
ros2 param set /micro_ros_node ...
    ↓ micro-ROS 参数服务
固件 motor_bridge_param_callback()
    ↓ 收集参数
motor_bridge_apply_pending_config()
    ↓ control_task_reconfigure()
motor_control_reconfigure()
    ↓ 重建电机表
新的电机配置生效
```

## 下一步建议

### 1. 真机确认连接修复
- 烧录本次构建产物并重启板子
- 确认节点注册、参数服务可见且断连重连正常

### 2. 修复连接问题后
- 测试参数下发功能
- 测试电机控制功能
- 更新测试以适配新配置

### 3. 文档更新
- 更新 AGENTS.md
- 更新 motor_hardware_interface/README.md
- 更新 micro_ros_firmware/README.md

## 文件清单

### 修改的文件
```
micro_ros_firmware/
├── lib/motor_control/include/motor_control.h    # 新增 reconfigure 接口
├── lib/motor_control/src/motor_control.c        # 实现 reconfigure
├── include/control_task.h                       # 新增 reconfigure 接口
├── include/motor_bridge.h                       # 新增参数回调声明
├── src/control_task.cpp                         # 实现 reconfigure
├── src/motor_bridge.cpp                         # 实现参数解析和应用
└── src/ros_bridge.cpp                           # 参数服务器初始化

ros2_ws/src/motor_hardware_interface/
├── config/motor_driver.yaml                     # 简化配置
├── config/motor.yaml                            # 添加 protocol_defaults
└── launch/
    ├── param_publish.launch.py                  # 只下发参数
    └── bringup.launch.py                        # agent + 参数下发
```

### 新增的文件
```
micro_ros_firmware/docs/param-config-design.md   # 设计文档
micro_ros_firmware/docs/param-config-progress.md # 本文件
```

## 关键代码片段

### motor_bridge_param_callback 核心逻辑
```c
bool motor_bridge_param_callback(const Parameter *old_param,
                                 const Parameter *new_param,
                                 void *context) {
  if (new_param == NULL) return false;

  // 处理 motor_count
  if (strcmp(new_param->name.data, "motor_count") == 0) {
    int64_t count = 0;
    if (param_get_int(new_param, &count) && count > 0) {
      g_pending_motor_count = (size_t)count;
      g_config_pending = true;
      return true;
    }
  }

  // 处理 bus_mapping_{name}
  if (strncmp(new_param->name.data, "bus_mapping_", 12) == 0) {
    // ...
  }

  // 处理 motor_{i}_{field}
  if (strncmp(new_param->name.data, "motor_", 6) == 0) {
    // ...
  }
}
```

### motor_bridge_apply_pending_config 核心逻辑
```c
void motor_bridge_apply_pending_config(void) {
  if (!g_config_pending || g_pending_motor_count == 0) return;

  // 检查配置完整性
  for (size_t i = 0; i < g_pending_motor_count; ++i) {
    motor_control_motor_config_t *motor = &g_pending_motors[i];
    if (motor->joint_name[0] == '\0') return;  // 不完整
    if (motor->id == 0) return;  // 无效
  }

  // 调用重建
  if (control_task_reconfigure(g_pending_motors, g_pending_motor_count)) {
    g_config_pending = false;
    g_pending_motor_count = 0;
  }
}
```

## 测试命令

```bash
# 构建固件
cd /home/ly/robot_ws/micro_ros_firmware
~/.platformio/penv/bin/pio run

# 烧录固件
~/.platformio/penv/bin/pio run -t upload

# 运行固件主机测试
cmake -E remove_directory .host-build
cmake -S test/host -B .host-build
cmake --build .host-build --clean-first
ctest --test-dir .host-build --output-on-failure

# 构建 ros2_ws
source /opt/ros/humble/setup.bash
cd /home/ly/robot_ws/ros2_ws
colcon build --symlink-install

# 运行 ros2_ws 测试
colcon test && colcon test-result --verbose

# 启动 agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6

# 下发参数
ros2 launch motor_hardware_interface param_publish.launch.py
```
