# Robot2

ROS 2 Humble 机器人控制工程，包含真机主机控制、micro-ROS Agent、传动模型和网页工作台。此发布不包含 MCU 固件、仿真和模拟电机实现。

电机控制链为 `JTC / follow_controller → MicroRosSystem → micro_ros_bridge → MCU → 电机`。主机生成周期目标，MCU 执行目标并处理本地故障和超时。

## 目录

| 目录或包 | 用途 |
| --- | --- |
| `micro_ros_agent` | Agent、共享消息定义和真机 CAN 工具 |
| `robot_description` | URDF、传动配置、型号库和 CAD 来源 |
| `robot_transmission` | 关节与执行器的位置、速度和力矩换算 |
| `micro_ros_bridge` | MCU 协议编解码、会话和控制授权 |
| `motor_hardware_interface` | ros2_control 插件、控制入口和反馈桥 |
| `robot_bringup` / `web_host` | 进程管理和网页工作台 |
| `bvh_broadcaster` | BVH 回放与离线动作映射 |

模型位于 `ros2_ws/src/robot_description/models`。主机、Agent 和固件使用同一份 `can_bridge_msgs`，线上协议版本由 `AgentWireContract.msg` 定义，当前为 schema 4。

## 构建与启动

网页构建需要 Node.js ^20.19.0 或 ≥22.12.0 和 npm。在仓库根目录执行以下命令。

```bash
cd ros2_ws/src/web_host/frontend
npm ci
npm run build
cd ../../../..

source /opt/ros/humble/setup.bash
cd micro_ros_agent
colcon build --packages-up-to can_bridge_msgs micro_ros_agent
source install/setup.bash
cd ../ros2_ws
colcon build --packages-up-to robot_bringup --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

ros2 launch robot_bringup robot.launch.py
```

打开 `http://localhost:8080/workbench/`。默认模型为 `half_body_robot`，模型、Agent 和控制会话在页面中选择。启动时电机保持失能。HTTP 和 rosbridge 监听本机，WebSocket 校验工作台来源。启动参数见 [robot_bringup](ros2_ws/src/robot_bringup/README.md)。

## 控制与设备维护

- 滑块目标通过 `/control/joint_target` 交给连续跟随控制器，动作通过 `/control/trajectory` 交给 JTC。
- `/motor/submit_policy` 提交使能或失能策略，`/control/stop` 停止运动。状态通过关节反馈、TF、IMU 和 `/control/status` 回传。
- 模型切换、停止或断连后，重新操作需要显式使能并提交新目标。会话恢复流程见 [主机控制](docs/architecture/host-synchronized-control.md)。
- 设备页提供只读查询、清错、失能态设零，以及 CERB 方向设置和驱动板参数保存。写入条件和保存范围见 [设备参数](docs/architecture/motor-device-parameter-settings.md)。
- MCU 固件需另行配套部署，使用与主机一致的线上协议版本。W25 电机记录在启动时读取。

控制器、轨迹插值和限位参数见 [motor_hardware_interface](ros2_ws/src/motor_hardware_interface/README.md)。

## 测试

在已加载环境的 `ros2_ws` 目录运行相关包的测试。ROS 集成测试使用独立域。

```bash
ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1 colcon test --packages-select \
  robot_description robot_transmission motor_hardware_interface \
  micro_ros_bridge robot_bringup web_host
colcon test-result --verbose
```

前端单元测试使用 `npm test`，浏览器测试使用 `npm run test:browser`。USB/CAN 延迟、停止距离、机构支撑和掉电保存需要台架测量。

## 资料

[文档索引](docs/README.md)汇总架构、测量报告与硬件资料。[模型说明](ros2_ws/src/robot_description/README.md)介绍模型目录和标定来源。

`build/`、`install/`、`log/`、`.pio/`、`node_modules/` 和网页构建输出均为本地产物。
