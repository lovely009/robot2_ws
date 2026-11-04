# Robot2 工程

开发主线为 `dev/robot2_ws`。构建成功和隔离测试通过不代表完整闭环与实机验收通过。

ROS 2 Humble 整机平台。实机只有一条电机执行链：`主机 JTC/MicroRosSystem → micro_ros_bridge → micro-ROS MCU → 电机`。主机负责模型、标准运动接口与状态；主路径完整计划由主机 JTC 插值，MCU 负责周期目标执行与本地授权期限。

## 功能与职责

| 包 | 职责 |
| --- | --- |
| `robot_description` | 真机、TF、动力学与仿真共享的模型、传动配置和型号库 |
| `robot_transmission` | 关节与执行器换算，包括差动机构 |
| `micro_ros_bridge` | 主机消息与 MCU 线上协议之间的编码、会话及授权校验 |
| `motor_hardware_interface` | MCU 实测关节反馈与传动映射；官方模拟硬件的离线控制器测试 |
| [robot_sim](ros2_ws/src/robot_sim/README.md) | 仿真、协议级电机模拟和 HIL 支撑（含原 `motor_sim` 库与节点） |
| `robot_bringup` / `web_host` | 托管启动和网页控制客户端 |

模型统一位于 `robot_description/models`，仿真与真机引用同一份资产。`mock_control.launch.py` 使用官方 `mock_components/GenericSystem` 验证离线控制器。

## 构建与启动

MCU 线上协议为 **schema 4**（`can_bridge_msgs/msg/AgentWireContract.msg`）。`can_bridge_msgs`、主机 bridge、Agent 与 MCU 固件必须配套生成消息和构建，不能混用旧二进制。

首次检出时先生成网页资产（在仓库根目录执行，需要 Node.js（^20.19.0 或 ≥22.12.0）及 npm）：

```bash
cd ros2_ws/src/web_host/frontend
npm ci
npm run build
cd ../../../..
```

再构建并加载 `micro_ros_agent` 工作区中的接口与 Agent，然后构建主工作区：

```bash
source /opt/ros/humble/setup.bash
cd micro_ros_agent
colcon build --packages-up-to can_bridge_msgs micro_ros_agent
source install/setup.bash
cd ../ros2_ws
colcon build --packages-up-to robot_bringup
source install/setup.bash
```

不连接设备的启动检查：

```bash
ros2 launch robot_bringup robot.launch.py
```

模型默认选择 `half_body_robot`，可在网页切换。启动不使能电机；Agent 与控制会话由工作台管理。打开 `http://localhost:8080/workbench/`，根路径和旧 `/index.html` 自动跳转。HTTP 与 rosbridge 仅监听本机；WebSocket 只接受当前工作台端口的本机 HTTP 来源，不接受任意网页或无 Origin 的连接。启动用法见 [robot_bringup](ros2_ws/src/robot_bringup/README.md)。

`robot.launch.py` 启动桥接与工作台，`bench.launch.py` 管理模型和仿真会话，`firmware_tf.launch.py` 发布实测关节与 TF。

## ROS 2 接口

模型元数据统一读取 `/api/model-transmissions`；旧 `/api/model-joints` 和 `/api/device-can/*` 接口已删除。

- 手动目标：`/control/setpoint`；轨迹：`/control/trajectory`、`/arm_controller/joint_trajectory`，由 `workbench_control` 校验并交给主机控制器。
- 主机控制器输出：`/motor/host_command_array`；网页显式使能：`/motor/submit_policy` 服务；停止：`/control/stop`。旧 `/motor/policy_array` 仅保留外部兼容入口。
- 状态：`sensor_msgs/JointState`、TF2、`sensor_msgs/Imu`、电机控制状态和 `/control/status`。

模型切换由 `bench_manager` 停止原会话并启动新模型的主机控制栈；使能和新目标均需重新显式提交。MCU 保留限位、故障、命令新鲜度和本地授权期限。桥在显式使能后内部完成 MCU 传输握手；停止、断连或过期后不自动恢复。

主机初次发现反馈最多等待 5 秒，运行期新鲜度保护不变。反馈进程退出或接口持续失效时，管理器先停控，再按冷却间隔最多尝试三次重建；连续新鲜反馈且控制器就绪后才重置重试预算。重建只恢复反馈与控制入口，不恢复使能或旧目标。

运行期系统/电机参数设置和保存已移除。固件不提供 ROS 参数服务，仅在上电只读加载已有 W25 电机记录，不提供擦写或保存；频率改为编译期固定配置。保留电机上电必要归一化、只读查询、故障清除和失能态单电机设零；配置来源为 `micro_ros_firmware/include/`（固定容量与频率见 `firmware_config.h`）。

设零入口为“电机参数 → 选择电机 → 维护操作 → 设零”。先停止主机关节控制并失能该电机，确认当前姿态后提交；收到设备写应答才显示成功，不自动使能或恢复旧目标。设零会改变设备位置限位对应的机械位置，完成后须核对实际反馈与机械行程；不写 W25，也不隐式执行 SAVE，掉电后零位是否保留仍需设备实测。

## 验证

```bash
colcon test --packages-select robot_description motor_hardware_interface \
  micro_ros_bridge robot_bringup web_host
colcon test-result --verbose
```

固件 host 测试位于 `micro_ros_firmware/test/host`，前端测试位于 `ros2_ws/src/web_host/frontend`。模拟测试不接设备；实机控制周期、总线负载、停止时延、抱闸与机构支撑需要另外测量。当前不包含 MoveIt 规划、碰撞规划或足式平衡控制。

## 资料入口

- [文档与硬件资料](docs/README.md)：当前架构、同步测量报告、硬件手册与厂商软件归档位置。
- [模型目录约定](ros2_ws/src/robot_description/README.md)；[半身电机坐标系](ros2_ws/src/robot_description/models/half_body_robot/cad/frames/README.md)。
- [BVH 回放与离线映射](ros2_ws/src/bvh_broadcaster/README.md)；[主机硬件接口](ros2_ws/src/motor_hardware_interface/README.md)。

构建与运行目录 `build/`、`install/`、`log/`，以及 `.pio/`、`node_modules/` 均为本地产物；安装目录可能由正在运行的工作台使用，不作为源码清理对象。
