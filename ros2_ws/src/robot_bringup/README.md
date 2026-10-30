# MCU 整机启动

实机运动统一经过 `robot_supervisor → micro_ros_bridge → micro-ROS MCU → 电机`。主机负责配置核验、控制权、标准轨迹接口和状态展示，MCU 负责轨迹插值、执行和本地授权期限。主机直连 CAN 控制入口已停用。

| 入口 | 用途与默认行为 |
| --- | --- |
| `robot.launch.py` | 默认整机入口，包含托管网页；必须明确 `model` 和 `robot_id` |
| `managed_robot.launch.py` | 相同控制链的无界面入口；`start_web:=true` 可开启网页 |
| `bench.launch.py` | 维护与仿真进程管理、只读网页；不启动电机命令桥、命令节拍或 Agent |
| `firmware_tf.launch.py` | 固件实测关节与 TF，只读；拒绝 `enable_trajectory:=true` |

两个整机入口均默认 `start_agent:=false`、`allow_trajectories:=false`。启动页面、申请租约或恢复进程都不会自动使能电机。

## 构建与版本配套

本次采用线上协议 **v3**，必须配套构建 `can_bridge_msgs`、主机 bridge、监督器与 MCU 固件。旧固件不包含本地控制租约及轨迹/测量会话元数据，不能通过整机就绪检查；不要只更新主机后继续运行旧固件。

在已加载 ROS 2 Humble、已安装工程依赖的终端中，先构建并加载 Agent/接口工作区，再构建主工作区：

首次检出还需在 `ros2_ws/src/web_host/frontend` 执行 `npm ci && npm run build` 生成网页资产，随后再运行下面的 ROS 构建命令。

```bash
ROBOT2_ROOT=/path/to/robot2_ws
cd "$ROBOT2_ROOT/micro_ros_agent"
colcon build --packages-up-to can_bridge_msgs micro_ros_agent
source install/setup.bash

cd "$ROBOT2_ROOT/ros2_ws"
colcon build --packages-up-to robot_bringup
source install/setup.bash
```

`robot_motion` 由 `robot_supervisor` 依赖并加载，不是另一套设备写入进程。公共模型和型号库位于 `robot_description`；真机启动不需要从 `robot_sim` 获取几何。固件使用同一份消息源重新生成接口并构建，刷写与真实机构验收需按部署流程执行。

## 标定记录与配置来源

可使用安装包或显式源码目录。下面使用安装态模型：

```bash
ROBOT_MODEL_DIR="$(ros2 pkg prefix --share robot_description)"
ROBOT_DRIVER_FILE="$ROBOT_MODEL_DIR/models/half_body_robot/config/motor_driver.yaml"
ROBOT_URDF_FILE="$ROBOT_MODEL_DIR/models/half_body_robot/urdf/half_body_robot.urdf"

ros2 run robot_calibration calibration_check fingerprint \
  --driver-file "$ROBOT_DRIVER_FILE" --urdf-file "$ROBOT_URDF_FILE" \
  --model-name half_body_robot --robot-id arm-01

mkdir -p commissioning
ros2 run robot_calibration calibration_check template \
  --driver-file "$ROBOT_DRIVER_FILE" --urdf-file "$ROBOT_URDF_FILE" \
  --model-name half_body_robot --robot-id arm-01 \
  --output commissioning/arm-01.yaml
```

模板默认为未验收，需记录该台机器的总线、方向、零位、限位和保护核对结果。模型迁移改变了 URDF 的资源路径和配置引用，旧摘要不能直接沿用；核对新配置后重新生成验收记录。

```bash
ros2 run robot_calibration calibration_check check \
  --driver-file "$ROBOT_DRIVER_FILE" --urdf-file "$ROBOT_URDF_FILE" \
  --model-name half_body_robot --robot-id arm-01 \
  --manifest-file commissioning/arm-01.yaml
```

示例模型名不代表已经适配任意实机。缺记录、核验不通过或 MCU 状态不新鲜时，只发布未就绪原因，拒绝运动授权。

## 启动整机

不连接设备的检查，可先启动默认入口：

```bash
ros2 launch robot_bringup robot.launch.py \
  model:=half_body_robot robot_id:=arm-01 \
  calibration_file:="$PWD/commissioning/arm-01.yaml"
```

网页默认在 `http://localhost:8080`，ROS WebSocket 端口为 9090。通过 `web_port`、`rosbridge_port` 可改端口；`start_web:=false` 关闭页面。无界面入口等价命令：

```bash
ros2 launch robot_bringup managed_robot.launch.py \
  model:=half_body_robot robot_id:=arm-01 \
  calibration_file:="$PWD/commissioning/arm-01.yaml"
```

需要使用源码模型时，补充 `model_dir:="$ROBOT2_ROOT/ros2_ws/src/robot_description"`。完成实机验收后才显式连接稳定设备路径，并按需要开放轨迹：

```bash
ros2 launch robot_bringup robot.launch.py \
  model:=half_body_robot robot_id:=arm-01 \
  calibration_file:="$PWD/commissioning/arm-01.yaml" \
  start_agent:=true \
  agent_device:=/dev/serial/by-id/REPLACE_WITH_REAL_CONTROLLER \
  allow_trajectories:=true
```

轨迹模式要求本 launch 管理 Agent。关键控制进程退出会结束整组，包括所属 Agent；无自动重启、自动武装或旧目标恢复。网页或 rosbridge 退出不结束健康的控制监督，由客户端租约和 MCU 本地授权期限处理失联。不要让两个 Agent 读取同一设备，也不要同时启动多个整机写入链。

网页操作顺序为：就绪 → 申请控制 → 显式武装 → 提交使能策略 → 等待 MCU 确认 → 提交目标。页面隐藏、离线或状态过期会清除目标并停止续租；返回页面后需重新取得控制权。托管页面的参数改写、零位和保存操作保持只读。

## 标准运动 Action 与租约

| 接口 | 作用 |
| --- | --- |
| `/robot/state`、`/robot/calibration_state` | 类型化整机状态与标定核验结果 |
| `/robot/acquire_control`、`renew_control`、`release_control` | 全机器人独占租约，100～2000 ms |
| `/robot/set_armed` | 显式武装或解除；武装不等于驱动已使能 |
| `/robot/control/command`、`policy`、`trajectory` | 携带租约、epoch、session、时间戳及全局递增 sequence 的意图 |
| `/robot/authorize_motion` | `robot_system_msgs/srv/AuthorizeMotion`，为指定 Action UUID 发放一次性许可 |
| `/robot/follow_joint_trajectory` | 标准 `control_msgs/action/FollowJointTrajectory` |
| `/robot/stop`、`/robot/reset_fault` | 软件停机与满足条件后的故障复位；不会自动恢复运动 |

标准 Action 保持标准消息类型。调用者先申请 TRAJECTORY/TASK 租约、武装并提交策略，待 MCU 确认后，用 `AuthorizeMotion` 将具体目标 UUID 与当前租约绑定，再发送同一 UUID 的 Action。许可最多一秒且仅使用一次。直接运行任意 `ros2 action send_goal` 不会自动取得控制权。

`robot_supervisor.client.ControlClient.follow_joint_trajectory(goal)` 封装 UUID 授权与目标发送；调用者仍需显式续租并驱动 executor。Action 不自动武装或续租。支持的点数、时间、位置轨迹与容差契约见 `robot_motion/README.md`；当前不接受含未适配速度、加速度或力矩数组的规划结果。轨迹由 MCU 插值，成功必须依据匹配回执和新鲜实测到位状态。取消只有获得新的 MCU 停止回执后才返回 canceled，确认超时返回 aborted。

手动目标应至少 10 Hz 产生新意图，并及时续租。监督节点仅在输入新鲜时转发，续租不会无限重放旧目标。控制失效时 MCU 使用本地单调期限撤权，覆盖快照和缓冲轨迹。

```bash
ros2 service call /robot/stop std_srvs/srv/Trigger '{}'
```

软件停机、驱动失能和负载保持是不同操作；停止策略及实测时延仍须结合驱动器、抱闸与机构验收。

## 写入口与观测

bridge 的七个写入口映射到监督器内部通道：

| facade 入口 | managed 名称 |
| --- | --- |
| `/motor/command_array` | `/robot/managed/motor/command_array` |
| `/motor/policy_array` | `/robot/managed/motor/policy_array` |
| `/motor/trajectory` | `/robot/managed/motor/trajectory` |
| `/motor/control_lease` | `/robot/managed/motor/control_lease` |
| `/motor/param_set` | `/robot/managed/maintenance/param_set` |
| `/motor/system_command` | `/robot/managed/maintenance/system_command` |
| `/motor/execute_config` | `/robot/managed/maintenance/execute_config` |

外部客户端使用 `/robot/control/*` 与授权后的 Action，不直接向 managed 或 `/micro_ros_do/*` 写入。维护出口仅由监督器发送只读参数报告请求。话题 remap 和发布者数量检查约束协作进程，部署还需网络隔离或 DDS/SROS2 权限，避免未授权程序直写内部通道。

就绪、故障和拒绝原因通过标准 `/diagnostics` 发布，可接 `diagnostic_aggregator`。实验记录使用 rosbag2，例如：

```bash
ros2 bag record /joint_states /tf /tf_static /diagnostics \
  /robot/state /robot/calibration_state /motor/control_states \
  /motor/motion_states /motor/trajectory_state /motor/events
```

不要记录包含私有令牌的 `/robot/control/*` 输入或授权请求。带控制输出的 bag 仅在隔离环境分析，固件、模型与标定版本应随实验归档。

## 维护与仿真入口

```bash
ros2 launch robot_bringup bench.launch.py
```

此入口启动只读网页和按需台架管理，不启动 Agent、下行编码桥或命令节拍。`firmware_model:=...` 可增加只读 TF。实验进程必须由操作者显式启动；实机运动仍通过整机入口取得授权。旧 `robot.launch.py` 台架行为已迁到此入口，调用脚本应按用途选择。
