# robot_dynamics 独立依赖边界

**Status**: accepted

## Context

`robot_dynamics` 是 `dev/robot_ws` 七包工作区中的独立运动学/动力学计算包。当前版本只验证 ROS 2 Humble 提供的 Pinocchio 可被 ament_cmake 查找、链接和测试；现有真机控制、独立电机仿真、Gazebo 模型与 Web 上位机均不需要通过该包运行。

若把 Pinocchio 直接加入 `motor_drive`、`motor_hardware_interface` 或 `robot_arm_gazebo`，会把通用计算依赖带入底层驱动、实时控制适配或模型资产包，模糊现有职责边界。

## Decision

- `robot_dynamics` 作为通用机器人运动学/动力学计算入口，当前只直接依赖 Pinocchio。
- `robot_dynamics` 不依赖 `motor_drive`、`motor_hardware_interface`、`motor_sim` 或 Gazebo，也不进入 ros2_control 实时循环。
- 当前不提供 ROS 节点、topic、service、参数、launch 或 URDF 加载逻辑。
- FK、IK、RNEA、重力补偿、轨迹前馈和 URDF 模型加载仅在出现具体需求时增加。
- 后续若控制或仿真包需要消费 `robot_dynamics`，或者依赖方向需要改变，必须另立 ADR。

当前工作区包级边界为：

```text
can_bridge_msgs                         固件与 ROS 2 消息契约
motor_hardware_interface -> motor_drive 真机 ros2_control 适配 -> 无 ROS 底层驱动
motor_sim                               与机器人无关的独立电机仿真
robot_arm_gazebo                        机器人模型资产与 Gazebo 集成
robot_dynamics -> Pinocchio             独立运动学/动力学计算边界
web_host -> can_bridge_msgs             Web 上位机消息
web_host -> robot_arm_gazebo            运行期模型资源
```

## Considered Options

- **直接把 Pinocchio 加入现有控制或模型包**：初始文件更少，但会扩大实时控制或模型资产包的依赖与职责，否决。
- **只保留设计稿，不建立 ADR**：代码可以工作，但当前架构索引无法给出稳定边界，否决。
- **新增独立包（已选）**：多一个最小 ament 包，但依赖和后续 API 有清晰归属。

## Consequences

- `motor_drive` 继续保持 ROS 无关，现有真机、仿真和 Gazebo 运行路径不变。
- Pinocchio 缺失时只影响 `robot_dynamics` 的配置与构建，不提供静默降级或源码下载。
- 后续动力学 API 可以独立测试；跨包集成必须显式记录新的依赖决策。
