# 工作区架构事实源收敛设计

- 日期：2026-09-23
- 状态：已实施
- 范围：仅收敛架构文档与决策记录，不改变代码、构建或运行行为

## 背景

`robot_dynamics` 已作为第七个 ament 包加入 `ros2_ws/src/`，并完成 Pinocchio 的查找、链接和 smoke test。然而当前架构入口仍将工作区描述为六包，且 `robot_dynamics` 的边界只记录在已实施但状态仍为“待用户审阅”的设计稿中。由此产生两类漂移：

1. `README.md`、`AGENTS.md`、`CLAUDE.md`、ADR 索引和历史 ADR 的当前适用性说明与实际包清单不一致；
2. `robot_dynamics` 尚未进入权威 ADR 决策链，后续使用者无法从架构索引确定它能依赖什么、不能进入哪些运行路径。

本次只修复已确认的事实漂移，不借机重构现有包、调整依赖或建立推测性的自动门禁。

## 决策

将当前工作区明确为七个 ament 包，并在 `docs/adr/` 新增 ADR 0008，记录 `robot_dynamics` 的独立依赖边界。

当前包级职责与依赖关系为：

```text
消息契约：
  can_bridge_msgs

真机链路：
  motor_hardware_interface -> motor_drive

独立电机仿真：
  motor_sim

机器人模型与 Gazebo：
  robot_arm_gazebo

运动学/动力学：
  robot_dynamics -> Pinocchio

Web 上位机：
  web_host -> can_bridge_msgs
  web_host -> robot_arm_gazebo（运行期模型资源）
```

`robot_dynamics` 遵守以下边界：

- 作为通用机器人运动学/动力学计算入口，只直接承载 Pinocchio 依赖；
- 不依赖 `motor_drive`、`motor_hardware_interface`、`motor_sim` 或 Gazebo；
- 不进入 ros2_control 实时循环；
- 当前不提供 ROS 节点、topic、service、参数或 launch；
- FK、IK、RNEA、重力补偿、轨迹前馈和 URDF 加载仅在出现具体需求时添加；
- 将来若需要跨包集成或改变依赖方向，另立 ADR，不在本次提前设计。

工作区 ADR 编号使用 0008。`docs/adr/README.md` 已引用固件目录中的 ADR 0007；跳过同名编号可避免工作区 ADR 与固件 ADR 在讨论和链接中产生歧义。

## 修改范围

### 新增

- `docs/adr/0008-robot-dynamics-dependency-boundary.md`
  - 状态为 `accepted`；
  - 记录七包结构、`robot_dynamics` 边界、备选方案与后果。

### 更新当前事实入口

- `README.md`
  - 六包改为七包；
  - 包清单加入 `robot_dynamics`；
  - 分支描述同步为七包。
- `AGENTS.md`
  - 工作区摘要和包职责加入 `robot_dynamics`。
- `CLAUDE.md`
  - 工作区摘要、包清单和分支描述同步为七包；
  - 架构总览加入独立的运动学/动力学边界。
- `CONTEXT.md`
  - 分层表加入 `robot_dynamics`，说明它不属于电机驱动实时链路。
- `docs/adr/README.md`
  - 增加 ADR 0008；
  - 当前工作区描述更新为七包。

### 修正当前适用性说明

仅修改下列历史 ADR 顶部描述当前仓库状态的文字，不改历史决策正文：

- `docs/adr/0001-motor-canfd-layering-and-config-single-source.md`；
- `docs/adr/0005-dev-real-branch-and-motor-drive-rename.md`。

### 更新已实施设计状态

- `docs/superpowers/specs/2026-09-22-pinocchio-dependency-design.md`
  - 状态从“待用户审阅”改为“已实施”；
  - 不重写原设计范围、决策或验证标准。

历史实施计划保留当时语境，不因当前包数量变化而改写。

## 不变项

本次不修改：

- 任一 `package.xml` 或 `CMakeLists.txt`；
- C++、Python、JavaScript、launch、YAML、URDF 或控制器配置；
- 现有包依赖和运行路径；
- 真机 fail-fast、安全边界和配置单一来源；
- `motor_drive` 与 `motor_sim` 的协议实现；
- Pinocchio 的使用方式或安装方式。

## 验证

1. 枚举 `ros2_ws/src/*/package.xml`，确认共有七个 ament 包，名称与当前文档包清单一致。
2. 搜索当前架构入口和 ADR 当前适用性说明中的“六包”“6 包”，确认不存在仍描述当前工作区的陈述。
3. 检查 ADR 索引可到达 ADR 0008，ADR 0008 引用的路径存在。
4. 对照 `package.xml`，确认文档依赖图没有声明不存在的包级依赖。
5. 检查 diff 只包含上述文档文件。

不运行 colcon：本次不改变代码、构建文件或依赖声明，构建结果不能为文档一致性提供额外证据。

## 备选方案与取舍

### 增加自动依赖方向测试

可解析所有 `package.xml` 并维护允许或禁止依赖清单。当前依赖图规模很小，测试会形成一份与 ADR 重复的政策数据，而且尚无依赖倒置事故，因此暂不增加。若后续出现错误依赖或包数显著增长，再将 ADR 中的稳定约束转成自动门禁。

### 抽取共享线协议契约层

可让 `motor_drive` 与 `motor_sim` 共用协议实现或 golden vectors，但两者接口方向和配置模型不同；强行共享会改变构建边界，并可能让独立仿真耦合真机栈。本次没有协议漂移证据，不实施。

### 只修改包数量

只把“六”改成“七”无法补上 `robot_dynamics` 的权威决策边界，下一次扩展仍需从历史设计稿推断，因此不足以解决根因。

## 后续触发条件

仅在以下情况出现时扩展本设计：

- `robot_dynamics` 开始提供实际 FK、IK 或动力学 API；
- 需要从 URDF 构建 Pinocchio 模型；
- 控制或仿真包需要消费 `robot_dynamics`；
- 出现违反既定依赖方向的提交，证明自动门禁有实际价值；
- `motor_drive` 与 `motor_sim` 的线协议实现发生可复现漂移。
