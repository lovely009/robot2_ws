# 人形/具身机器人操作界面第一方参考研究

> 状态：参考；第一方公开资料核查日期：2026-10-23。
> 范围：Unitree G1/H1 的公开 APP/开发工具/动作资料、Pollen Reachy 2 Dashboard/VR teleop、Hello Robot Stretch Web Teleop。
> 边界：只陈述第一方页面或官方 GitHub 能直接支撑的内容；“交互建议”是面向 `/home/ly/robot_ws` Web 上位机的推断，不代表厂商内部实现。Foxglove、Rerun、Isaac、MoveIt 由主线程文档覆盖，本文不重复。

## 证据分级

- **公开事实**：官方文档、官方产品页或官方仓库明确展示/说明的界面与流程。
- **交互建议**：从公开流程提炼的设计方向，需结合本项目安全边界重新设计。
- **证据不足**：公开资料没有足够界面细节；不以 SDK API 或营销文案反推闭源平台内部结构。

## 1. Unitree G1/H1

### 已核实公开事实

- 官方 Explore APP 产品页公开列出机器人实时状态、温度/告警、关节与 IMU 校准、逐关节角度调节，以及教程和电子文档入口。
- G1 官方 App 教程覆盖开/关机、网络连接、23/29 DoF 标定、遥控器配对、姿态/动作和调试；H1/H1-2 教程覆盖设备/网络设置、标定以及关节启动/切换。
- G1 官方描述仓库记录 APP 路径 `Device → Data → Robot → Machine Type` 可读取 `mode_machine`，用于区分 23 DoF、29 DoF 等机型模式；这说明操作前确认构型/模式是第一方公开考虑项。
- `unitree_sdk2_python` 示例公开了站立/趴下、速度、姿态、轨迹和特殊动作等接口；资料要求在低层电机控制前先通过 APP 关闭高级运动服务，以避免命令冲突。
- G1 loco client 公开区分连续移动、站立/下蹲等离散任务，以及 Damp、ZeroTorque、Balance、ContinuousGait、挥手、握手等动作/状态操作。
- 官方 `xr_teleoperate` 公开支持 G1 23/29 DoF、H1、H1-2；实机流程包含 VR 第一视角、终端连接状态、初始姿态对齐、按 `r` 启动、按 `s` 记录、按 `q` 退出。

### 可借鉴交互

- 顶部持续展示连接、机型/DoF、运动模式、数据新鲜度和告警；构型不匹配时允许查看动作，但阻止向不匹配对象执行。
- 标定、姿态对齐、遥控器配对做成有步骤和完成条件的检查清单；逐关节调参放在专家入口，并显示目标/反馈值。
- 连续速度控制与站立、下蹲、阻尼等离散动作分区；高风险动作展示目标状态并要求确认。
- “高级运动服务/低层控制所有权”应在界面显式可见，互斥控制通道不可同时抢占。
- VR 或遥操作的连接、姿态对齐、开始、记录、停止做成明确状态机；停止、暂停和控制权接管的语义需由本工程命令契约明确，软件停止按钮不标为硬件急停。

### 适用限制

- Explore APP 为闭源产品，公开页面足以证明状态、告警、校准、逐关节调节和教程入口存在，但没有公开完整线框、内部状态机或每个菜单。
- SDK 方法不是 APP 截图；不得把 `g1_loco_client` API 描述为 APP 的实际按钮或页面。
- 公开资料未证明自动碰撞规避、力控策略或故障恢复细节。

### 官方直链

- Developer Hub：<https://support.unitree.com/home/en/developer/>
- H1 App：<https://www.unitree.com/app/h1/>
- G1 App：<https://www.unitree.com/cn/mobile/app/g1/>
- Python SDK：<https://github.com/unitreerobotics/unitree_sdk2_python>
- G1 loco client：<https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/g1/loco/g1_loco_client.hpp>
- XR teleoperation：<https://github.com/unitreerobotics/xr_teleoperate>
- G1 描述/机型模式：<https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md>

## 2. Pollen Reachy 2

### 已核实公开事实

- 当前 Dashboard 通过浏览器访问 `http://<IP.address>:8000/` 或 `http://<robot.name>:8000/`；旧仓库 README 出现过 3972 端口，不能把它当作当前 Reachy 2 地址。
- Dashboard 分为 Robot、Services、Network、Updates、Visualization、Media：可看机器人信息、关节状态、发送姿态命令、部件 compliance；启停服务并看日志；选择 Wi-Fi/IP；拉取和更新服务；使用 RViz 或 Foxglove 查看 ROS 数据；查看头/躯干相机和音频音量。
- 页面持续显示序列号。官方仓库 README 还明确把 Dashboard 定位为机器人状态总览和快速调试面：已检测电机、运行服务、电机温度、compliance 调整。
- 官方 VR teleop 流程公开为 Unity 应用：按 IP 添加/选择机器人并连接；Mirror/过渡房显示实时相机、状态、帮助和设置。
- 进入前显示机器人当前姿态的半透明绿色覆盖；按住 `A` 进入。新流程先只开放头部和移动底座，确认环境后再次按 `A` 才开放手臂。
- 进入过程中有 3 秒倒计时并降低电机运动速度；按住 `A` 可回到 Mirror 房间/暂停。摘下头显会自动暂停，官方文档仍要求先显式停止 teleop 再摘头显。
- 退出 teleop 会使电机进入 compliance；官方要求先降低手臂，避免失力后跌落或碰撞。

### 可借鉴交互

- 首页同时显示机器身份、固件/服务版本、关节状态、温度、在线服务、网络和媒体健康摘要，并提供进入日志的路径。
- 功能按“本体、服务、网络、更新、数据可视化、媒体”组织，避免把运维操作混入实时运动控制。
- 遥操作采用分级解锁：先相机确认环境与低风险子系统，再开放高影响关节；显示目标姿态叠加、倒计时和限速状态。
- 手动连续输入可采用按住语义，释放或控制端失联时停止该输入；失焦行为需按命令类别定义，不能套用到主机持续节拍或已启动的动作执行。
- 提供部件 compliance、服务日志、更新状态等维护入口，并与实时动作入口使用不同视觉危险等级。

### 适用限制

- 当前 Dashboard 文档公开的是分区和主要能力，公开截图/说明没有穷尽所有控件、默认值或后端内部逻辑。
- VR 文档证明公开 teleop 流程，不能据此断言生产环境具备碰撞规避、力控或未公开安全策略。
- 旧端口、旧版本界面可能仍存在于仓库历史，不能与当前文档混写。

### 官方直链

- Reachy 文档：<https://docs.pollen-robotics.com/>
- Dashboard 发现页：<https://docs.pollen-robotics.com/getting-started/dashboard/discover/>
- Dashboard 文档：<https://docs.pollen-robotics.com/getting-started/dashboard/>
- Teleop 分步流程：<https://docs.pollen-robotics.com/teleoperation/using-application/step-by-step/>
- Teleop 文档：<https://docs.pollen-robotics.com/teleoperation/>
- Dashboard 仓库：<https://github.com/pollen-robotics/reachy-dashboard>
- Teleop 仓库：<https://github.com/pollen-robotics/Reachy2Teleoperation>

## 3. Hello Robot Stretch Web Teleop

### 已核实公开事实

- 官方 `stretch_web_teleop` 使用 ROS 2、WebRTC、Nav2 和 TypeScript，支持 Stretch RE1/RE2/SE3、Ubuntu 22.04、ROS 2 Humble。
- 运行 `./launch_interface`，浏览器访问 `https://<robot-ip>/operator`；可用 `./stop_interface.sh` 停止；同一时刻只允许一个浏览器连接。
- 页面包含广角头相机和夹爪相机、中部 `Base` / `Wrist & Gripper` / `Arm & Lift` 三个标签页、Safety 区运行停止和电量、顶部 action mode 与 `Slowest` 到 `Fastest` 五档速度。
- 广角画面四周提供云台平移/倾斜；快捷视角包括 `Look Ahead`、`Look at Base`、`Look at Gripper`，`Follow Gripper` 可自动让头部追踪夹爪。
- Predictive display 在相机画面上叠加预期运动轨迹；轨迹越长，动作速度相应变化。
- Step Actions 为单击固定距离，Press-Release 为按住运动，Click-Click 为点击启动、再次点击或移出按钮停止。按钮运动时变蓝，到关节限制时变红。
- Movement recorder 可录制并回放单关节或整臂动作；可收听机器人麦克风、选择常用语句并在机器人端播放，也可停止音频。
- 夹爪相机附近提供 `center wrist`、`stow wrist` 等快捷动作。README 明确把 ngrok 互联网远程访问标为原型且有安全问题。

### 可借鉴交互

- 将“看到哪里”“做什么”“何时停”放在同一操作视区：多相机、快捷视角、跟随模式、控制面板和常驻 Safety 并列。
- 连续关节控制同时提供多档速度、预测叠影和实际限位反馈；按钮颜色表达运动与限位，不只依赖文字。
- 明确区分点动、按住运动和点击启停；任一模式切换时先停止当前运动，避免残留命令。
- 单操作者连接应显示当前占用者并提供安全接管流程，不能让两个浏览器静默发送命令。
- 动作录制/回放作为独立工作流，先显示动作范围和预览，再允许在有急停保护的条件下执行。

### 适用限制

- 上述是官方 GitHub `master` README 的公开接口事实，不保证每个已部署版本一致。
- 该 README 没有证明所有机型都在同等验证条件下使用全部功能。
- ngrok 方案是官方明确标注的原型；只能作为“远程暴露需单独威胁建模”的反面约束。

### 官方直链

- Web teleop 仓库：<https://github.com/hello-robot/stretch_web_teleop>
- README：<https://github.com/hello-robot/stretch_web_teleop/blob/master/README.md>
- Hello Robot 文档：<https://docs.hello-robot.com/>

## 对 Web 上位机的合并结论

以下均为**交互建议（推断）**，不是三家厂商的公开实现声明：

| 模式 | 公开样本 | 建议落到本项目 |
|---|---|---|
| 状态总览 | Reachy Dashboard、Unitree APP | 固定显示连接、机型/DoF、模式、数据新鲜度、温度和告警；电量与硬件急停状态仅在有可信数据源时显示 |
| 控制分层 | Unitree loco、Stretch action modes | 连续速度、关节点动、离散姿态、服务运维分域并互斥 |
| 安全升级 | Reachy 分级解锁、Stretch Safety/限位色 | 分级开放控制；手动持续输入按住；动作执行与页面失焦策略分别定义 |
| 空间感知 | Stretch 多相机/快捷视角、Reachy 姿态覆盖 | 相机主视区、目标姿态叠影、快捷视角和可关闭的自动跟随 |
| 可维护性 | Reachy services/logs、Unitree calibration | 标定、服务、日志、更新与运动控制分离，并保留操作记录 |
| 回放 | Unitree XR `s`、Stretch recorder | 录制/回放必须可见预览、关节范围、停止入口和硬件急停前提 |

## 公开证据缺口

- Unitree：闭源 APP 的完整页面结构、控件状态和内部联锁未公开；本文只使用产品页、教程和官方 SDK/tool 仓库能直接证明的内容。
- Reachy 2：Dashboard 当前分区与 teleop 流程已核实，但公开材料不足以证明所有维护控件细节或生产安全策略。
- Stretch：以官方 `master` README 为界面依据，未声称覆盖所有发行版、部署配置和现场定制。
- 三家资料均不足以支持“自动碰撞规避、力控、故障自恢复已经由 UI 保证”的结论；这些能力必须以本项目自己的设计和实测为准。
