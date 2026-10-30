# 整机监督与受控命令入口

本包提供整机启动门禁、全机器人独占控制租约、显式武装、主机故障锁存和受监督的命令出口。MCU 的控制状态仍是电机状态权威；`armed=true` 仅表示主机允许当前持有者提交策略，不表示驱动已经使能或机构已经停稳。

请从 `robot_bringup/managed_robot.launch.py` 启动。`robot.launch.py` 现在同样使用托管 MCU 入口；维护台架改为只读 `bench.launch.py`。托管入口将六个 facade 写接口重新映射，旧网页和旧 `/motor/*` 写入不会自动变成获授权的托管客户端。

## 启动条件

启动时必须给定 `robot_id`、`model_name`、`driver_file`、`urdf_file`。`robot_calibration` 定期提供与本进程配置摘要相同、已人工验收的标定记录。监督器还要求当前 MCU 会话的完整拓扑与配置相符、控制状态持续新鲜、无设备故障；CERB 的设备参数必须核对完成、已保存且设备保护开启。未验收、缺记录、会话变化或旧数据均不能授权。

配置检查和命令校验使用同一套 `robot_transmission`，受触及传动组必须完整提交。运行中改变配置需要重启监督器，不会通过重置故障继续使用旧限位。默认还检查托管出口和 MCU 下行的发布者数量，发现重复控制入口会阻断运行。

## 服务和消息

| 接口 | 类型 / 行为 |
| --- | --- |
| `/robot/state` | `robot_system_msgs/RobotState`，10 Hz、可靠、锁存，不包含租约令牌 |
| `/robot/acquire_control` | `AcquireControl`，申请独占控制权；mode 为 1 手动、2 轨迹、3 任务 |
| `/robot/renew_control` | `RenewControl`，只续当前租约，不自动武装 |
| `/robot/release_control` | `ReleaseControl`，撤权并请求停机 |
| `/robot/set_armed` | `SetArmed`，显式主机武装／解除；不会自动发布使能策略 |
| `/robot/stop` | `std_srvs/Trigger`，无需令牌即可请求软件停机并锁存 |
| `/robot/reset_fault` | `std_srvs/Trigger`，健康恢复且观察到 MCU 已撤权后清除主机锁存，不自动复权 |
| `/robot/control/command` | `LeasedCommand`，手动目标流 |
| `/robot/control/policy` | `LeasedPolicy`，显式增益和使能意图 |
| `/robot/control/trajectory` | `LeasedTrajectory`，第一版受限的单段轨迹 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray`，标准整机诊断及最近输入／拒绝原因，约 1 Hz |

每个输入需当前 `lease_id`、`epoch`、64 位 MCU `session_id`、非零且新鲜的 ROS `header.stamp`，以及跨三个输入通道严格递增的 64 位 `sequence`。默认输入年龄上限为 0.25 秒；租约范围为 100～2000 毫秒，使用单调时钟计时。令牌只由申请服务返回，勿放入公共状态、持久配置或普通日志。

顺序为：启动核验通过 → 申请租约 → 显式武装 → 提交策略 → 等待 `RobotState.mcu_authorized`，提交策略并等待 MCU 确认策略受理；首目标触发 MCU 激活 → 提交目标。`policy_generation` 由监督器按 MCU 高水位分配，客户端值不作为授权依据；高水位跨 Agent 会话保留，32 位代数耗尽后需要确认 MCU 已重置并重启监督器，不能只重启 Agent 来回绕。

首版租约覆盖整台机器人。非空的全禁用策略等价于解除整机武装，清除缓存目标、取消轨迹并请求整机失能；同一包混合使能和禁用会被拒绝。持有者可保留租约，但必须等待停止后的新完整失能回执，再显式武装、使能并提交新目标；快速 disable/enable 不会恢复旧运动。

手动目标需至少 10 Hz 产生新的意图并显式续租。监督器以 100 Hz 向 bridge 出流，但单条输入最多缓存 0.2 秒；客户端停止产生目标或租约到期会撤权、取消轨迹并下发失能。SDK 在 `robot_supervisor.client`，`ros2 run robot_supervisor robot_control --help` 提供状态和服务操作；SDK 不在后台自动续租或重放运动。

## 标准 ROS 诊断

`/diagnostics` 使用 ROS 2 的 `DiagnosticArray`、`DiagnosticStatus` 和 `KeyValue`，可直接接入 `diagnostic_aggregator` 与 `rqt_robot_monitor`。`robot_supervisor/readiness` 在就绪时为 OK、启动核验未通过时为 WARN、故障锁存时为 ERROR，携带当前控制者、模式、MCU 会话、租约剩余时间、标定版本、配置摘要和阻止运行的原因。`robot_supervisor/control_input` 记录最后一次输入是否被接受，并保留最近一次拒绝原因；后续正常目标不会抹掉这项历史。

诊断约每秒发布一次，不随高频目标逐帧发布，也不包含租约令牌。它们描述主机监督和输入接收结果，不是 MCU 到位或实机安全停机回执。专用 `RobotState` 保留整机授权语义；不再提供额外的 JSON 诊断话题。

## 轨迹和停止边界

默认不接受非空轨迹。只有托管 launch 同时拥有串口 Agent 生命周期，才可显式设置 `allow_trajectories=true`：关键进程退出会结束整套 launch 并停止其 Agent，避免仅停止主机快照而留下 MCU 长轨迹继续执行。外部 Agent、恶意 DDS 节点或整个操作系统故障不能由一个主机节点提供安全保证。

第一版只接受完整、同步采样的单段轨迹，最多 256 个电机路点、默认最长 2000 ms，逐轴首点必须为 t=0 并与新鲜实测起点一致。校验包括传动组完整性、线量程、型号约束及 URDF 位置／速度／力矩；具体插值约束以 `validation.py` 为准。标准 `FollowJointTrajectory` 由同进程 `robot_motion` 适配器提供，以新鲜 MCU 轨迹回执和实测反馈判定结果；规划和碰撞检测由上层实现。

软件停机同时请求空轨迹、空快照与逐电机失能，并短时间重发。失能可能使无抱闸机构失去支撑，**不等于保持负载，也不等于硬件急停**；必须完成独立保护和实机验收。主机还会向封闭的维护命名空间发送只读 `COMMAND_REPORT`，以重新取得非周期参数报告，不开放改零位、清故障、改参数等维修操作。

Topic 重映射和发布者数量检查是协作进程的运行约束，不是访问控制。非可信 DDS 参与者仍可能直写 MCU 线话题；需要独立网络隔离或 DDS 安全策略。现有标定、驱动 watchdog、机械限位与电源／抱闸保护均不得由本包替代。

## MCU 本地授权与标准 Action

线上 v3：MCU 控制快照 schema 2、拓扑 schema 3。显式武装后，监督器每 50 ms 请求最长 200 ms 的 MCU 授权，且不超过客户端租约剩余时间。MCU 以本地单调截止时间执行，到期清除缓冲轨迹、目标与使能策略；恢复需新授权及新显式使能。桥不代替监督器续期。`mcu_authorized=true` 仅在当前代数获得新鲜 MCU ACK 后发布。

`/robot/authorize_motion` 接收当前控制租约、全局输入序号、新鲜时间戳及目标 UUID，发放 0.5 秒内一次有效的许可；随后向 `/robot/follow_joint_trajectory` 发送相同 UUID 的标准 Action。SDK `ControlClient.follow_joint_trajectory` 帮助完成这一绑定，不自动申请、武装、使能或续租。Action 运行期间拒绝其它运动输入，停止/解除仍可用；取消等待新的 MCU 失能回执，超时报告停止未确认。成功后 MCU 在有效授权内保留末点，不隐式取消负载支撑。
