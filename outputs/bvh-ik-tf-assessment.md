# BVH → IK → TF：优化评估

基于 2026-10-29 当前源码与有界离线测量；本轮不修改产品代码。标记约定：**现状**为源码直接支持，**推断**为源码组合后的判断，**建议**为候选优化。

## 推荐顺序

先落地同 q 的残差/雅可比复用；播放侧补关节插值与人体/机器人共同时钟；再评估按需多初值和轨迹重定时。关节轨迹 q(t) 是核心产物，ROS TF 与网页模型均由它派生。当前网页已经绕开 ROS TF，优化 TF 发布频率不会直接改善它的动画。

## 结论

- **现状：Web 动作工作台预览不经过 ROS TF。** 服务端自述是“browser preview only, no ROS publishers”（`/home/ly/robot2_ws/ros2_ws/src/web_host/web_host/motion.py:1`），浏览器从 HTTP 接口取 `preview.json`（`/home/ly/robot2_ws/ros2_ws/src/web_host/frontend/src/components/MotionWorkbench.vue:79-90`），预览模块也明确不导入 ROS/store（`/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/motion_preview.js:1-5`）。
- **现状：ROS 预览另走 `JointState`，再由 `robot_state_publisher` 输出 TF。** 预览节点按动作 `dt` 发布 `/mocap_robot/joint_states`（`/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/preview.py:16-23`）；启动文件把该话题接给 `robot_state_publisher` 并配置 `publish_frequency: 60.0`（`/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/launch/retarget.launch.py:24-36`）。官方文档说明该参数是动态 TF 的最大发布频率，动作采样仍由 `joint_states` 驱动，不会凭空生成新动作帧：[robot_state_publisher](https://index.ros.org/p/robot_state_publisher/)。
- **推断：两条预览可独立优化，但同一 Web 进程仍有后台 TF 负担。** `MotionWorkbench` 自身只用 HTTP，但其上级 App 总会启动共享 runtime（`/home/ly/robot2_ws/ros2_ws/src/web_host/frontend/src/App.vue:154-162`），runtime 会加载并连接 `core/ros`（`/home/ly/robot2_ws/ros2_ws/src/web_host/frontend/src/runtime.ts:199-246`），后者无条件订阅 `/tf` 与 `/tf_static`（`/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/core/ros.js:418-419`）。因此动作预览画面不需要 TF，页面连接后却会继续接收并处理 TF。

## 采样与同步

- **现状：Web 显示端把所有人体关节位置、机器人 link 位置和机器人关节位置压到至多约 900 个同索引样本。** 计算代码为 `ceil(N/900)` 跳帧并强制保留末帧（`/home/ly/robot2_ws/ros2_ws/src/web_host/web_host/motion.py:154-179`）；完整关节动作仍写进压缩的 `motion.npz`（同文件 `:143-148`）。240 帧时步长为 1，240 帧全部保留。
- **现状：源骨架、机器人骨架和关节角共用 `indices/times`，三者在同一预览帧对齐。** 同文件在 `:155-180` 用同一索引生成源位置、正向运动学 link 位置和关节角。
- **现状：浏览器按浏览器单调时间推进，并选择 `times[i] <= time` 的最近前序样本，不做插值。** `MotionWorkbench.vue:49-64`；低于约 900 帧时保留全部源样本，因此 240 帧场景不会因显示抽样产生额外阶梯。
- **推断：长片段会被固定 900 帧上限降采样；短片段主要受源帧率限制。** 例如 60 s、60 Hz 的 3600 帧会抽样到约 900 帧；此时不插值会显得跳步。建议先对机器人关节角插值后用 URDF 做 FK；人体世界位置的线性插值只是显示近似，要求骨长不变时应保留局部旋转并做 SLERP + FK。随后再按时长和目标显示 Hz 调整采样，保留载荷预算，避免取消 900 帧上限后长片段载荷失控。
- **现状：ROS 预览每次 timer 都按已运行时间选帧，不按“上次帧号 + 1”。** `preview.py:23` 以源 `dt` 建 timer，`:27-35` 用 `playback_frame(elapsed, dt, …)`；该函数向下取整（`/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/bvh.py:140-141`）。回调延迟时会自然跳帧，而不会累积追赶；重复发布或网络洪峰的代价仍可能很高。
- **建议：ROS 预览把 `JointState` 更新频率限制在 `min(源帧率, 60 Hz)`，并在此频率做插值。** 这样不会向 60 Hz 上限的输出端灌入超过其能力的更新；是否保留超高帧率细节由 IK 侧动作数据决定。无需引入硬实时假设。

## 网络载荷

- **现状：`preview.json` 是无压缩 JSON，且响应为 `Cache-Control: no-store`。** 内容包含完整 URDF、源关节位置、机器人 link 位置和机器人关节位置（`motion.py:172-181`）；发送端只设置类型、长度和 no-store，没有 `Content-Encoding`（同文件 `:253-262`）。
- **推断：预览载荷主体约为 `T + 3×(源关节数 + 机器人 link 数) + 机器人关节数` 个数值/样本，再加一份 URDF 文本。** 依据上述预览字段；这些数组以十进制 JSON 展开，压缩空间通常明显。优先建议对 `preview.json` 做 gzip/br 传输，并把已完成 job 的该文件改为按 job id 可缓存；二者比立即改二进制协议更小。
- **现状：共享 ROS 连接对 `/tf` 使用 `throttle_rate: 0`、`queue_length: 1`。** `/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/core/ros.js:264-269`；所有 transform 都写入 `store.tf`，且每条 TF 消息都会 `markDirty(DIRTY.TF)`（同文件 `:403-416`）。
- **现状：Web TF 新鲜度用浏览器到达时间，丢弃消息 header 时间。** `ros.js:407-414` 写 `Date.now()`；`tf.js` 只以 child frame 取变换并用 `now - tf.at` 算年龄（`/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/core/tf.js:110-114`）。
- **现状：当前关节读数实际来自 `/joint_states`，不是 `tf.js` 的 `jointReading`。** 关节行读取 `store.jointStates`（`/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/components/jointRow.js:83-95`），`arm.js` 只导入 `collectModelJoints`（`/home/ly/robot2_ws/ros2_ws/src/web_host/web/js/panels/arm.js:21,114-115`）；源码扫描未发现 `jointReading` 调用。

## 优先建议

1. **先清掉无消费方的 TF 流量：** 若短期没有“URDF × TF”动态视图，动作屏进入时取消 `/tf` 订阅，或把 TF 订阅改成按面板启用；保留当前关节行依赖的 `/joint_states`。
2. **再压缩预览交付：** `preview.json` 加 HTTP 压缩与按 job 版本缓存；只有测量后仍大，再改 typed array/二进制，避免先改渲染协议。
3. **最后改采样策略：** Web 在载荷预算内按目标显示帧率调整抽样，并让源骨架、机器人骨架和关节角继续共用同一时间轴；浏览器取帧可按 30/60 Hz 光标推进或二分查找，减少每次动画更新时对最多约 900 个样本的线性 `findIndex` 扫描。
4. **ROS 预览限速到有效显示频率：** `JointState` 与 `publish_frequency` 对齐到不超过 60 Hz；人体与机器人 ROS 回放应先共用起点、播放进度和帧时间；若后续要求源时间可追踪，由订阅层保留每个 `TransformStamped.header.stamp`，与单独记录的接收时间区分。

**验收建议（针对后续实现）：** 240 帧保持全部样本；长片段样本数与末帧时间符合目标显示 Hz；动作预览在不连接 ROS 时仍可播放；动作屏无非必要 `/tf` 消息；ROS 预览 120 Hz 输入不再生成超过 60 Hz 的 TF 更新。


## IK：优先减少重复计算，再调整求解策略

当前链路为离线 BVH 源骨架 FK → 躯干局部坐标下的上臂/前臂方向与手部朝向 → 机器人 IK → 逐帧关节角。语义映射、首帧标定和解析雅可比已经存在，应保留。当前样例 3091 帧、74 个含 End Site 的骨架节点、帧间隔 0.016667 s；机器人模型 18 个单自由度关节，左右臂各 9 个。来源：[目标与坐标处理](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/retarget.py:83)、[映射配置](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/config/half_body_robot.yaml:1)、[现有雅可比核对测试](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/test/test_retarget.py:67)。帧数与自由度另经本轮离线加载确认。

1. **先复用同一 q 的 residual/Jacobian 结果。** `residual()` 同时生成两者，`solve()` 的 `fun` 与 `jac` 却分别调用它。仅在单次 solve 内、q 完全相同时复用结果，可以保持现有数学行为；不能跨目标/求解共享缓存。来源：[重复回调](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/retarget.py:113)。SciPy 的 [官方接口](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)也分别接收 `fun(x)` 和 `jac(x)`。
2. **再把固定三个初值改为按需回退。** 现有代码已经用上一帧 `fit` 热启动，但仍无条件计算参考姿态与限位中点两个候选。建议先热启动，仅在任务误差、关节跳变或收敛状态不合格时增加候选；验收必须对比全片方向/手腕误差、分支连续性和失败率，不能只比较速度。来源：[三初值与候选评分](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/retarget.py:146)。当前姿态正则面向参考姿态，上一帧距离只参与候选评分，建议进一步评估把时间连续性直接纳入求解目标。
3. **BVH 只计算 IK 所需节点及其祖先。** 当前每帧遍历所有源节点，旋转经历矩阵→四元数→矩阵转换。保留躯干、肩肘手与祖先闭包，预先解析通道布局；显示完整人体时仍保留完整骨架输出。当前测量中目标生成约占 8.5%，因此优先级低于求解器。来源：[全节点局部变换](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/bvh.py:114)、[源骨架 FK](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/retarget.py:83)。
4. **动作平滑需要时间约束。** 当前在无速度约束的拟合后逐关节裁剪步长，保持原帧间隔，能够满足相邻帧速度限制，但可能产生追赶滞后；未处理加速度/jerk。建议先量化限速触发率、输出相对目标误差与加速度，再按需求选择减速重定时或加入时间约束。离线场景不必要求超出机器人速度能力的动作保持原速。来源：[限速后处理](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/retarget.py:155)、[导出限位/速度验证](/home/ly/robot2_ws/ros2_ws/src/bvh_broadcaster/bvh_broadcaster/retarget.py:180)。

### 本轮有界离线测量

2026-10-29，加载 ROS Humble 环境中的 Pinocchio 4.0.0、系统 SciPy 1.8.0，取仓库演示 BVH **前 240 帧**，分别运行原算法与仅在内存中覆盖的同 q 计算复用版本。初始化不计入计时，两个版本都保留 720 次求解；没有修改产品代码、启动 ROS 节点或发送控制命令。

| 指标 | 原实现 | 同 q 计算复用 |
|---|---:|---:|
| 转换总时间 | 3.765 s | 3.073 s |
| 平均每帧 | 15.69 ms | 12.80 ms |
| residual 调用数 | 22029 | 11594 |
| residual 内累计耗时 | 1.597 s | 0.913 s |
| 源目标生成累计耗时 | 0.321 s | 0.317 s |
| 未收敛候选求解数 | 3 | 3 |

关节角、方向误差、朝向误差三个数组均以 `rtol=0, atol=1e-12` 比较通过；本次总耗时下降约 **18%**。这是一轮局部样本测量，包含轻量计数/计时开销，不是全片性能结论或实时期限保证。原始测量数值暂存 `/tmp/robot2-bvh-ik-profile-20261029.json`；上述表格为持久记录。未收敛数包含被丢弃的候选，不等于失败帧数。

验收建议：缓存优化先通过现有雅可比/限位/速度测试和全片输出等价比较；按需多初值需要单独比较误差分位数与连续性。播放同步则检查人体/机器人帧号与时间戳，性能检查记录耗时分位数；无需为当前离线预览新增实时求解框架或替换 Pinocchio。

本轮验证：仅静态源码核查、上述 240 帧数值等价断言与离线计时；未运行完整回归、浏览器、ROS 回放、全片性能或真机验收。
