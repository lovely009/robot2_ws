# 机器人 Web 工作台参考

状态：参考；2026-10-23 核查第一方公开资料。下文明确区分产品公开能力与本工程建议；不替代已接受的 ADR。

## 参考对象与可迁移设计

| 对象 | 公开设计 | 本工程可借鉴的部分 |
|---|---|---|
| [Foxglove Layouts](https://docs.foxglove.dev/docs/visualization/layouts) 与 [Panels](https://docs.foxglove.dev/docs/visualization/panels) | 按任务保存面板布局、配置与变量；面板可组合 | 真机、动作、仿真各有开箱即用的布局；共享观察组件，支持保存布局 |
| [Foxglove State Transitions](https://docs.foxglove.dev/docs/visualization/panels/state-transitions) | 离散状态按时间显示，并可同步曲线缩放 | 生命周期、故障、命令结果与测量曲线按时间关联，定位异常发生点 |
| [Rerun Viewer](https://rerun.io/docs/getting-started/configure-the-viewer/navigating-the-viewer) | 左侧 Blueprint、中央视图、右侧选中对象属性、底部时间面板 | 对象选择、详情与时间的稳定空间关系；区分实体结构和布局结构 |
| [MoveIt RViz（Humble）](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | 分别显示当前、起始、目标与规划轨迹；可预览轨迹，再执行 | 动作目标和实测姿态分层；编辑、预览和执行有独立状态 |
| [Isaac Sim Gain Tuner 6.1](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html) | 关节表支持选择、批量编辑、单关节详情；结果比较命令与实测曲线 | 调参围绕对象和反馈组织；列/单位随能力变化；批量操作列出实际目标 |

人形与具身机器人厂商公开工具的补充见 [具身机器人面板参考](robot-console-embodied-reference.md)。
这些产品用途不同：Foxglove/Rerun 主要提供观察能力；MoveIt 展示规划执行；Isaac 的整定发生在仿真环境。
它们的界面不能作为本工程具备运动规划、碰撞检查或真机自动整定的证据。

## 设计综合：本工程建议

以下是基于资料和现有代码的设计判断，不是上述产品的共同规范，也不是已实现功能。

- 一个工作台提供三个同级入口：真机调试、动作工作台、仿真与 HIL。
- 会话上下文与工作视图分离：查看哪个页不决定控制哪台设备；实际目标和命令去向持续可见。
- 顶栏显示模型、真实会话模式、连接分层与数据新鲜度；WebSocket 连通、Agent 进程存活和固件有数据分别表示。
- 左侧对象树按模型已有层级和配置展示部件、关节与设备；允许未映射电机和无设备关节独立存在。
- 中央区按场景呈现 3D、设备表或会话拓扑；右侧检查器跟随选中对象，提供状态与对应操作。
- 底部共享观察区承载曲线、事件和命令结果；动作视图提供独立的动作编辑时间轴。
- 观测回看与动作预览不能触发下行。只有明确的执行动作进入命令层；回放模式的控制门由命令层检查。
- 真机调试提供参数对照、使能/增益、目标下发；编辑草稿、发送中、受理、核验、持久化分别表达。
- 动作视图提供资源库、关节分组、关键帧/轨迹、幽灵预览、执行进度；先完成现有动作能力迁移，再评估新增录制和编辑能力。
- 仿真/HIL 显示模型、后端、控制端、传输和端点之间的关系，分别显示进程状态和数据就绪状态。
- 首版提供预设布局、调整宽度和显示/隐藏观察区；全自由停靠、插件市场、多机器人集群管理暂不作为首版目标。

## 技术选型建议

建议采用 Vue 3 + TypeScript + Vite + Pinia；这是针对表单、表格、对象详情和跨视图状态的工程判断，
并非声称被调研产品均使用该技术栈。[Vue 状态管理指南](https://vuejs.org/guide/scaling-up/state-management.html)
推荐用 Pinia 管理新应用的共享状态；[Vite 构建文档](https://vite.dev/guide/build.html)明确支持生成静态托管产物。

| 层 | 选择与职责 |
|---|---|
| UI 与工作区状态 | Vue、Pinia；按连接、台架、设备、动作、观察、布局划分 store |
| ROS 连接 | [roslibjs](https://github.com/RobotWebTools/roslibjs)；独立适配器承担连接、订阅、重连与清理 |
| 3D | 继续使用 Three.js 和 [urdf-loader](https://github.com/gkjohnson/urdf-loaders)，封装生命周期和资源释放 |
| 高频曲线 | 优先验证 [uPlot](https://github.com/leeoniya/uPlot)；样本保存在有界缓冲中，不逐样本触发全页响应式更新 |
| 浏览器验证 | [Playwright](https://playwright.dev/docs/intro)；用假 ROS 连接覆盖断连、重连、切屏、回执和预览隔离 |
| 部署 | Vite 静态产物仍由现有 Python 服务托管；Node 是开发/构建依赖，页面运行不要求 Node 服务 |

具体版本应在实施时结合本机 Node、浏览器和 ROS Humble 环境锁定；本研究不预先安装或修改依赖。

## 对现有边界的影响

现有入口是 `ros2_ws/src/web_host`；工程事实仍以该包 README、源码与相关 ADR 为准。
新架构建议保持 `bench_manager` 的进程编排职责，以及 `command_streamer` 的主机侧命令节拍职责。
不能用浏览器定时器承担控制节拍，不能由页面重建 YAML 拓扑或推断电机协议。

引入构建链将取代 [ADR 0010](../adr/0010-web-host-frontend-layering-without-build-chain.md) 的部分决定，
需要在设计获确认并实施时补充替代 ADR、构建安装规则和测试方式；不应只改 README 后留下相互矛盾的规范。
现有静态 DOM/import 契约测试需要随组件结构调整，模型和动作 HTTP API 的行为测试继续有价值。

TypeScript 只检查编译期，ROS/HTTP 的外部数据仍需要运行时边界校验；协议字段和单位以现有权威契约为准。
操作回执不能笼统显示为“成功”：有事务关联能力的命令按实际回执关联；没有关联能力时明确显示核验限制。
仅向 WebSocket 发布成功，不等于固件受理、设备执行或参数保存成功。

## 不直接复用整套产品的理由

本工程包含设备参数事务、增益策略、动作下发和 HIL 编排，采用专用命令层更符合现有契约。
Foxglove/Rerun 可作为后续外部观察工具评估；首版无需把它们作为整个工作台的运行依赖。
[Foxglove Studio 原仓库](https://github.com/foxglove/studio)已归档，不能据其历史名称假定现行产品源码可直接搬用。
借鉴交互和信息架构；若复用具体源码，需另查对应版本的许可及依赖。
