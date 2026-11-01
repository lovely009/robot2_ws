# Web 改版与整合回归记录（2026-11-01）

> 后续墙钟时间线定位、生产修正、旧夹具迁移及仍未通过项见 [墙钟跟踪记录](wall-clock-followup.md)。下面保留 UI 交付阶段的检查记录。

UI 已完成本轮改版并通过模拟浏览器验证。整合仍为 WIP，完整 ROS/JTC/原生固件墙钟闭环未通过，不能合入 dev、发布或作为真实设备运行版本。

## 来源和隔离

- 工作树 `/workspace/robot2-integration`，本地分支 `integration/mcu-agent-web`。
- 原工作树 `/workspace/robot2_ws` 保持干净，SHA `525e3a946b30a2fabd0a6b07cf57a7544ddab251`。
- A：`412f19d3f83a018dcf6942f0025914f39e18ba25`；B：`e410243ae2dbe6eca62bc49c3347dfc8fa8d7dbf`。
- 最后 `git ls-remote` 确认 A/B/dev 与上述快照相同。未推送、未修改源分支、未合 dev/main。
- 首次合并为 `3610450b94fe832448a1a1aeccf9838a4875a539`；后续运行时适配为 `bd3864cc76ea8a6f4a8a30a372923117532343de`。
- 本轮所有电机、总线分配、标定与观测数值均为明确标注的测试 fixture；没有接触 USB/CAN/真实 MCU、电机或用户本机。

## 界面改动

1. 中文工程控制台的深色层级、标题、状态、边框、输入与按钮统一；保留调试、日志、IMU、动作和模型诊断能力。
2. 桌面保留对象、工作区、检查器、观察区布局，减少拥挤；窄屏侧栏固定 52px，四个面板标签切换并保留选择与草稿，不产生横向溢出。
3. 独立显示通信连接、本页控制权、MCU 运动授权、反馈新鲜度及使能确认；在线不再使用运动授权成功样式。
4. 加载、失败、重试和未就绪说明可见；目标频率标为 50/50/200 Hz，并明确“整体验收待确认”。
5. MCU 统一入口隐藏主机直连选项；维护请求通过 supervisor 服务，要求所有权、未武装、新鲜失能反馈。REPORT 仍为读取，不自动使能。
6. 防重复控制申请，标准 Action 取消绑定原 UUID，重复取消只发一次；晚到取消确认不会覆盖终态，取消不调用整机停机。
7. 缓存手动目标在第一次发送时固定原始时间戳，重发保留时间戳，不能续命。维护事务保留 request_id 与原始时间戳，受理不冒充设备完成。
8. 3D 渲染最多 30fps，只在状态/视角/尺寸变化时重画，加载完异步 STL 后重新取景；保留重模型及诊断，不删除功能。
9. 参数表使用可访问名称和已有 data 属性，不再生成动态 ID；修复原 B 静态契约失败，同时保留表单可访问性。

## 运行时适配

- 主路径：主机 JTC/MicroRosSystem → supervisor → Agent bridge → MCU → 电机。只导出 MCU 硬件插件。
- AgentCommand AUTHORITY、配置/缓冲轨迹的 lease_id、AgentStatus 租约 ACK 接入 A 的所有权/会话/期限屏障；不再在相同线上话题并列 LowCommand/MotorTrajectory 与 Agent 类型。
- Agent IDL 是整合草案，仍保留 schema_version=3，**不是原 B 的二进制兼容更新**。主机/Agent/MCU 都必须重新生成消息；发布版本号尚需随最终验收定稿。
- supervisor 的标准 Action 由主机 JTC 执行；旧电机轨迹界面转入同一私有主机 facade。直接原始写入口不进入生产路径。
- 使能策略仍显式申请并等待 MCU ACK；撤权先清旧 P0 使能及 P1 槽，再路由输出。所有权续期不等于目标新鲜度。
- 1ms 服务登记已验证的新目标值与原始绝对截止；只在 200Hz 边界发运动/使能帧，避免目标已经到达却等待边界登记造成的空档。没有修改 30/100ms 门限，没有把旧目标重新打时间戳。
- P1 按总线/槽位在内核待发队列 latest-wins，同拍 apply/hold 不再积累重复 37 轴输出。
- 无设备的只读 launch 保持核验与反馈；不强行激活缺少实时反馈的可写硬件。拥有 Agent 时才组合主机控制栈。
- 恢复 A 原始共享型号库；没有猜测真实 37 轴分配或制作实机标定记录。
- MCU 旧缓冲上传仍为完整组装、最多 256 个单电机路点；没有新增滚动轨迹协议。

## 已确认的验证

| 范围 | 结果 | 证据日志（交付包） |
| --- | --- | --- |
| 固件 host Debug | 37/37 可执行测试通过 | `native-admission-tests.log` |
| 原生 37 轴确定性时序 | 20 个源相位、8ms 传输延迟，每轴稳态每秒恰好 200 次物理编码输出 | `native-admission-phases.log` |
| 原始截止与撤权 | 同边界替换不误软停；原始期限到期停止；仅续所有权不能保活旧目标 | 同上 |
| ARM STM32H723 交叉编译 | 成功；RAM 323220/577536，Flash 295256/1048576 字节 | `firmware-admission-cross-build.log` |
| ROS 构建 | 15 包构建通过；后续修改包增量构建通过 | `ros-maintenance-build.log` 等 |
| micro_ros_bridge | 7/7 可执行测试通过，含此前 5 项租约回归 | `bridge-latest-validation.log` |
| motor_hardware_interface | 4/4 可执行测试通过 | `motor-hardware-final-tests.log` |
| supervisor 节点 | 18 项通过 | `supervisor-node-maintenance.log` |
| 网页后端和 Agent 契约 | 合计 143 项通过、2 项跳过，含生成消息满容量 CDR | `backend-and-contracts-final.log` |
| 启动测试 | 15 项通过（显式加载 launch_testing 插件） | `bringup-launch-final.log` |
| 前端 | 100 项单元测试通过，类型检查及构建通过 | `ui-review-final-unit.log`、浏览器构建日志 |
| Chromium 浏览器 | 4 项通过 | `ui-review-final-browser.log` |

浏览器覆盖实际完整模型、1600×1000 与 390×844、四场景导航、面板切换、选择末轴、折叠/展开、连接设置和重连、加载错误及恢复入口、重复申请、MCU ACK 门禁、标准 Action 成功/取消、重复取消和隐藏页面释放会话。控制协议测试单独省略 70MB STL 解码，完整模型由视觉预览测试覆盖。

只执行了上述范围，没有宣称全部仓库测试通过。历史完整 colcon 日志保留了旧接口失败和环境/并发运行造成的失败；后续单独重跑对应检查的结果以上表为准。

## 仍未通过/未覆盖

1. **完整墙钟 SIL 未通过。** 新增 `robot_bringup/test/sil/` 通过 ctypes 执行真实固件 gate/kernel、CERB 编解码、调度及 37 个有状态有界模型，并连接真实 JTC、supervisor、bridge。它没有使用“目标直接复制为反馈”的理想设备。初始夹具缺少参数归一完成状态已修正；工作台误判 JTC-only 控制器集合也已修正。
2. 最新 admission 版 SIL 在主机启动期间仍出现 control stream 超过 300ms，MCU 授权确认未持续建立；先前运行段日志有约 51ms 的运动输入间隔。现有证据不能把所有失效归为一个根因，也不能证明仅修改相位已让完整链路通过。后续应对 ROS 接收、主机控制器读/写与调度停顿做带源时间戳的跟踪；不允许重打时间戳或直接放宽门限。
3. A 的 3 项 `robot_supervisor/test/test_wire_runtime.py` 仍使用旧 LowCommand/LowState 和 MCU Action 夹具；B 的 2 项 `test_host_trajectory_integration.py` 仍绕过 supervisor 的主机直通夹具。它们尚未迁移到最终统一路径，**仍是失败/验收缺口**，没有删除、跳过或删改断言来冒充通过。
4. `robot_motion` 旧 MCU Action 单元测试 38 项通过，但不是新 HostMotionActionAdapter 的完整验收。标准 Action 切换期间取消、失联及最终使能反馈仍需要完整链路补测。
5. 未证明长计划在整个新链路完成；主机可保留完整计划，不代表 MCU 支持执行中续段。
6. USB CDC 12Mbps、真实 CAN 拥塞、WCET、物理停机时间、抱闸、真实 37 轴分配与装配标定均未验收。

原 B 的动态 ID 静态测试与温度固定字符串测试均在精确 B 快照复现。前者通过可访问名称替代动态 ID 修正；后者保留样本时间戳/去重断言并适配实际参数化温度周期，没有改动 IMU 运行逻辑。

## 截图与复核

- `before-desktop.png/mobile.png`：dev 模拟基线。
- `integration-before-desktop.png/mobile.png`：首次整合模拟基线。
- `review-final-desktop.png/mobile.png`：最终本轮改版，完整模型加载后实际浏览器截图。
- 早期改版 PNG 已保存到 Library：桌面 `file_0000000066048230aa4f1b60c5c7bf01`，窄屏 `file_0000000099c881fd875d1ce4466c8b93`。这两张来自已通过的首轮改版，不冒充最新完整模型截图。
- 截图、日志、补丁和本地分支 bundle 归入本轮证据包。所有截图明确标注模拟/非实测/整合未验收。

主要复核命令：

```bash
cmake -S micro_ros_firmware/test/host -B BUILD_DIR -DCMAKE_BUILD_TYPE=Debug
cmake --build BUILD_DIR -j4
ctest --test-dir BUILD_DIR --output-on-failure
cd ros2_ws/src/web_host/frontend
npm test
npm run build
PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH=/usr/bin/chromium npm run test:browser -- test/browser/integration-preview.spec.ts test/browser/managed.spec.ts
```

ROS 测试需加载本轮 Humble 安装，隔离不兼容的全局 pytest 插件；启动测试显式启用 `launch_testing.pytest.hooks` 和 `launch_testing_ros_pytest_entrypoint`。原生 SIL 需先构建 host `robot2_sil`，再显式设置 `ROBOT2_SIL_LIBRARY`，其失败日志必须保留。
