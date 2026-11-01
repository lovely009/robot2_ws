# MCU / Agent 整合阶段检查点（未完成验收）

日期：2026-11-01。此检查点不能合入 dev，不能作为真实设备启动或固件发布版本。
本轮未连接 MCU、电机或用户本机，未烧录、推送、部署或合并到 dev。

## 工作树与来源

- 原工作树：`/workspace/robot2_ws`，本地 `work`，SHA `525e3a946b30a2fabd0a6b07cf57a7544ddab251`，与远端 `dev/robot2_ws` 相同；原工作树保持干净。
- 整合工作树：`/workspace/robot2-integration`，分支 `integration/mcu-agent-web`。
- A：`feat/mcu-platform-integration`，`412f19d3f83a018dcf6942f0025914f39e18ba25`。
- B：`feat/firmware-agent-topics-v3`，`e410243ae2dbe6eca62bc49c3347dfc8fa8d7dbf`。
- 本轮远端复核：A、B、dev 均与上述审查快照相同。A 的 7 提交累积链作为整体纳入，没有重复 cherry-pick 中间分支。
- 原 UI 工作只有临时浏览器脚本，没有页面源码改动；脚本副本及补丁已保全到 `/workspace/robot2-ui-evidence/`，并从原工作树移出。

## 已完成的本轮适配

首次本地合并出现 33 个文本冲突，现已消除文本冲突。保留 B 的 Agent/JTC 实现作为主路径候选，保留 A 的 supervisor、校准、租约、安全库及共享模型。
**消除文本冲突不等于完成语义整合。** 以下跨层阻碍仍然存在。

1. 在 B 的 `motor_rt_task.cpp` 补回 A 的停机屏障调用：内核取消槽位先传到各总线调度器，取消未发的旧 P0 使能和旧 P1 目标；搬运帧保留 `tx_slot`。搬运安排在控制边界之后，避免新产生的停机意图晚一拍才清队列。
2. `motor_control_push_tx` 对同一总线、同一槽位的 P1 在内核待发队列就执行 latest-wins，保持原排队位置。快照应用与同边界保持输出不再积累两份 37 轴命令，避免 40 条搬运预算将重复项带到下一毫秒。
3. 新增 `test_37_axis_schedule.cpp`：真实固件内核、CERB 编解码、CAN 调度/聚合，与 37 个虚构有状态模型共同执行排队回归；跨三路虚构 CAN 分配。测试初始就绪/授权状态是 fixture，不能替代真实开机、supervisor 或 JTC 授权闭环。
4. 适配旧测试 fixture：补 B 的原始绝对截止、运动反馈及 T_max；饱和测试使用不受同轴停机屏障影响的无关帧；撤权恢复必须再次显式使能。未放宽 30/100 ms 安全期限。
5. 默认主路径频率调整：主机 supervisor/JTC 50 Hz、运动/控制状态上行 50 Hz、MCU 电机命令仍为 200 Hz。这些是配置及局部测试结果，不能显示为整机频率验收通过。IMU、温度、Agent 探活等辅助通道不是本次运动频率验收对象。
6. 只导出 `MicroRosSystem`，不重新导出主机直连 `CanFdSystem`；型号库安装继续使用 `robot_description` 的单一来源。移植 bench_manager 包名、依赖及共享模型路径适配。
7. Web 恢复 A 的运行模式读取、内存控制租约、显式申请/武装/MCU ACK 门禁，同时保留 B 的固件参数、配置结果及动作预览能力；HTTP 主机直连 CAN 入口返回 410，不打开 CAN/串口。**B 的实时桥输入仍未完成 A 租约约束，故当前代码不是可用的统一授权后端。**
8. 新增仅使用模拟接口的浏览器预览回归：37 轴信息全部标注非实测，跨四场景导航和重新连接，不产生任何运动发布或控制服务调用。

## 实测测试结果

| 检查 | 本轮结果 | 边界 |
| --- | --- | --- |
| 固件 host Debug CMake/CTest | 36/36 可执行测试通过 | 包括停机屏障、真实内核/CAN 排队回归；不是交叉编译 |
| 37 轴排队回归 | 每轴 200 次/秒，每相邻输出间隔 5 ms，三总线均覆盖 | 虚构分配，50 Hz 输入与反馈；不是实际电机测试 |
| 20/10/5 ms 相位枚举 | 1000 种相位中 120 种出现期限空档，最长 3 ms | **平滑运行验收阻碍**；到期停止正确，不延长安全窗口 |
| 前端 `npm run build` | 类型检查及 Vite 构建通过 | 有既有静态资源外链构建提示 |
| 前端 `npm test` | 99/99 通过 | legacy 测试显式使用 bench fixture；生产 HTTP 不开放 bench 控制模式 |
| 浏览器 managed 回归 | 1/1 通过 | 模拟服务：显式申请、武装、MCU ACK 前拒绝、隐藏后释放 |
| 浏览器 integration-preview 回归 | 1/1 通过 | 模拟 37 轴，导航、重连、390px 截图，零控制调用 |
| ROS Humble 消息生成 | can_bridge_msgs、robot_system_msgs 构建通过 | 容器无硬件设备、无网络，未启动 Agent |
| ROS 主机桥/共享模型构建 | micro_ros_bridge、robot_description 构建通过 | 不包含控制器完整工作区构建 |
| ROS 主机桥 CTest | 6/7 可执行测试通过；72/77 GTest 用例通过 | 5 项 A 租约测试在 DDS 建订阅时失败，见下文 |
| supervisor 纯核心及 MCU 租约 | 67/67 通过 | 独立核心测试，不能证明与 B Agent 主路径接通 |
| ROS Python/Agent/消息合并检查 | 93 通过、2 失败 | 一项 B 已有静态断言失败、一项环境缺少本轮 .pio 库 |

所有测试均为本轮实际执行。最初 host 合并的 3 项失败已通过适配安全语义修复；最初前端 3 项失败为 legacy fixture 未声明 bench 模式，现已修正且全量通过。没有删除失败的租约回归来制造通过。

## 当前阻碍与下一阶段

### 1. 授权和线协议尚未统一（本轮新整合失败）

`micro_ros_bridge/test/test_control_lease.cpp` 的 5 项测试复现：A 的 `MotorTrajectory` 与 B 的 `AgentTrajectoryRequest` 在 `/micro_ros_do/trajectory` 使用不同 DDS 类型，建立订阅报 incompatible type / invalid allocator。

B 的 `AgentCommand`、`AgentConfigRequest`、`AgentStatus` 还没有 A 的 MCU lease fence/ACK 语义。A 的 supervisor 目前仍按 LowCommand/ControlLease 与 MCU 轨迹执行链工作；B 的 bridge/JTC 使用 `/motor/host_command_array`、workbench `/control/*`。仅有 Web 门禁不能保护后端这些写入口。

必须继续将 supervisor 的有效所有权与原始 JTC 新目标时间戳同时接入唯一 Agent 写通道；MCU 门禁必须同时裁决外层租约和底层目标期限，不能在 bridge/supervisor 缓存上重打时间戳。迁移旧线协议和测试必须一起进行，不能并列同名异构接口。此检查点尚未实现这一步，不能启动真实执行链。

### 2. 50 Hz 的时序裕度尚未通过

确定性扫描覆盖主机 20 ms 产生、10 ms 接收、5 ms 控制的所有毫秒相位，发现 120/1000 相位有最长 3 ms 的目标期限空档。测试确认原 source+30 ms 截止没有被 hold 延长；这项测试通过表示安全停止正确，**不表示平滑 50/50/200 运行通过**。需要继续改善接收/边界调度，不能靠放宽期限使验收转绿。

### 3. 仍需构建与验证

- 完整 JTC/MicroRosSystem → 真固件核心/CAN → 37 有状态模型 → 反馈的整条闭环尚未运行。此环境没有前一工作区的 SIL 文件，未把局部排队测试冒充那个闭环。
- ROS 基础镜像未安装 controller_manager/hardware_interface 等完整控制器依赖；本轮只构建已具备依赖的消息、主机桥和共享模型。整机 launch/backend/ROS 全套测试未覆盖。
- 未运行 PlatformIO/ARM 交叉编译；缺少 `.pio/libdeps/.../libmicroros/include/uxr/client/config.h`，因此 wire 最大 CDR/实际流预算测试失败。没有绕过该检查。
- B 原有 `test_temperature_uses_actual_sample_stamp` 静态要求源码包含 `500U`，但 B 已改为参数化周期。本轮在 B 的精确 SHA 导出的源码中复现相同失败（5 通过、1 失败），属于已有测试失配，尚未修改。
- B 自带的 `outputs/host-trajectory-firmware/firmware.bin`、`.elf` 是继承文件，**不是本轮整合构建产物**，不得拿来烧录或作为本轮验收证据。
- MCU 缓冲轨迹仍是完整组装、上限 256 个单电机路点；主机长计划与 MCU 执行中续段能力不能混称。
- USB CDC 12 Mbps 的物理吞吐、CAN 负载、公平性在真实拥塞下的表现、WCET、停机物理延迟、抱闸与实际电机分配均未验收。

### 4. UI 优化仍未实施

已实际运行并查看模拟桌面/390px 页面，保存基线截图。发现：移动端展开侧栏占用大部分宽度，中央 3D/关节/动作区拥挤，字号偏小；部分旧状态文本仍容易与当前控制授权混读。当前只有接口适配与门禁保全，没有完成新的视觉布局或全交互验收，也没有“优化后”截图。

`before-desktop.png`、`before-mobile.png` 是 dev 的模拟基线；`integration-before-desktop.png`、`integration-before-mobile.png` 是本整合检查点的模拟基线。所有电机数值与总线分配均为 fixture，不是用户设备实测。静态页面由浏览器测试临时启动并在结束后关闭，没有公网部署或持续对外服务。

## 复核命令

```bash
# 从整合工作树执行；构建和日志均在仓库之外
PATH=/workspace/robot2-tools/bin:$PATH cmake -S micro_ros_firmware/test/host -B /workspace/robot2-integration-build/firmware -G Ninja -DCMAKE_BUILD_TYPE=Debug
PATH=/workspace/robot2-tools/bin:$PATH cmake --build /workspace/robot2-integration-build/firmware -j4
PATH=/workspace/robot2-tools/bin:$PATH ctest --test-dir /workspace/robot2-integration-build/firmware --output-on-failure
/workspace/robot2-integration-build/firmware/test_37_axis_schedule
cd ros2_ws/src/web_host/frontend
npm run build
npm test
PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH=/usr/bin/chromium npx playwright test managed.spec.ts integration-preview.spec.ts --reporter=line
```

ROS 使用本轮 `robot2-integration-check` 容器（`ros:humble-ros-base`），仅挂载整合源码与构建目录，不挂载任何设备，不开放端口；相关命令及原始日志在交付证据包中。容器依赖、node_modules、构建产物不提交。
