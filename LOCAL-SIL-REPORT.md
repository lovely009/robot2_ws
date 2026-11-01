# 本机纯软件验证记录（2026-11-01）

本报告记录测试发生时的本机路径、PID和分支状态，属于历史证据；这些路径不参与可复用脚本的配置。合入整合分支后，不表示默认长时稳定性或遗留断言已通过。

原始日志、截图、JSON时间线和构建产物未加入Git，仍完整保存在本机原验证目录的 `evidence/`、`build-host/`、`runtime/`；`evidence/run-manifest.json`记录原提交和证据哈希。报告中的相对证据链接在原验证目录可读，新工作树须先按复现入口生成证据。`.gitignore`排除这些生成目录；失败证据未删除或改写。

工程和只读网页已在 `ly-System-Product-Name` 启动。短闭环及一次明确条件下的30秒对照通过；默认夹具的长轨迹稳定性仍失败，不满足合入 dev 或真实执行的验收条件。

## 位置与边界

- 独立副本：`/home/ly/Documents/Codex/2026-11-01/task-2/robot2-sil`。
- 分支：`local/sil-validation-20261101`；基线 `403073dd137be3ca5b32cec91d42b1e283c0c82c`。
- 本机源仓库 `/home/ly/robot2_ws` 未修改，仍在 `feat/firmware-agent-topics-v3` / `52e3c79cd39205cf4ee5f28186e1962e82bcf4b9`，工作树清洁。未 reset/clean、push、部署、重建远端旧分支。
- 源仓库 dev 仍为 `525e3a946b30a2fabd0a6b07cf57a7544ddab251`，main 的已知远端跟踪提交仍为 `6d3e92a1f271c339ee4d9afaa7c00b4a0f19c1eb`。本次没有联网更新 Git refs。
- 无仓库 AGENTS.md 或 `.agents/skills` 约束文件。复用仓库 `ManagedNativeFixture`、真实固件 core/gate/CAN 编解码与调度、有状态软件电机、生产 supervisor/bridge/JTC 进程；37轴及其标定、URDF、总线分配均为虚构。
- 未打开真实串口、CAN，未使能真实电机、未刷固件，未改变系统实时策略/驱动。保持原30ms目标及100ms相关安全边界；没有重新盖目标时间戳、放宽超时或引入前瞻协议。

## 当前服务

- HTTP：<http://127.0.0.1:8080/workbench/>。
- WebSocket：`ws://127.0.0.1:9090`。
- ROS 域189，`ROS_LOCALHOST_ONLY=1`。`ss` 确认两个 TCP 端口均绑定127.0.0.1，未开放所有网卡。
- 父进程69694；supervisor 69725、bridge 69726、控制 launch 69727；HTTP 69890、rosbridge 69891。实际子进程完整清单见 [live-processes.json](evidence/live-processes.json)，PID以运行记录为准。
- 留下的是**只读、未武装、无所有者、MCU未授权**的预览；rosbridge禁止发布控制话题/调用控制服务。浏览器确认37张电机卡片、纯软件标识、无页面异常。半身模型仅供几何展示，其18关节映射与虚构的37轴 `j1..j37` 不对应，不可视为整机机构运动验证。
- 实际接口状态见 [live-endpoints.json](evidence/live-endpoints.json)，端口见 [listen-ports.txt](evidence/listen-ports.txt)，截图见 [live-preview.png](evidence/live-preview.png)。文件描述符检查只发现 DDS 的 `/dev/shm`，未见真实设备句柄。

停止本次预览（先校验PID身份，再让父进程清理它拥有的子进程）：

```bash
python3 /home/ly/Documents/Codex/2026-11-01/task-2/robot2-sil/stop-local-preview.py
```

停止后可在此目录运行 `./start-local-preview.sh` 前台重启，Ctrl-C结束。网页静态副本 `runtime/web` 含软件仿真横幅；正式网页 launch 默认 `0.0.0.0`，本次没有使用该默认绑定。

## 实际构建与回归

| 检查 | 本次结果 | 证据 |
|---|---|---|
| ROS 主机/网页/bringup | 15包构建完成；Pinocchio Boost header警告，未阻断构建 | `evidence/bringup-build.log` |
| 固件 host CTest，含原生测量修改后的最终复测 | 37/37通过，0.70秒 | `evidence/host-tests-final.log` |
| 前端类型检查与构建 | 通过 | `evidence/frontend-build.log` |
| 前端单测 | 100/100通过 | `evidence/frontend-tests.log` |
| 模拟浏览器预览/托管操作断言 | 4/4通过 | `evidence/browser-tests-local.log` |
| 仓库原始37轴完整短闭环 | 1通过，6.52秒；到位、同会话第二目标、取消保持使能、断流撤权失能 | `evidence/native-chain.log` |
| 五项原有运行回归 | 3通过、2失败；握手/桥不能独自续权、取消/停机ACK、REPORT命名空间通过 | `evidence/runtime-suite.log` |
| 两项失败的批次启动边界 | 均在启动时300ms控制状态过期，控制器未激活，尚未进入动作断言；启动时序原因未完全定位 | `evidence/runtime-pytest/test_real_jtc_executes_cancels*/control.log` |
| 标准JTC失败用例独立复测一次 | 仍失败，但已走过到位、第二目标、取消、控制器停用和退出；最后总帧数221→222断言失败。新增帧count=0，无新增运动目标，MCU未授权/虚拟驱动失能。原断言未删改 | `evidence/isolated-jtc.log`及对应timeline |
| 正式无Agent未标定启动与真实rosbridge Action/内部写拒绝 | 2通过，4.74秒 | `evidence/launch-web-runtime.log` |
| 实际常驻网页 | HTTP/WS成功；37轴反馈、未武装；页面无异常 | `evidence/live-browser.json` |

未运行：全仓库全部测试、全浏览器套件、ARM交叉编译、真实micro-ROS串口Agent构建/运行、硬件时序与负载验收。未将构建成功当成闭环通过。

首次固件测试36/37的唯一失败是沙箱禁止Unix socket；获准本机执行后37/37通过。npm使用锁文件，下载到任务内node_modules和`/tmp/robot2-npm-cache`，无sudo或系统依赖安装。原浏览器截图硬编码 `/workspace/...` 导致ENOENT，已改为 `UI_EVIDENCE_DIR`/本地默认目录；同一断言复测通过。无需使用Library成果包。

## 固定条件下的调度测量

机器为 i5-12600KF、16逻辑CPU，普通Linux调度，不绑核、不改实时权限。控制器日志显示FIFO策略申请未获许可，本次未调整它。测量时没有同时构建或浏览器负载；常驻预览在测量结束后启动。

原生侧固定缓冲记录每次软件CAN输出的 `steady_clock` 时间、虚拟MCU tick及轴号，容量37×200×60、无溢出。无每毫秒Python样本分配。仍保留仓库DDS时间线与GC观察。**墙钟输出泵调用时间、虚拟tick和真实CAN物理传输是三种不同证据**；本次无物理CAN。

默认循环垃圾回收，预先固定三次10秒目标，均提前ABORT，未达到完整10秒。统计窗口从首次输出后100ms至首次上报SOFT_STOP，包含失效前空档，不混入后续撤权传播/拆进程。

| 批次 | 目标到终态秒数 | GC第二代停顿ms | 模拟服务空档ms | 软件输出墙钟最大间隔ms | 结果 |
|---|---:|---:|---:|---:|---|
| 1 | 7.457 | 11.646 | 12.766 | 14.873 | ABORT，最终失能 |
| 2 | 3.892 | 17.357 | 18.905 | 18.926 | ABORT，最终失能 |
| 3 | 7.739 | 16.326 | 17.240 | 20.240 | ABORT，最终失能 |

三个窗口均覆盖37轴，轴间输出数量完全一致，虚拟输出间隔均为5ms；墙钟暂停后存在追赶，不能用虚拟200Hz掩盖墙钟间隔。各流P50/P95/P99/最大间隔及原始事件见 [measurement-summary.json](evidence/measurement-summary.json) 与 `evidence/measure-{1,2,3}/`。

具体时间线（批次1）：最后一包在tick12072受理，原截止12100；GC于单调时钟3001276170096ns开始，3001287815881ns结束；服务恢复时tick12104已软停，下一包tick12105才受理，自己的原始截止12120未被刷新。主机见到SOFT_STOP后锁住执行并撤权。批次2/3同样关联GC停顿。故当前已证实的初始故障源在Python软件MCU夹具的执行停顿；没有证据把这些停顿统称为CPU节流或真实MCU行为。

另做**一次明确的30秒因果对照**：仅在夹具进程就绪后先做GC、然后暂缓该进程的循环GC；主机生产进程、安全期限、消息时间戳、虚拟tick推进规则全部不变。结束恢复GC。此对照通过，不取代默认环境的失败。

其稳定统计窗口为29.931652秒，37轴各5986次软件输出、无丢轴/溢出/亚毫秒追赶；1496包37轴线上目标全部受理：

| 流/计时点 | 频率Hz | P50间隔ms | P95 ms | P99 ms | 最大ms |
|---|---:|---:|---:|---:|---:|
| JTC主机目标，原始生成时间 | 50.0000 | 20.000 | 20.043 | 20.071 | 20.133 |
| supervisor转发的接收观察 | 50.0003 | 20.003 | 20.317 | 20.906 | 21.348 |
| MCU线上目标受理 | 50.0004 | 20.003 | 20.344 | 20.843 | 21.660 |
| MCU状态发布 | 50.0000 | 20.000 | 20.152 | 20.280 | 20.397 |
| bridge后主机状态接收 | 50.0001 | 20.001 | 20.178 | 20.413 | 20.836 |
| 37轴软件CAN输出，逐轴墙钟统计 | 200.0000 | 5.000 | 5.075 | 5.105 | 5.344 |

无大于3ms的模拟服务空档。37轴最终位置均0.049962446rad，目标0.05rad。主机转发目标年龄P99 2.188ms、最大2.530ms。到位后停止向MCU交付目标，仍保留所有权续期，91.982ms内观察到授权撤销/全部软件驱动失能。这是SIL观察的上界，不是真实电机停机时延或30ms目标到期的精确采样值。

此前每毫秒Python高频采样的三次失败另外保留在 `evidence/python-sampling-{1,2,3}`，不并入上述低开销统计。

## 复现入口与后续边界

在此副本目录加载 `/opt/ros/humble/setup.bash`、`ros2_ws/install/setup.bash`，设置：

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$PWD/runtime/ros-log"
export ROBOT2_SIL_LIBRARY="$PWD/build-host/librobot2_sil.so"
python3 local_sil_measure.py --runs 3 --duration 10 --prefix followup-default
python3 local_sil_measure.py --runs 1 --duration 30 --prefix followup-gc --domain 175 --defer-fixture-gc
python3 local_sil_analyze.py
```

本次仅修改SIL观测代码、浏览器证据路径，新增本地测量/服务脚本和报告，没有生产协议修改。保留默认失败及未到达断言：启动时序、完整工作台6秒/重力/客户端与控制循环退出组合尚未验收。进一步应减少夹具持有的Python对象或把软件MCU收发/时钟移到更代表真实执行边界的原生进程，并保留主动注入停顿的安全回归。一次暂缓GC的30秒成功不能证明普通系统的长期调度上界，也不能批准真实电机执行。
