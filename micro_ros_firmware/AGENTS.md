# micro_ros_firmware —— Agent 操作手册

STM32H723VGT6 micro-ROS 固件（PlatformIO + STM32duino + FreeRTOS，USB CDC）。
本文件只放即用命令、硬约束与踩坑；架构细节看权威文档，勿在此复制：
`README.md`（硬件/时钟/任务/话题/安全级联/开放项）、`lib/README.md`（库分层与各库契约）、
`docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md`、
`docs/architecture/motor-runtime-decision-log.md`（设计理由与变更史）。

## 构建与烧录 / host 测试

`pio` 不在 PATH；固件用 PlatformIO（不能用 colcon），命令均在本目录执行：

```bash
~/.platformio/penv/bin/pio run            # 编译
~/.platformio/penv/bin/pio run -t upload  # ST-Link swd 烧录（已免 sudo）
~/.platformio/penv/bin/pio run -t clean   # 全量清理（不是 clean_microros）

cmake -S test/host -B .host-build && cmake --build .host-build -j2 && \
  ctest --test-dir .host-build --output-on-failure
```

- **栈审计**（可选，改任务栈/加局部大数组后跑）：`platformio.ini` 的 `build_flags`
  临时加 `-fstack-usage` 与 `-fcallgraph-info=su,da`（**勿提交**），`pio run -t clean && pio run`
  后 `python3 tools/stack_audit.py [--orphans 16] [--root <回调名>]`，看完把 flag 撤掉再
  clean 重建。逐任务数字**不含函数指针回调**（ROS executor 那一层），必须用 `--root`
  补上；见 `README.md` 的开放项一节。
- host 测试由 CTest 枚举，直接编译固件真实 .c、无桩无复制；白盒用例 `-I` 内核
  src。新增内核 .c 只改 `test/host/CMakeLists.txt`。
- **必须 `-t clean` 全量重建**：改 `platformio.ini` 构建宏（USB CDC 两宏、
  HAL_FDCAN/IWDG、HSE_VALUE、-Os、FDCAN_SELF_ACK_BENCH）或 `ldscript/*.ld`。
  SrcWrapper HAL 对象不随 flag 重编、增量链接不监听 .ld，不 clean 静默用旧 ELF；
  改 -Os 后还要重测 motor_rt WCET/jitter。
- 勿跑 `-t clean_microros`（联网重下依赖）；libmicroros/ 被删后重建需 GitHub
  可达，本机 .pio 内 repositories.py 的 ghfast 镜像补丁未入库，重下前先确认还在。

## 台架关键顺序

- **CANable 先成为 ACK 节点，再复位/烧录控制板**：SLCAN 顺序
  `C`→`S8`→`Y5`→`M0`→`A1`→`O`（**两只已知 CANable 版本的 5Mbps 索引都是
  `Y5`**；2026-10-12 实测修正了 `16e7497-dirty`=Y7 的旧说法，见
  `docs/guides/fdcan1-bench-guide.md`。间隔 ≥150ms；错误索引会被静默拒绝），
  sim 自动做。`M0`/`A1` 用来清掉前一个抓包工具遗留的 silent/禁止自动重发状态。
  顺序反了 FDCAN1 无人 ACK 进 bus-off（固件自清 INIT 已实测可恢复，但重同步仍需
  ACK 在先）；已 bus-off：停旧进程→重开通道→再复位板子，只重启 agent 没用。
- **烧录前先停模拟器**：MCU 挂起期间总线无人 ACK，模拟器继续发帧会把 CANable
  打进错误态。
- 控制板 CDC 与 CANable 的 `ttyACM` 编号会交换，一律按 `/dev/serial/by-id/` 区分。
- FDCAN2/3 悬空导致的 bus-off 属预期，不算 FDCAN1 台架失败。
- 模拟器在 `../micro_ros_agent/tools/fdcan_motor_sim.py`（默认 CERB 1–4，1=R58、
  2–4=R48；默认 /dev/ttyACM1）；完整九节点台架需显式传入
  `--cerb 1,2,3,4,5,6 --vaxor 7,8,9`。完整流程见仓库根
  `docs/guides/fdcan1-bench-guide.md`。

## 台架：模拟器与全链路（2026-09-22 实测）

- **启动一律走 `micro_ros_agent/tools/bench_up.sh`**（模拟器先起当 ACK 节点 → 再复位板子）。
  默认 `position` 模式，理由见下。
- **模拟器的设备路径按 by-id 解析，别硬编码 `/dev/ttyACM*`**：USB 重新枚举（拔插、HUB
  复位、板子重上电）后编号会交换——2026-09-22 实测 CANable 从 ttyACM1 变成 ttyACM0、
  板子 CDC 变成 ttyACM2；硬编码的模拟器于是开到了**别的节点**，总线上没有 ACK 节点、
  固件 FDCAN1 bus-off，现象是「总线全死」而与设备无关。`--tty` 不给就自动按
  `/dev/serial/by-id/*CANable*` 解析。
- **模拟器是单实例，已加 flock 独占锁**。两个实例同时开同一个 CANable 时，各自都会下发
  `C/S8/Y/M0/A1/O` 并互发帧 → 表现为**随机总线错乱、SYNC 间隔异常、部分电机不跟**，
  极难定位（2026-09-22 就是因为一个忘了停的残留实例，把排查带偏了很久）。第二个实例
  现在会直接失败并说明原因——被挡下时先找残留进程，别怀疑总线。
- **串口读必须与其它工作解耦**：模拟器的读在独立线程（`_reader_loop`）。原先读与
  rclpy/Gazebo 桥共用一个循环，主机负载一上来（Gazebo + 测试脚本）循环周期涨到
  0.1–0.6s → CANable 的 USB 缓冲被写满 → 它写阻塞、控制器出错、**停发 ACK** →
  固件侧表现成 `TEC=128/LEC=ACK error/TXBRP=0xff`、`rx_frame_count=0`、整条总线像死了。
  这台 CANable 对主机侧调度延迟极其敏感：**主机侧程序卡顿会直接表现成"总线故障"**，
  而且报错方向指向"对端不发 ACK"，会把人往固件/CANable 上引。用 cangaroo 单独测一下
  总线是最快的反证。
- **「某台不跟」先查 `motion_states.target_source`，别怀疑力矩**。`0 = NONE` 表示该台
  没过 `entry_may_move` 的门（`policy_enable` / `motion_authorized` 之一；
  `dev_params_allow_motion` 自 2026-10-12 起恒放行，已不再是原因）；此时**快照仍被受理、`snapshot_accepted` 照涨、
  ROS 侧完全看不出是哪台被门住**，只是那台不动（实测 m2/m4 钉在 −π/2 就是这个形状，
  与力矩无关）。`tools/traj_play.py` 在对齐阶段会逐台打印并告警。
- **力矩上限不是本模型的瓶颈**（曾误判并把 effort 抬到 45，已 revert 400df13）。
  用 URDF 自身的质量与几何对势能做数值求导（PE(q) 对关节角中心差分），四关节最大
  静态重力力矩是 **joint_link1 1.87 / joint_link2 0.27 / joint_link3 0.27 /
  joint_link4 0.00 N·m**——活动连杆合计仅约 0.73 kg。原有的 `<limit effort="10.0">`
  比所需大 5–37 倍，**够用**。把 `kp*err` 当成"所需力矩"是拿控制增益当物理负载，两者
  不是一回事。
- **position 模式不可用（模型缺位置 PID）**：xacro 的 `sim_control_joint` 只声明
  `position`/`effort` 两个命令接口，**没有位置 PID 参数**，gz_ros2_control 无增益即
  输出零力矩。实测直接往 `/arm_position_controller/commands` 发 0.5，四关节不动或仅
  重力下垂。**只能用 effort 模式**（xacro 注释的口径也是它）。
- **示教轨迹只有时间与位置**（velocity/effort 恒 0），所以**没有前馈可用**，臂的保持
  完全靠 MIT 的 kp 项——调参时别指望用 `effort` 字段补重力。
- **轨迹容量**：整条轨迹上限 `MOTOR_CONTROL_MAX_TRAJECTORY_POINTS=256` 个路点，且
  **四台电机共用**（4 台 = 64 个时刻）。示教导出的文件动辄 600/1080/1800 点，装不下。
  长轨迹用 `micro_ros_agent/tools/traj_play.py`：主机侧切成 ≤256 点的窗口、在窗口边界
  交棒（下一窗口首点复用上一窗口末点，保证位置连续）。窗口切分可用 `--dry-run` 先看。
- **轨迹播完不释放归属**：内核只在取消/替换时 `release_trajectory_motor`，播完仍由轨迹
  通道持有该电机 → 期间发快照命令**被静默忽略、无任何报错**。要交还给快照通道，显式发
  一条空轨迹取消（web 的"暂停"按钮即此）。

## micro-ROS 坑

- **meta 槽位上限（PUBLISHERS / SUBSCRIPTIONS / SERVICES / STREAM_HISTORY）**：
  这些是 libmicroros.a 的**库编译期**事实，-D 改不了；builder 见 .a 即 early return。
  改 meta 须删 `.pio/libdeps/stm32h723vgt6/micro_ros_platformio/libmicroros/` 和
  `build/mcu/`（保留 `build/mcu/src` 源码缓存）再 `pio run`（可能需联网），否则
  旧 CMake cache 会继续追加旧 `-D` 参数，节点 WAIT_AGENT 无限重连且编译期零提示。
  首选**离线**路径：`../micro_ros_agent/tools/refresh_microros_messages.py`
  （复用缓存、不联网，meta 与 .msg 改动同一趟生效）。
  `tools/check_microros_config.py`（pre: 脚本）构建前校验旧缓存并 Exit(1)。
  当前用量：publishers 10/12、subscriptions 5/6、services 0/5（**参数服务器
  2026-09-21 退役后 services 成纯余量，有意不降**——降它要改 check 阈值，收益只是
  静态数组、不占 DTCM 堆）。
- **改消息后**跑 `../micro_ros_agent/tools/refresh_microros_messages.py`
  增量刷新（复用缓存、不联网），勿 clean_microros。消息唯一来源是
  `../micro_ros_agent/src/can_bridge_msgs`，固件侧只经 extra_packages 符号链接复用。
- **配置事务总线级一问一答**：同一总线任一时刻至多一个已发未答的 SDO 阶段
  （`motor_control_config.c` 的 `bus_stage_in_flight`）；并发发问必表现为
  「回读一直超时」，CERB 阶段超时 30ms——改收发节奏时别删这层串行。
  `kind=7 READ_IDENTITY` 协议内核专用（建档/重连自动发，不经 mailbox；共享映射为
  1..6、8..14）。新增共享 kind 时同步两侧枚举和
  `include/motor_config_contract.hpp` 的穷举断言，并更新 mailbox 白名单、内核门禁、
  `convert_config` 与 host 契约测试；协议专用 kind 不进入共享映射。

## 参数面与内存

- **参数面是两条话题，没有参数服务器**（2026-09-21 起；2026-10-20 起下行为一条
  `config`）：下行 `micro_ros_do/config`（`ConfigRequest`，reliable，`kind` 区分
  `PARAM_SET` / `SYSTEM_COMMAND`），上行 `micro_ros_up/system_status`（`SystemStatus`，
  best-effort 5 Hz）。`ros2 param set/get` 与 `rclc_parameter` 已整体退役；
  参数不再占任何常驻堆（此前 72 槽常驻 23.6 KB DTCM 堆）。
- **`ParamSetTarget` 的 `field` 是设备槽序**（0=pos_min 1=pos_max 2=vel_limit
  3=torque_limit 4=limit_protect 5=direction），由 `src/param_bridge.cpp` 的
  `kFields[]` 显式列出——**不是** `PARAM_FIELD_COUNT` 枚举边界（PID 六项曾因此被
  静默带进参数树）。⚠️ 它也**不是** `param_field_t` 的次序（那里 3 = pos_min）：
  两套里 `field=3` 都合法，互用不报错、只静默写错槽位。
- 位置限位必须**成对**出现在同一条 `PARAM_SET` 消息里；上界 16 目标 =
  `CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY`（有 `static_assert` 守着），
  **改 .msg 上界到 17+ 会立刻编译失败**——那是故意的。
- `field=4`（limit_protect）是**正向语义**（1 = 设备侧保护生效），设备原值相反
  （0 = 保护开）；反转只在 `src/param_bridge.cpp` 的 `submit_safety_protect`
  一处发生，协议层只认原值。
- **写 ≠ 保存**：`param_set` 只改设备 RAM + 固件记录；`system_command` 的
  `COMMAND_SAVE` 才触发设备 `0x2539` + W25 落盘。完成判据是 `system_status` 的
  `save_seq` 前进且 `save_in_progress` 转 false——它覆盖**设备分批下发 + W25 落盘**
  全程（旧实现在落盘前就报完成）。
- `micro_ros` 任务栈是**静态分配**（32 KiB 落 RAM_D1 `.bss`，见 `micro_ros_task.cpp`），
  `motor_rt` 栈仍是动态分配、留在 0-wait DTCM。*（历史：静态化的初衷是给参数槽腾
  DTCM 堆；参数槽已随参数服务器退役，但**这个分配本身保留**——`motor_rt` 的动态栈
  仍在 0-wait DTCM 上，别顺手改回去。）*

## W25 / 存储

- storage 任务（P1/1024 words）是 W25Q64 **唯一所有者**：boot 同步 init 并**顺序
  整读两个槽**——槽 A `0x0000–0x1FFF`、槽 B `0x2000–0x3FFF`（每槽 2 个 4KB 扇区），
  每块整读后过 `param_store_blob_validate()`（CRC 覆盖头+记录）再选 `seq` 新者；
  **刻意不做「只读 32B 头选胜者」的捷径**（未经校验的 seq 会选错槽）。无芯片/ID
  不符静默降级，不阻断启动。
- blob v3 = 32B 头 + 64×120B 记录（7712B）；save 擦**非活动槽的 2 个扇区**、
  写 7712B、按 256B 页读回 memcmp，任一失败不翻转活动槽（旧槽仍可启动）。
- **升级/回滚**：v2（48B 记录）与 v1（32B 记录）加载时各自就地迁移到 v3
  （`migrate_v2` / `migrate_v1`，两条 fan-in 路径，**不是** v1→v2→v3 链）。旧布局
  槽 B 基址是 `0x1000`，新布局下它落在槽 A 擦除范围内——boot 只在**两个新槽都
  无效**时兜底读一次 `0x1000`。**已知缺口（不修）**：旧槽 A 有效、旧槽 B 更新时
  会按旧槽 A 启动，即**静默回退一代**。
  **回滚到旧固件会丢参数**：新固件存过一次后两槽都是 v3，旧固件判两槽皆无效、
  从零开始。
- **上电比对是"变更检测"，不是"保护必须在位"**：基准 = 上次成功落盘时设备里的
  值，所以先关 `0x2650:00` 保护再触发一次保存，记录就被重新基线化成"关"，下次
  上电不再报冲突；v2 迁移上来的记录要等第一次保存才比较这两槽（迁移只搬低 4 位）。
  这两条缺口**已由 2026-09-20 的上电归一实质关闭**（归一在比对之前把保护恒写为
  「生效」）。可选加固方向见 `docs/architecture/motor-runtime-decision-log.md` 的 2026-09-19 条目。
- boot 的「胜者 blob」缓冲与 `g_write_buf` **共用同一块 scratch**（`g_blob_scratch`）：
  `load_boot_params()` 只在 `storage_task_start()` 内、`xTaskCreate` 之前跑一次，
  与 `perform_save()` 无时间重叠；入口 `configASSERT(g_task == nullptr)` 把关。
- **落盘只有一个触发：`system_command` 的 `COMMAND_SAVE`**（2026-09-21 写/保存分离后）。写事务成功
  只把该台 `record_dirty_slots` 置脏（`device_param_report.record_dirty` 随之置真），
  不再是落盘触发；"设备写事务成功 → 500ms 去抖自动落盘"已删（聚合脏标志与
  `motor_control_take_record_dirty()` 一并移除）。只写不保存 = 未保存，下次上电不
  通过——这正是 维护者 要的行为。SAVE 对 dirty 台写设备 `0x2539` 并落 W25。
- Flash 缓冲必须 ≥4 字节对齐（代码统一 `alignas(8)`）：-Os 下结构体访问生成
  STRD，非对齐直接 HardFault。
- 擦写阻塞：v2 单扇区实测 ~1s（**旧值**）；v3 双扇区阻塞值**待台架实测**
  （见 validation plan）。**只在 storage 任务**；IWDG（~500ms）由 motor_rt 每拍
  独立喂狗，调试断点冻结计数。

## 关键路径 / 文件速查

- `src/main.cpp` —— setup 板级初始化 + start_task 编排五任务；`loop()` 必须空。
- `src/motor_rt_task.cpp` —— 唯一电机实时任务：1ms 服务（RX drain / 内核 service /
  SYNC / TX 路由 pump / IWDG 喂狗）+ 2ms/500Hz 控制边界（会话边沿、快照 apply、配置
  FIFO、boot 参数）；内核唯一调用者、唯一正常 FDCAN 发送者。20ms/50Hz 只用于
  协议状态心跳，不是运动控制边界。
- `lib/session_sync/` —— 会话电平 → 逐电机授权的纯派生（host 白盒可测）。
- `lib/motor_control/` —— ID 驱动内核（注册表/型号目录/生命周期/安全级联/配置
  事务）；公共 API 只在 `include/`，内部布局 `src/motor_control_internal.h`
  不向 app 导出。
- `lib/motor_protocol/` —— 唯一 adapter 注册表 + vaxor/CERB adapter、codec 与 P1
  聚合；厂家字节/CAN-ID/FD 属性唯一归属地；事务由静态阶段描述符表驱动。
- `lib/motor_tx_router/` —— P0/P1/P2 有界出队、物理帧预算、P0 重试与 HAL submit seam。
- `lib/save_transaction/` —— 设备保存结果关联与 W25 阶段门控的纯 C 状态机。
- `lib/control_mailbox/` —— 命令快照 latest-wins 双缓冲 + 配置 FIFO（16）+
  会话/授权电平边沿；纯 C、静态内存、不自建锁。
- `src/storage_task.cpp` + `lib/storage_projection/` —— Flash 唯一所有者；
  参数视图 ↔ blob 的纯 C 投影（host 可测）。
- `src/heater.cpp` —— 加热温度 PID 纯函数（HOST_TEST 可测），设备侧归 sensor_task。
- `lib/board/include/board_irq.h` —— ISR syscall 阈值常量（5），板级库唯一门槛事实。
- `src/system_clock.cpp` —— 外设时钟单点（SYSCLK 480M / PLL1Q=SPI2 240M /
  PLL2Q=FDCAN 80M / HSI48=USB）；勿为 FDCAN 动 PLL1Q（连带改 SPI2 速率）。
- 其余：`lib/fdcan`（ISR→SPSC ring 64→P0/P1/P2 TX 调度）、`lib/imu`、`lib/dma`、
  `lib/w25q64`、`lib/ws2812`、`lib/param_store`；分层与依赖方向见 `lib/README.md`。

## 代码规范

- C17/C++17；`-Wall -Wextra -Wpedantic`；2 空格缩进；snake_case（文件/函数/变量）、
  PascalCase（类型）、`k` 前缀常量。
- 中文注释写意图（数据流/单位/安全边界）；协议名（vaxor/cerb/CANopen/MIT/SDO）
  与 ROS 2 API 保留英文。
- `src/` 只放任务编排与 ROS 桥，驱动/协议进 `lib/`；库不得反向依赖 `src/`、ROS
  或其他任务，设备库之间不互相调用（见 `lib/README.md`）。
- **板级库（lib/fdcan、lib/dma、lib/board）不得 include FreeRTOS**；ISR 优先级
  合规只用 `BOARD_ISR_SYSCALL_THRESHOLD`（board_irq.h），main.cpp 有编译期反向断言。
- 固定所有权：FDCAN ISR → SPSC ring → motor_rt 唯一消费者；ROS 桥只写 mailbox、
  经 `motor_rt_copy_*()`/`drain_events()` 读副本，禁止第二任务碰 ring/HAL 发送。
- 新增电机协议：实现 adapter + 静态阶段描述符，注册进
  `lib/motor_protocol/src/motor_protocol_registry.c` 的唯一列表（运行时和协议帧辅助逻辑
  共用；追加新项以保持现有拓扑 protocol 编号），**不改内核收发循环**；型号数值进
  `motor_control_profile.c` 编译期目录，改动同步 host 用例。
- 本平台 `int32_t` 是 `long`，模板（如 std::clamp）须显式指定类型参数。
- 固件不用 printf：`src/syscalls.cpp` 已给强符号空 `_write`，禁止动态日志接 USB CDC。
- 真机只允许 `dev-real` 无负载验证且硬件急停可用；软件 stop 不替代急停/机械限位
  （当前无抱闸）。验证项与 WCET/overflow 待实测条目见 validation plan。

## 架构约束与格式化

- 模块设计遵循**高内聚、低耦合**。所有权、允许依赖和测试边界以
  `docs/architecture/module-boundaries.md` 为准；领域术语与所有权见 `README.md`。
- `lib/fdcan` 只管理协议无关队列；`lib/motor_protocol` 适配器拥有厂商 P1 聚合；
  `lib/motor_tx_router` 选择 P0/P1/P2、核算物理帧预算并提交帧。正常 CAN TX 仍只由
  `motor_rt_task` 发起。
- `lib/save_transaction` 是无 ROS/RTOS/Flash 依赖的 SAVE 状态机；
  `src/param_bridge.cpp` 只提供请求、结果、时钟和错误回调，`storage_task` 仍独占 W25。
- mailbox 与 protocol 的配置 kind 保持独立枚举。新增共享 kind 时同步修改两边枚举、
  `include/motor_config_contract.hpp` 的逐项映射和 host 契约测试；协议专用 kind 不进入映射。
- C/C++ 使用 Uncrustify 0.72.0 和 `.uncrustify.cfg` 规定的格式，空白文件规则见
  `.editorconfig`。提交前执行
  `tools/format_firmware.sh --check`；有意格式化固件源码时执行
  `tools/format_firmware.sh --write`。固件 host Python 测试使用 4 空格缩进，遵循
  `.editorconfig` 的 `*.py` 规则。
