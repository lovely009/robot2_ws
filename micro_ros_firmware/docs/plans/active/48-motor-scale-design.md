# 48 台电机规模设计 + 参数服务器必要性评估

> **状态：提案（2026-09-19）；§5 的裁定已于 2026-09-21 执行（见 §5.3 结案块）。**
> **本文里出现的 `PARAM_BRIDGE_MAX_PARAMS` / `kMaxMotors` 与所有指向
> `include/param_bridge.h` 的行号都是史证的**——参数面退役时那个宏连同池子一起删了，
> 链接与行号已失效。§1/§1.1/§1.2 保留当时的推导（它们是"为什么参数服务器在 48 台
> 规模下不成立"的完整论证）。
>
> 本文收敛三件事：①「按总线上限 48 台设计」的容量契约；
> ②只在 48 台才暴露的运行时失效点的机制解法；③rclc 参数服务器的必要性裁定。
>
> **方法**：全平面取证（发现 / 参数 / CAN / 协议 / 主机 / 文档）+ 硬预算核算 + 对抗校验，
> 再由三个独立方案（容量契约 / 既定决定落地 / 运行时骨架）合成。所有数字标注来源：
> **[实测]** 台架实测、**[源码]** 源码可复算、**[推导]** 由前两者推算、**[未验]** 必须台架。
>
> 三笔预算（DTCM 堆 / CAN 总线 / micro-ROS 链路）各自经过一个**独立代理尝试反驳**：
> 三项均未被推翻，但抓出两处算术修正并已并入正文——CAN 帧时漏算 FD 的 CRC 域 4 位
> （53.6 → 54.4 µs，负载 +约 1%）、micro-ROS 消息尺寸需区分"rmw 判容量用的
> `get_serialized_size`"与"真 ucdr 序列化大小"（§3.0）。
>
> **关系**：本文修正 [交接文档](../../../../docs/reports/2026-09-19-firmware-param-surface-handoff.md)
> 的容量数字，承接其已拍板的决定，并补上它未覆盖的 48 台运行时失效点。决定成立后应写入
> [决策日志](../../architecture/motor-runtime-decision-log.md)（交接文档 §四已提示此步未做）。

---

## 0. 结论摘要

| # | 结论 | 依据 |
|---|---|---|
| 0 | **最难的那一关在 ROS 边界，不在固件内部：48 台的消息过不去。** micro-ROS 的 best-effort 流是**一块全局 512 B 静态缓冲**且**不分片**（fragmented 回退对 BE 显式返回 INVALID）→ 单条消息 >≈500 B 结构性地无法收发。48 台时上行 `motion_states`（N≥23 起）与 `motor_status_array`（N≥15 起）**静默消失**；而下行 `MotorCommandArray`（BE 订阅，约 784 B）与 `MotorTrajectory`（约 5 KB）**同样进不来**。**即"能发现、报不出、也命令不动"三件事同一个根因** | §3.0 |
| 1 | **当前 HEAD 跑不起来。** `PARAM_BRIDGE_MAX_PARAMS = 416` 需 136,448 B 而堆池只有 122,880 B；rclc 在 `low_mem_mode` 下按 `max_params` **一次性全量分配且不查 NULL**，失败后第 555 行的循环直接解引用 `data[i].name` → agent 一连上就会踩到 | §1.1 |
| 2 | **参数面每电机实际注册 11 项**，不是常量声称的 5 项——上一提交把 6 个 PID 字段加进了 `param_field_t` 枚举，而注册循环以 `PARAM_FIELD_COUNT` 为界。`kMaxMotors = 81` 是错误门限，416 槽实际只够 36 台，第 37 台起 `rclc_add_parameter` 失败被 `(void)` 静默吞掉 | §1.2 |
| 3 | **48 台真正的约束有四个**：micro-ROS 传输的单条消息上限（§3.0，最硬）、DTCM 堆（唯一稀缺的固件资源）、每总线硬件 RX FIFO0 的 8 深 + ISR 被临界区屏蔽、node 的全局唯一命名空间（朴素三总线接线会锁死 32 台）。**不是**约束的：CAN 带宽（每总线 13~14%）、内核与消息容量（64 刻意保留）、USB CDC 字节带宽 | §3 |
| 4 | **参数树作为配置通道不必要且不可行**：48 台 × 5 项要 250 槽 = 80 KB = 堆池的 2/3，且与板上实际几台无关、永久占用 | §5 |
| 5 | **保留 `system.*` 10 项是划算的惰性选择，不是设计需要**：代价 10.25 KiB（8.5% 堆）+ 5 个 handle，买到 `ros2 param` CLI 与 web_host 现有 rosapi 管线零改动。**参数服务器当初能进来的唯一原因是先落地——全仓库没有一句「为什么用参数服务器而不是话题」的正面论证**。**2026-09-21 owner 已按本文第二步执行整体退役**（含 §5.3 的推演被提前满足：块话题那轮与参数面退役同一批刷新） | §5.3 |

---

## 1. 先修：两个正在生效的缺陷（P0）

### 1.1 `PARAM_BRIDGE_MAX_PARAMS = 416` 算不出来 [源码]

[include/param_bridge.h:56](../../../include/param_bridge.h) `#define PARAM_BRIDGE_MAX_PARAMS 416U`，
而**同一行上方的注释**（:52-53）自己算的是「48×5 + 10 = 250 槽；取 256 留余量」。
416 这个数全仓库**没有任何推导来源**。

**堆池上限**（[实测]，台架记录与链接脚本一致）：

```
DTCM 128 KiB = 131,072 B
堆池 = DTCM − 主栈 8 KiB = 122,880 B
  （.data/.bss 落 AXI/RAM_D1，DTCM 只放可选 .dtcm_noinit + 堆/主栈，
    见 ldscript/stm32h723vgt6.ld:178-213；FreeRTOS heap_useNewlib_ST 与 newlib 同池）
```

**每槽成本 328 B**（[实测] 边际值，两对独立点给出同一斜率）：

```
(heap_free(176) − heap_free(208)) / (208−176) = (12,272 − 1,776)/32 = 328.00 B/槽
(heap_free(256) − heap_free(224)) / (256−224) = (29,748 − 19,252)/32 = 328.00 B/槽
```

**416 槽的后果**：

```
416 × 328 = 136,448 B > 122,880 B   → 必然分配失败
```

而且失败形态不是干净的返回值。`.pio/…/rclc_parameter/src/rclc_parameter/parameter_server.c:543`
在 `low_mem_mode` 下 `allocator.zero_allocate(options->max_params, sizeof(Parameter), …)`
——`calloc` 失败返回 NULL，代码**不检查**，紧接着 :555 的循环写 `parameter_list.data[i].name`：
`i=0` 时地址为 NULL（被 `RCL_CHECK_ARGUMENT_FOR_NULL` 挡下、只记错误码），
`i=1` 时地址为 `0 + sizeof(Parameter)` ≈ 0x70，`RCL_CHECK` 放行 → **写非法地址**。

`param_bridge_create()` 由 [ros_bridge.cpp:124](../../../src/ros_bridge.cpp) 在会话建立时调用，
即**第一个 micro-ROS agent 连上就会踩到**。本板 4 台也测不出缓解——init 按 `max_params`
预分配，与板上实际几台电机无关。

**修法（两步，缺一不可）**：

1. 取值按 §5 的裁定（撤电机参数 → **32**；若暂不撤，则按 `48×5+10 = 250`），并把那段
   五个数互相矛盾的注释整段重写为当前口径。
2. **加一道 pre-flight 拦截**，把「解引用空指针」变成「带诊断的干净失败」：

```c
// rclc 在 low_mem 模式下按 max_params 一次性全量分配，且不对 NULL 做检查
// （parameter_server.c:543/555）——堆不够时表现为写非法地址而非返回错误。
// 这里是唯一拦截点：按实测边际 328 B/槽 预估，不够就拒绝建服并报错。
constexpr size_t kParamSlotBytes = 328U;   // 176..256 四点实测边际
constexpr size_t kParamHeapReserve = 4096U;
if (xPortGetFreeHeapSize() <
    PARAM_BRIDGE_MAX_PARAMS * kParamSlotBytes + kParamHeapReserve) {
  return false;   // 上层 ros_bridge_create 失败 → 重连重试，错误可见
}
```

`xPortGetFreeHeapSize()` 已在 [param_bridge.cpp:743](../../../src/param_bridge.cpp) 用于
`system.heap_free`，无新增依赖。

> **328 B/槽在这里是保守值，不是外推**：它是 176..256 槽区间的边际斜率，含每槽的分配器块开销；
> 槽数更少时每槽占用不会更高。作为"够不够"的下界护栏，偏保守是安全方向（宁可拒绝建服，
> 不可写空指针）。

**撤掉电机参数注册不影响持久化**（P0 容易让人误以为会）：W25 记录由**内核视图投影**喂
（`lib/storage_projection/src/storage_projection.c`，回读在 `storage_task.cpp`），与 rclc 参数树无关；
`system.save` 走的是 storage 任务通知。撤的是"参数树的注册/写入口"，不是设备参数的落盘路径。

### 1.2 参数面每电机实际注册 11 项 [源码]

[param_bridge.cpp:579](../../../src/param_bridge.cpp) 的注册循环：

```c
for (int f = PARAM_FIELD_POS_MIN; f < PARAM_FIELD_COUNT; ++f)   // 3 .. 13 = 11 项
```

而 [param_field.h:18-47](../../../lib/param_store/include/param_field.h) 的 `PARAM_FIELD_COUNT = 14`
——上一提交（`0a34618`）把 6 个 PID 字段（`CURRENT_P..POSITION_I`）追加在 `COUNT` 之前，
于是**它们自动进了参数树**。`view_field()`（:504-535）对它们走 `default: return NAN`，
所以是 6 个恒定 NaN、不可写、纯占槽的参数项，还会出现在 `ros2 param list` 与 web 参数页上。

派生后果：

```
kMaxMotors = (416 − 10)/5 = 81        ← 用的是意图口径，恒真
实际可容台数 = floor((416 − 10)/11) = 36
第 37 台起 rclc_add_parameter 返回错误，被 (void) 吞掉 → 静默无参数面
```

**修法**：把「注册哪些字段」变成显式集合，而不是靠枚举边界：

```c
static const param_field_t kRegisteredFields[] = {
    PARAM_FIELD_POS_MIN, PARAM_FIELD_POS_MAX, PARAM_FIELD_VEL_LIMIT,
    PARAM_FIELD_TORQUE_LIMIT, PARAM_FIELD_LIMIT_PROTECT,
};
#define PARAM_MOTOR_FIELD_COUNT (sizeof(kRegisteredFields)/sizeof(kRegisteredFields[0]))
```

三处循环（注册 :579、初值 :586、以及影子同步）统一遍历它；`kMaxMotors` 与
`_Static_assert(PARAM_MOTOR_FIELD_COUNT == 5)` 同时变真。PID 六项归块话题（§5.3），
不再进参数树。

> 副作用：web 上位机若硬编码列了 `m<id>.current_kp` 之类的名字，会从「读到 NaN」变成
> 「参数不存在」，需同步检查。

### 1.3 五处常量漂移 [源码]

同一个数在多个地方有多个值，任何一份文档单独看都对，合起来对不上：

| 出处 | 说的值 |
|---|---|
| `include/param_bridge.h:56`（实际 `#define`） | **416** |
| `include/param_bridge.h:52-53`（同文件注释） | 256（覆盖 49 台） |
| `src/param_bridge.cpp:71-74`（注释） | 176 / 33 台 / heap_free 12,328 B |
| `docs/adr/0007…md:26` | 208（覆盖 48 台上限） |
| `docs/plans/active/motor-runtime-validation.md:33` | 176（覆盖 41 台） |
| `AGENTS.md:72-73` | 224（覆盖 42 台） |
| `README.md:274,283-285` | 208 / 每电机 4 项 / 堆池约 116 KiB |
| `docs/plans/active/w25-blob-v3-design.md:366` | 176（41 台） |

收敛方式见 §4 的 P1：**单一来源 + 派生**，而不是再改一遍数字。

---

## 2. 「48 台」到底是什么口径

三个口径并存，且都是有意的（[决策日志 §6.10](../../architecture/motor-runtime-decision-log.md)）：

| 口径 | 值 | 出处与约束力 |
|---|---|---|
| 架构容量 | **64** | ARD-0007 §8.1 的 MUST（注册表容量至少 64 个电机实例，编译期固定）；被 `MotorCommandArray` 等消息数组继承。**刻意不跟 48 收**——AXI 还剩 127.9 KB 不是稀缺资源，砍到 48 只省约 4.6 KB 却要动每一处 `static_assert` 与消息边界 |
| 设计上限 | **48** | 决策日志 §6.10：「48 台上限、实际部署约 40 台」 |
| 实际部署 | **约 40** | 只用于文档与台架目标，**不得当尺寸基线** |

**「16 台/总线」不是协议上限也不是时序上限**，唯一出处是 P0 容量的算术反推
（[can_tx_scheduler.h:31-33](../../../lib/fdcan/include/can_tx_scheduler.h)：「48 台分摊到 3 条总线是
16 台/总线，正好等于容量」）。协议真实节点空间是 vaxor 1..60 / CERB 1..127，
固件里**没有任何按总线限 16 的校验**。

> 这是本文与 §6.10 的**唯一分歧**：本文保留「16/总线」作为 **P0 容量与 node 分配表的推导依据**，
> 但要求把它写成显式假设（`kMotorsPerBus`）而不是散落的注释算术；且不新增按总线的运行时强制
> （现场总线分配不均可接受，代价是 P0 撤权需多拍，已在 §3.2 记录）。

---

## 3. 资源账：哪些是约束，哪些不是

### 3.0 micro-ROS 传输的单条消息上限 —— 最硬的一关 [源码]

这是"48 台发现得到、却报不出来"的根因，也是本文认为**优先级高于参数面**的一项。

```
UXR_CONFIG_CUSTOM_TRANSPORT_MTU = 512            （libmicroros/include/uxr/client/config.h:60）
  且 microros_user.meta:11 设 RMW_UXRCE_TRANSPORT=custom
best-effort 输出流缓冲 = 全局**一块**定长静态数组：
  uint8_t output_best_effort_stream_buffer[RMW_UXRCE_MAX_TRANSPORT_MTU]
  （rmw-microxrcedds/rmw_microxrcedds_c/src/rmw_microros_internal/types.h:91）
  在 rmw_init.c:316 用 uxr_create_output_best_effort_stream() 建一次，**所有 publisher 共用**
```

发布路径（`rmw-microxrcedds_c/src/rmw_publish.c:57-70`）：

```c
if (uxr_prepare_output_stream(...) ||
    uxr_prepare_output_stream_fragmented(...)) { written = cdr_serialize(...); }
if (!written) { ret = RMW_RET_ERROR; }        // ← 唯一的回退是 fragmented
```

而 fragmented 回退对 best-effort 流**直接拒绝**
（`Micro-XRCE-DDS-Client/src/c/core/session/write_access.c:222-226`）：

```c
if (stream_id.type == UXR_BEST_EFFORT_STREAM || stream == NULL) {
    return UXR_INVALID_REQUEST_ID;            // 分片只对 reliable 流有效
}
```

**结论**：best-effort 话题的单条序列化消息超过约 500 B（512 − 子消息头/WRITE_DATA 开销）
就**结构性地发不出去**，而且**失败是静默的**——固件对 `rmw_publish` 的返回值一律 `(void)`
（`src/motor_bridge.cpp` 各处），`MotorDiagnostics` 里也只有 `tx_dropped` / `p0_retry` /
`event_dropped`，**唯独没有出站消息丢弃计数**。

**更要紧的是这是双向的**。下行命令面撞的是同一堵墙：
`MotorCommandArray` 与 `MotorTrajectory` 的订阅 QoS 是 `qos_stream()`
（[motor_bridge.cpp:453,464](../../../src/motor_bridge.cpp)，定义在 :137-144：BEST_EFFORT + depth 1），
而输入侧 best-effort 流同样不分片（`uxr_create_input_best_effort_stream`，缓冲同受 MTU 约束）。
**48 台的命令快照（约 788 B）因此也进不来**——不是"下发被忽略"，是执行器根本收不到。
即：**48 台"能发现、报不出、也命令不动"**，三件事同一个根因。

48 台下的消息大小（两种口径，判容量用的是前者）：

| 话题 | rmw 用于判容量的 `get_serialized_size`（含对齐重复计数） | 真 ucdr 序列化大小 | 发不出去的起始台数 |
|---|---|---|---|
| `motion_states`（100 Hz） | 12 + 21·⌈N/2⌉ + 23·⌊N/2⌋ = **1,068 B** | 20N + 13 = 973 B | **N ≥ 23** |
| `motor_status_array`（20 Hz） | 12 + 34N = **1,644 B** | 32N + 14 = 1,550 B | **N ≥ 15** |
| `motor_topology`（1 Hz） | 约 1,358 B（型号名 10 字符） | 同量级 | 走 **reliable** 输出流（`qos_reliable(1)`，:510），上限 = MTU×history = 2048 B，**不受 512 限制**；但分片发布要 flush 并阻塞 micro_ros 任务，且 48 个真实型号名长度未核实［未验］ |
| `MotorCommandArray`（下行，**订阅**） | 12 + 48×16 = 784 B | — | **同样发不进来**：订阅 QoS 是 `qos_stream()`（BE，[motor_bridge.cpp:453](../../../src/motor_bridge.cpp)），BE 输入流不分片 |
| `MotorTrajectory`（下行，订阅） | 48 台受控 + 256 点 ≈ 5 KB | — | 同上（BE 订阅），**远超上限** |

**闸门取哪个数**：`rmw_publish` 先调 `functions->get_serialized_size(ros_message)` 拿
`topic_length`，再拿它去 `uxr_prepare_output_stream(...)` 判容量——所以**过线与否由
`get_serialized_size` 决定**（它因逐元素对齐重复计数而偏大），上表的起始台数按它算。
真 ucdr 值只影响"实际能不能塞进 512"，两者都 >512，判决不变。

**修法（推荐抬 MTU，而不是分页）**：

1. `UXR_CONFIG_CUSTOM_TRANSPORT_MTU` 512 → 2048（`microros_user.meta` 的构建宏 + 重生成
   libmicroros，用 `tools/refresh_microros_messages.py` 的既有流程，勿跑 `-t clean_microros`）。
   代价是这块 best-effort 缓冲与 reliable 流的静态缓冲随 MTU 线性增长（AXI/RAM_D1，不占 DTCM 堆），
   量级 +几 KB，**必须复核 agent 侧**（本仓库的 `micro_ros_agent` 是源码工作区，可同步改）。
   **抬 MTU 一次解决上下行两向**，是本文推荐它的主要原因。
2. 下行若不想抬 MTU：把 `MotorCommandArray` / `MotorTrajectory` 的订阅 QoS 从 `qos_stream()`
   改成 `qos_reliable(1)`——reliable 输入缓冲是 `RMW_UXRCE_MAX_INPUT_BUFFER_SIZE`
   （MTU×history = 2048 B），784 B 的快照放得下，且输入可靠流支持分片。代价是命令面失去
   "latest-wins 无积压"语义（ARD §34 的立意），需与内核的 freshness 机制一起复核。
   **这两条不是二选一**：抬 MTU 是根因修复，QoS 改动是可选的加固。
3. **把丢弃变可见**：检查 `rmw_publish` 返回值并计入诊断（与 §4 P2 的 `rx_hw_lost` 一并加），
   否则 48 台上线后第一个现象是"话题还在、就是没数据"。
4. 分页（按 N 台切多条消息）作为备选：需要同时抬 `RMW_UXRCE_MAX_PUBLISHERS`
   （现 10 槽、已用 9 槽，见下），且上位机要重组——比抬 MTU 侵入得多。

**顺带**：`RMW_UXRCE_MAX_SERVICES = 5` 恰好被参数服务器的
get/get_types/set/list/describe 吃满；`RMW_UXRCE_MAX_PUBLISHERS = 10` 已用 9
（6 电机 + 2 IMU + 1 `/parameter_events`）**只剩 1 个空槽**。这两条同时说明：
任何"按总线拆话题""加一条诊断话题""加一个自定义服务"的改动都会在 init 阶段撞墙——
**而退役参数服务器恰好释放 5 个 service 槽 + 1 个 publisher + 1 个 executor handle**（§5.3）。

### 3.1 DTCM 堆 —— 唯一稀缺的固件资源 [实测]

```
池 = 122,880 B（物理不可扩：FreeRTOS 与 newlib 同池）
当前（224 槽）heap_free = 29,748 B
非参数常驻堆 = 122,880 − 29,748 − 224×328 = 19,660 B
  ├─ FreeRTOS 动态任务栈 = motor_rt 4 KiB + storage 4 KiB + sensor 2 KiB + led 1 KiB = 11,264 B
  └─ 其余约 8,396 B：rcl / executor / transport 的启动分配
参数槽可用预算 = 122,880 − 19,660 = 103,220 B = 314 槽
```

| 方案 | 槽数 | 参数面占用 | 建服后 heap_free | 判定 |
|---|---|---|---|---|
| 当前 HEAD | 416 | 136,448 B | **负 33 KB** | **不可行**（§1.1） |
| 保留每电机 5 项 | 250 | 82,000 B | 21,220 B | 可行但余量薄（最坏瞬态未测），**且与既定决定相反** |
| 保留每电机 11 项（现状口径） | 538 | 176,464 B | 负 | 不可行 |
| 撤电机参数、只留 `system.*` | 32 | 10,496 B | **92,724 B** | **可行，余量充裕（既定方向）** |

### 3.2 CAN 总线 —— 负载不是约束，**中断余量**才是 [推导+源码]

```
单帧时间（标准 ID，含 IFS/ACK/EOF 与 CAN FD 的 CRC 域；1M 仲裁 / 5M 数据）
  通式 t = 30 bit@1M + (5 + 8N + CRC + 4) bit@5M
    30 = SOF1+ID11+RRS1+IDE1+FDF1+res1+BRS1 (17) + CRCdel1+ACK1+ACKdel1+EOF7+IFS3 (13)
    数据段 5 = ESI1+DLC4；CAN FD 的 CRC 域 = CRC(17 或 21) + stuff count 3 + parity 1
    CRC = 17（N≤16）/ 21（N>16）
  12B FD/BRS 帧（vaxor 0x501/0x701、CERB 0x110/0x300）= 30 + (5+96+17+4)/5 = 54.4 µs
  20B FD/BRS 帧（vaxor 0x702）                        = 30 + (5+160+21+4)/5 = 68.0 µs
16 台 CERB 共 SYNC 触发的上报突发（每总线）= 16 × 54.4 µs = 870.4 µs
每总线稳态负载 = vaxor 14.14% / CERB 13.29%（含位填充最坏约 17.7% / 16.6%）
  （早期算式漏了 CRC 域的 4 位，得到的 53.6 µs / 13.95% 偏低约 1%）[推导]
```

**失效点在别处**：每总线硬件 RX FIFO0 只有 **8 深**
（[fdcan.c:68](../../../lib/fdcan/src/fdcan.c)，且 `RxFifo1ElmtsNbr = 0` 即 FIFO1 完全没用），
而 CERB 由**每总线同一条 20 ms SYNC** 触发上报。8 深 × 54.4 µs = **435 µs** 的排空窗口
——ISR 只要在 435 µs 内跑起来就不丢；超过就是 `FDCAN_IT_RX_FIFO0_MESSAGE_LOST`，
**帧不可恢复**。

而 ISR 恰好在会被屏蔽的位置 [源码]：

```
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5             （FreeRTOSConfig_Default.h:207）
HAL_NVIC_SetPriority(it0/it1, BOARD_ISR_SYSCALL_THRESHOLD=5) （fdcan.c:144-146；board_irq.h:9）
→ FDCAN 接收中断的优先级数值正好等于 FreeRTOS 的屏蔽阈值
→ taskENTER_CRITICAL() / portSET_INTERRUPT_MASK_FROM_ISR() 期间 CAN 接收被屏蔽
```

**关键机会**：RX 回调体内**不调用任何 FreeRTOS API**（[can_bsp.c:190-217](../../../lib/fdcan/src/can_bsp.c)
只用 HAL 与 `can_rx_ring_push_from_isr`），所以它**有资格**被抬到阈值之上——移到优先级 4，
临界区就再也屏蔽不了 CAN 接收，8 深 FIFO 的窗口从「整个临界区时长」缩回「ISR 自身延迟」。
这是零成本的现成修法（详见 §4 P2）。

### 3.3 发现与就绪 —— 串行，但典型值可接受 [推导+源码]

每总线严格一问一答（`bus_stage_in_flight`，[motor_control_config.c:358-371](../../../lib/motor_control/src/motor_control_config.c)），
每拍（1 ms）至多推进一个阶段、至多发起一台的识别/回读；单阶段超时 30 ms、最多重试 2 次。

```
每台阶段数 = 识别 1（0x1008）+ 设备回读 REFRESH 6（0x607D:01/:02、0x6080、
             0x2538、0x2650:00、0x2541）= 7
每总线 16 台 × 7 阶段 × ≥2 拍 ≈ 224 拍 ≈ 0.22 s          （典型，三条总线并行）[推导]
每总线最坏（每阶段都重试耗尽）≈ 6 × 3 × 30 ms × 16 ≈ 8.6 s（+识别 4.3 s ≈ 13.0 s）[推导]
```

**当前无优先级**：`config_step` 按表项索引顺序推进，识别与回读混在一起 → 第 48 台 READY
的最坏门是 13.0 s。**把「识别优先于回读」做成每总线两级**（身份先读完，再统一回读）
可把最坏降到约 4.34 s——理由：识别是绑定的前提，回读可以慢，但没有 profile 的台什么也做不了。

### 3.4 事件环 —— 冷启动确实不够 [源码]

`MOTOR_CONTROL_MAX_EVENTS = 128`，而冷启动每台 4 条（`DISCOVERED` / `PROFILE_BOUND` /
`READY` / `DEV_INIT`）→ 48 台 ≈ **192 条 > 128**。§6.10 把该常量 64→128 时只算了
`DISCOVERED` 的 48 条突发，漏了同样每台一条的其余三类。

更糟的是**断链期不排空**：事件环的唯一消费者是 ROS 桥 `publish_events`，而它只在
`ros_bridge_spin()` 里跑；agent 未连时 `spin` 不跑，环只进不出 → 溢出丢最旧、只留计数。

**修法**（两条都要）：环 128 → 256（AXI 不稀缺，事件 `sizeof ≈ 24 B`，代价约 +3 KB `.bss`），
并把注释钉死「事件环是有损的，真值在拓扑/状态电平话题」；断链期间抑制入环或降级合并。

**另一条 48 台专属风险（此前被误记为"尽力转发"）**：事件**话题**的 QoS 是
`qos_reliable(8)`（[motor_bridge.cpp:521](../../../src/motor_bridge.cpp)），而 rmw 对 reliable
发布走 `uxr_run_session_until_confirm_delivery(…, session_timeout)`——**逐条同步等确认**。
`publish_events()` 是「drain 16 条 → 逐条 publish → 再 drain」直到排空
（[motor_bridge.cpp:390-408](../../../src/motor_bridge.cpp)），所以 48 台冷启动约 192 条事件
= **约 192 次串行等确认**，全部发生在 10 ms 桥定时器里。这条通道**既不是有损的、也不是
异步的**：真正的有损在内核那 128 深的事件环，而送到 ROS 的这一段是阻塞式的可靠流。
48 台必须量这一段在桥定时器里占多少毫秒（否则 `motor_bridge_poll` 与 `param_bridge_poll`
会被它拖住），并考虑：事件按批合并成一条、或给关键回告（`CONFIG_RESULT`）单列一条可靠通道。

### 3.5 其余（均非约束）

| 项 | 值 | 判定 |
|---|---|---|
| 注册表 `MOTOR_CONTROL_MAX_MOTORS` | 64（`sizeof(entry) ≈ 460 B` → 29.4 KB `.bss`，落 AXI） | ok，48 台用 75% |
| 命令快照 / P1 槽 | 64（各与注册表 `static_assert` 锁齐） | ok |
| `MOTOR_CONTROL_MAX_TX_PENDING` | 160（一批 P1 48 + 30 ms 软停 P0 48 同拍 = 96） | ok |
| W25 blob v3 | 64 记录 ≥ 48；记录 120 B，越槽阈值 ≥128 B；每槽 2 扇区 7712 B ≤ 8192 B | ok，48 台不需再升版 |
| USB CDC 字节带宽 | 48 台稳态上行 180.2 KB/s、下行 80.9 KB/s、双向 261 KB/s ≈ FS 批量上限的 21.5% | ok，但**不是**真正的上限（§3.0 才是） |
| USB CDC 包预算 | framing 写缓冲仅 42 B，一条 `motion_states` 被切成 27 个 FS 包 → 稳态 6,479 包/s，占 19 包/帧预算的 34% | tight；CDC TX 队列仅 1,024 B，队列满时 `write()` 在 micro_ros 任务内自旋 |
| ROS 桥静态数组 | 按 64 元素预分配共 12,416 B + 消息结构体 324 B ≈ 12.9 KB `.bss`（RAM_D1） | ok，与 48 台无关 |
| ROS 2 侧写死假设 | `motor_hardware_interface` / `helpers.py` / `web_host` 的关节与总线全由 YAML 驱动，web 侧是 id→Map | ok，无硬编码台数 |
| **config FIFO** | `CONTROL_MAILBOX_CONFIG_QUEUE_CAPACITY = 16` | **需扩到 64**——块写按 48 台目标下发时 16 深装不下（§5.3 块话题） |
| `config_step` WCET | `bus_stage_in_flight` 是 O(n²)：n=48 → ≈2304 次比较/拍（n=4 时 16 次，放大 144×） | **未验**，落 TBD-RT-01 |
| **出站消息丢弃可见性** | `(void)rmw_publish_rc` 遍布各桥；诊断无出站丢弃计数 | **必须补**（§3.0） |

### 3.6 设备侧没有任何触发式上报 [源码]

CERB 的 CANopen 字典表共 **34 个对象**，其中**没有 PDO 映射（0x1600/0x1A00）、没有 EMCY（0x1014）、
没有心跳（0x1017）、没有 SYNC 的 COB-ID（0x1005）**——即驱动器**不支持任何"变化才报"的机制**，
也没有紧急报文。它的上报只有两种：

| 通道 | 触发方式 | 内容 |
|---|---|---|
| 20 ms 周期状态帧（`0x300+node`） | **由固件每 20 ms 每总线发的 SYNC（`0x080`）触发**——同步节拍，不是事件 | 位置 / 电机端速度 / 电流 Iq / 错误码 / **线圈温度 `data[8..9]`（大端 int16，0.1 °C）** / 控制模式反馈 / 状态位（含 **故障位 `data[11] bit5`**） |
| 200 Hz 命令回显 | 跟随控制节拍 | 命令实时值 |

其余全靠 **SDO 主动读**（含 `0x603F` 错误码、`0x6079` 母线电压、
`0x6077` 力矩、`0x6078` 实际电流，以及 `0x2662/0x2663` 温度相关对象——注意线圈温度
已在周期帧里随帧上报，无需为它轮询）。

**推论（直接影响参数所有权方案）**：

- 「设备参数被改动后上位机自动知道」**做不到**——设备不发变更通知。只能靠低频轮询、
  或在连接/上电时读一次、或把 `0x2539` 保存动作当作边界。
- 故障是**搭在 20 ms 周期帧上顺带上报**的，所以固件的故障发现延迟天然 ≤ 一个上报周期 ×
  OFFLINE 判定窗口（100 ms）；这条链路不依赖轮询，是这套设计里**唯一**真正"推"的东西。
- 因此「上位机需要时再读设备」只适用于**按需的、低频的**读；任何"实时看某台设备的参数"
  都只能走固件已有的状态话题。

---

## 4. 设计

四阶段。每阶段给可判定的验收判据；标注 **本板可验** 还是 **必须 3×16 台架**。

### P0 —— 修复正在生效的缺陷（先做，无依赖）

| 动作 | 文件 | 判据 |
|---|---|---|
| `PARAM_BRIDGE_MAX_PARAMS` 按 §5 裁定取值 + 整段重写矛盾注释 | `include/param_bridge.h:39-56` | 头部注释里只剩一个数，且与 `#define` 一致 |
| 加建服前堆预算拦截（§1.1 代码） | `src/param_bridge.cpp:825` 起 | 故意设 `MAX_PARAMS = 999` → 建服干净失败并报错，**不 HardFault** |
| 注册字段改显式集合，PID 六项退出参数树 | `src/param_bridge.cpp:579-591`、`lib/param_store/include/param_field.h:18-47` | `_Static_assert(PARAM_MOTOR_FIELD_COUNT == 5)`；host 用例断言「注册项数 = 表长 × 台数」 |
| 文档漂移对齐到契约 | 见 §1.3 表 | 全仓库 `grep PARAM_BRIDGE_MAX_PARAMS` 只剩定义处 + 派生处 |

> P0 与 48 台无关——**今天这块板子就有问题**。建议独立成一个 `fix(fw)` 提交先合。

### P1 —— 容量契约单一来源（把 48 焊成可校验的契约）

新增一个**只含编译期常量、无 include** 的契约头。**放 `lib/motor_capacity/include/motor_capacity.h`，
不要放工程级 `include/`**——理由：本仓库跨库共享编译期常量的既有先例正是
[`lib/board/include/board_irq.h`](../../../lib/board/include/board_irq.h)（`lib/fdcan/src/fdcan.c:10`
与 `lib/dma/src/dma.c:12` 都 include 它），而目前**没有任何 `lib/*` 编译单元引用工程级
`include/` 头**——用已验证的机制，不赌平台隐式路径。

```c
#define MOTOR_CAP_ARCH_MOTORS      64U   // 架构容量（ARD §8.1 MUST，刻意保留）
#define MOTOR_CAP_DEPLOY_CEILING   48U   // 设计上限（决策日志 §6.10）
#define MOTOR_CAP_DEPLOY_TARGET    40U   // 实际部署，仅文档与台架目标
#define MOTOR_CAP_BUS_COUNT         3U
#define MOTOR_CAP_MOTORS_PER_BUS   16U   // = ceil(48/3)，显式假设，不作运行时强制
#define MOTOR_CAP_P0_CAPACITY      32U   // = 2 × 每总线台数
#define MOTOR_CAP_EVENTS_MIN      256U   // = 2×48 + 32（§3.4）
#define MOTOR_CAP_DTCM_POOL    122880U   // 实测
#define MOTOR_CAP_SLOT_BYTES      328U   // 实测边际，仅在 176..256 槽区间有效
```

派生：`MOTOR_CONTROL_MAX_*`、`CAN_TX_BUS_COUNT`、`CAN_TX_P0_CAPACITY` 全部改为引用契约；
`motor_rt_task.cpp:21-34` 既有的跨库 `static_assert` 块追加契约自洽断言。

**预算断言只钉住实测有效区间**（328 B/槽是 176..256 槽的边际斜率，不可外推到 416）：

```c
static_assert(MOTOR_CAP_DEPLOY_CEILING * 5U + 10U <= 256U,
              "参数槽预算超出 328 B/槽 的实测有效区间，须先补测");
```

判据：`pio run` 通过；host 用例断言派生值与契约一致。**本板可验**（除 48 台的真实占用）。

### P2 —— 48 台专属失效点的机制解法

**⓪ micro-ROS 单条消息上限（最硬，先做）**

| 动作 | 文件 | 说明 |
|---|---|---|
| `UXR_CONFIG_CUSTOM_TRANSPORT_MTU` 512 → 2048 | `microros_user.meta` 构建宏 + 重生成 libmicroros | 用既有 `tools/refresh_microros_messages.py` 流程；**勿跑 `-t clean_microros`**。同时改 `micro_ros_agent` 侧并复核会话协商 |
| 出站丢弃计数进诊断 | `src/motor_bridge.cpp`（各处 `(void)rmw_publish_rc`） | 新增每话题 `publish_fail` 计数 → `MotorDiagnostics`（与 ① 的 `rx_hw_lost` 同一次消息刷新） |
| 复核 RAM_D1 增量 | `pio run` 输出 | best-effort 缓冲 + reliable 流缓冲随 MTU 线性增长，落 RAM_D1（不占 DTCM 堆） |

判据：48 台时 `motion_states` / `motor_status_array` 持续发布且 `publish_fail == 0`；
主机侧 `ros2 topic hz` 与 `ros2 topic echo` 可见全部 48 台。

**① RX 丢帧（唯一会在协议层真丢数据的地方）**

| 动作 | 文件 | 说明 |
|---|---|---|
| FDCAN 接收中断优先级 5 → **4** | `lib/fdcan/src/fdcan.c:144-146` | 抬到 FreeRTOS 屏蔽阈值之上；**前提已满足**：回调体内无任何 FreeRTOS API。须在 `board_irq.h` 写明这是**刻意的例外**，并加一条评审/编译期约束防回归 |
| `MESSAGE_LOST` 计入诊断 | `lib/fdcan/src/can_bsp.c:199`、`can_bsp.h` | 新增每总线 `rx_hw_lost` 计数 → `MotorDiagnostics`（改消息须跑 `refresh_microros_messages.py` 增量刷新） |
| SYNC 分相 | `src/motor_rt_task.cpp:285-294` | 三路同拍改 bus0/bus1/bus2 相位错开（0/7/14 ms），把三条总线的突发错开 |
| （可选）FIFO0 加深 8 → 16 | `lib/fdcan/src/fdcan.c:68,86-95` | 需先按 RM0468 复核 H723 FDCAN 消息 RAM 总额；**未核实，非必需**——优先级那一条已能消掉临界区屏蔽这个主因 |

判据（**必须 3×16 台架**）：16 台/总线连续运行，`rx_hw_lost == 0` 且 `ring_drop == 0`；
同时记录 `MESSAGE_LOST` 出现时的 ISR 延迟分布。

> **不做**：`AutoRetransmission` 改 DISABLE。它能缓解「失联节点占住 8 深 TX FIFO」，
> 但 [fdcan.c:36-47](../../../lib/fdcan/src/fdcan.c) 明写「数值是台架实测基线，无示波器/总线分析仪
> 不得改动」——改变 TX 时序语义必须配一轮位时序与错误计数台架基线，不在本设计的射程内。

**② node 全局唯一命名空间**

`motor_id = 厂家 node 号`，注册表以它为全局键；同号出现在另一条总线 → `latch_config_conflict`
→ 该台禁运动、**锁存且无运行期解除路径（只能整机重启）**
（[motor_control_discovery.c:100-118](../../../lib/motor_control/src/motor_control_discovery.c)）。
**朴素的「三条总线各 1..16」接线会让 32 台直接锁死**，而固件里没有任何 node 分配表或校验。

| 动作 | 说明 |
|---|---|
| 写死分配纪律 | `bus0 = 1..16 / bus1 = 17..32 / bus2 = 33..48`（vaxor 与 CERB 共用同一全局空间，上界 60 → 48 台余 12），落决策日志与接线文档 |
| 冲突诊断带上两边总线号 | 现有 `CONFIG_CONFLICT` 事件只带触发侧 `bus_index`；补上原表项总线，让现场一眼看出是哪两条线撞了 |
| 显式解除路径 | 至少给出「修线 + 重扫」的可执行操作；锁存语义本身（安全侧不自动解除）**保留** |

**待厂家确认**（阻塞部署）：CERB 的 node 号怎么改（字典表里没有节点配置对象，
[cerb-vendor-questions.md](cerb-vendor-questions.md) 也没列这条）；vaxor 的 node 是否可配。
**若设备侧 node 不可改**，则「全局唯一」这条路走不通，必须改为复合键 `(bus, node)`——
那会改动 ROS 侧 `motor_id` 契约与全部冲突判定，是另一个量级的决定。

**③ 发现就绪时间与事件环**

| 动作 | 文件 | 判据 |
|---|---|---|
| 每总线两级推进：识别优先于回读 | `lib/motor_control/src/motor_control_config.c:461-503` | 最坏就绪门 13.0 s → ≈4.34 s [推导]；台架实测第 48 台 READY 时刻 |
| 事件环 128 → 256 + 注释钉死「有损通知」 | `lib/motor_control/include/motor_control.h:37-41` | host 用例：192 条冷启动事件不丢；断链期入环被抑制 |
| `config_step` 的 O(n²) WCET → 按总线维护 O(1) 的 in-flight 标志 | `motor_control_config.c:358-371,455-466` | 每拍比较数从 n² 降到 n；台架量 48 台冷启动期的 1 ms 周期 jitter |
| **台架模拟器升级**（否则 §6 的 P2 判据全部悬空） | `micro_ros_agent/tools/fdcan_motor_sim.py` | 支持三总线、错峰 ≤0.4 ms（现 2.2 ms×48 = 105.6 ms ≫ 20 ms SYNC 周期）；或把相关判据显式标为 blocked-by-hardware |

### P3 —— 参数面收敛与文档对齐

**单位统一为 ROS 标准（REP-103），取消参数面的工程单位换算**（2026-09-20 决定）：
话题面本来就已经是 rad / rad·s⁻¹ / N·m / A / V / °C，唯一偏差在 `param_bridge.cpp`——
它用 `lib/param_store/include/param_units.h` 把参数值换成 deg / deg·s⁻¹ / A 给操作员看
（`param_bridge.cpp:219,221,348,349,515,517,519,521` 共 8 处）。
**决定**：传输边界一律 SI，人类单位降级为**展示层**的事（页面自己换算）。
后果：删掉那 8 处调用与 4 个换算函数（连带其 host 用例）；`torque_limit` 由
"峰值电流(A)"改为 **N·m**——内核本来就在 N·m 域钳制
（`motor_control_config.c:878-893`），边界退化成纯透传，那对 N·m→A→mA 的双重换算消失。
W25 记录与内核域不变，**无需迁移**；参数树只是状态的投影，不存值。
这条**推翻决策日志 §6.4**（"参数对外一律工程单位"），落地时需新增一条决策记录。
附带收益：以后新增任何 `.msg` 都不必再决定单位——照 REP-103 写即可。

按 §5 裁定执行（推荐路径：电机参数撤出、`MAX_PARAMS` 收到 16 或 32、块话题承载配置）。
块话题的具体施工设计（消息字段、复用既有 config 事务机、块 1 十六阶段 / 块 2 十三阶段、
保护开关只接受 0、上电自动写、`motor_status_array` 新增保护/方向状态、config FIFO 16 → 64）
已具备可施工粒度，但**它依赖 owner 对 §5 裁定的确认**，本文不重复展开。

---

## 5. 参数服务器必要性评估

### 5.1 它现在给了什么 [源码]

| 能力 | 现状 | 唯一性 |
|---|---|---|
| 逐电机静态量（限位×2 / 限速 / 限流 / 保护开关） | `m<id>.*`，实际还多注册了 6 个 NaN 的 PID 项 | 唯一入口，但**不可行**（§3.1） |
| 系统参数 10 项 | `system.{save, refresh, refresh_seq, set_zero, set_zero_seq, clear_fault, clear_fault_seq, last_error, last_error_motor, heap_free}` | 唯一入口 |
| 上位机交互 | web_host `app.js` 经 rosapi 的 `get_parameters` / `set_parameters` / `list_parameters` 直连固件；9 个 `system.*` 名字 + 每台 5 个字段名 | 拆掉即整页报废 |
| 命令行 | `ros2 param get/set/list`（`low_mem_mode` 兼容；批量 get/dump 不兼容，请求容量恒为 1） | 独有 |
| 四个脉冲命令 + 四个完成序号 | 「写 = 电机 id，轮询只读序号判完成」的既有协议 | 已台架验证（2026-09-17 真机结案） |

### 5.2 它要多少 [实测]

```
不可行的那一半：48 台 × 11 项 + 10 = 538 槽 × 328 B = 176,464 B（池的 1.44 倍）
按设计意图的 5 项：48 × 5 + 10 = 250 槽 × 328 B = 82,000 B（池的 67%，永久占用）
只留 system.* 10 项：32 槽 × 328 B = 10,496 B = 10.25 KiB（池的 8.5%）
```

**三个结构性事实**：

1. 成本与**板上实际几台电机无关**——`init` 时按 `max_params` 一次性全量分配
   （`parameter_server.c:543/548/555/595`），一台电机都没有也照样吃满。
2. 释放的同理：撤电机参数把 416 → 32，`heap_free` 从负值回到约 **92.7 KiB**。
3. 每槽 328 B 里只有约 269 B 是结构（`sizeof(Parameter)=112` + descriptor + 50 B 名…），
   余下约 59 B 是分配器块开销——**每槽约 6 次小块分配**，长跑碎片演化未测。

### 5.3 裁定

**结论：对 48 台设计而言，参数服务器不必要。**

> ### ✅ 结案（2026-09-21，owner 裁定，已执行）
>
> 本文推荐的**两步走**没有按"第一步先缩到 32 槽"执行——owner 直接把**第二步**
> （整体退役）做完了，理由是本文自己给的「顺路」判据：当时正准备为块话题/命令话题
> 刷新 libmicroros，多一条消息类型的边际成本接近零。所以第一步（`MAX_PARAMS 416 → 32`）
> **从未落地**，直接跳到话题面。
>
> **落地形态**：下行 `micro_ros_do/param_set`（`ParamSetArray`）+ `system_command`
> （`SystemCommand`），上行 `micro_ros_up/system_status`（`SystemStatus`）；
> 四个脉冲命令与四个完成序号逐字保留语义，只是换了承载。读侧不动
> （`device_param_report` 本就双行覆盖六槽）。详见决策日志 §6.9 结案块与变更记录。
>
> **⚠️ 本节末尾"第二步的额外收益"说法要下调。** 原文写"退役会释放
> `MAX_SERVICES` 的 5 个槽、`MAX_PUBLISHERS` 的 1 个槽（`/parameter_events`）与
> 1 个 executor handle"，读起来像三笔净赚。实测：
>
> | 项 | 原说法 | 实际 |
> |---|---|---|
> | `MAX_SERVICES` | 释放 5 槽 | ✅ 确实成余量（现用 0/5）。但这是**有意不降**的静态数组，不占 DTCM 堆，收益仅为"以后加服务不用重建库" |
> | `MAX_PUBLISHERS` | 释放 1 槽 | ❌ **被新话题吃回去了**：`system_status` 占 1 槽，另 1 槽留给块话题 → 10 → **12** |
> | `MAX_SUBSCRIPTIONS` | （原文未提） | ❌ 新增 2 条订阅（`param_set` / `system_command`）→ 5 → **6** |
> | executor handle | 释放 1 个 | ⚠️ 但参数桥自身从 5 个 handle 变 2 个订阅，净额 9 → **6**（不是 8） |
>
> **真正的收益只有一笔，且是运行期量**：**~23.6 KB DTCM 堆**（`heap_free` 上升）。
> 连带减少的 `.bss` −11.4 KB / Flash −37.2 KB 是 libmicroros 与代码的产物，与
> "参数面"无关，不该记在它头上。原文把静态数组槽位与串口堆混在一句里当收益讲，
> 是本计划唯一一处需要回调的结论。

三条依据，按强度排序：

1. **它从未被论证过。** 文档考古的结论是：全仓库**没有一句「为什么用参数服务器而不是话题」的
   正面论证**。它是 2026-09-16/17 先落地的事实，之后所有动作只是顺着既有通道加。
   唯一相关的陈述是一条**事后的反面约束**——「不用话题/服务，因为逐电机动作的既验证通道
   本就是参数，且 `MAX_SERVICES=5` 已被 rclc 参数服务器占满，加服务要重建 libmicrosos」。
   这是沉没成本 + 约束，不是设计理由。
2. **它的主用途（逐电机配置）在 48 台不可行**，而这与它自身的形态有关，不是调参能救的：
   唯一稀缺资源是 120 KiB 的堆池，而它要拿 2/3 去常驻一批**已经存在 W25 里、且上位机
   本来就按表格读写的值**。
3. **剩下的 `system.*` 是命令，不是参数。** `save / refresh / set_zero / clear_fault` +
   四个 `*_seq` 完成序号是「写 = 电机 id、轮询序号判完成」的命令/应答协议，只是穿着参数的
   外衣。交接文档 §5.1 记录的 `outstanding` 永久残留缺陷（事务在飞时掉电 → 该字段影子同步
   被无限跳过、参数面卡在旧值）正是这种语义扭曲的症状，不是实现疏忽。

**但保留 `system.*` 是划算的惰性选择**，所以推荐**两步走**：

**第一步（随 48 台设计一起做）**：`MAX_PARAMS 416 → 32`，电机参数撤出参数树，
`system.*` 原样保留。代价 10.25 KiB（8.5% 堆）+ 5 个 executor handle；收益是 web_host 的
参数页与 `ros2 param` 工具链**零改动**，四个脉冲命令的已验证链路不动。并在决策日志里
写死一句：**参数树不是配置通道**（配置一律走块话题）。

**第二步（可选，建议在块话题那一轮一并做完，不要另起一轮）**：若要做，就在**同一次**
`refresh_microros_messages.py` 增量刷新里加一条系统命令消息（`save/refresh/set_zero/clear_fault`
+ 复用既有 `request_id` 关联 `CONFIG_RESULT` 事件回告），然后整体退役参数服务器，
回收最后 10.25 KiB 与 5 个 handle。**判据是「顺路」**：既然为块话题已经必须刷新 libmicroros，
再多一个消息类型的边际成本接近零；反之若单独为它开一轮刷新型，就不值。

第二步还有一笔 §3.0 带出来的额外收益：**退役参数服务器会释放
`RMW_UXRCE_MAX_SERVICES` 的 5 个槽（现被它独吃）、`RMW_UXRCE_MAX_PUBLISHERS` 的 1 个槽
（`/parameter_events`，现 9/10 已用）与 1 个 executor handle**。在 48 台要加诊断话题、
按总线拆话题或加自定义服务时，这几个槽位是实打实的余量——保留派把 `MAX_SERVICES=5`
当作"不能动它"的理由，方向恰好相反：那 5 个槽正是它占着的。
*（**上面这段的账目不完整，2026-09-21 实测修正，见本节结案块**：退役确实腾出了
service 槽，但新增的两条下行话题与一条上行话题把 publisher/subscription 槽吃了回去，
净收益 ≈ 0；真正的收益是 ~23.6 KB DTCM 堆。）*

**若 owner 选择不撤电机参数**（即保留 250 槽、沿用参数树做配置）：本文认为不可取，理由是
余量只有 **21,220 B**，而堆的最坏瞬态（并发 SDO 事务 + rosidl 字符串分配）**至今未测**——
在一个物理不可扩的池子上用 67% 常驻一个「与在线上电机数无关」的表，是拿未见过的风险去换
一次 web 页零改动。此路若走，必须先把 48 台规模的 `heap_free` 稳态最低位与最坏瞬态量出来。

---

## 6. 验收判据

**本板（4 台）可验**：P0 全部；P1 的契约自洽与 `pio run`；参数面槽数/注册项数的 host 用例；
事件环 256 的 host 用例；`MESSAGE_LOST` 计数被诊断消费。

**P0 的真实判据是实测内存，不是编译通过**：agent 连上后取一次
`ros2 param get /micro_ros_node system.heap_free`，其值应与 32 槽预算一致（预期约 92.7 KiB），
并记录长跑稳态最低位——这是"328 B/槽"模型是否成立的**唯一实测证据**，也顺带证明 416 的修复生效。

> 注：`system.heap_free` 的读取依赖参数服务器，所以它必须在 §5 裁定的"第一步"之后仍然可用。

**端到端 48 台验收当前被三件事挡住**，须在计划里显式记账（不要默认它们已存在）：

1. **软件生产链已同步，但端到端契约仍未验证**：截至 2026-09-22，
   `motor_hardware_interface` 与 `web_host` 已接入 `MotorTrajectory` 的
   `segment_index/segment_count` 分段契约；同一轨迹最多 256 点，分段保持绝对
   时间线。仍未在 rosbridge、micro-ROS Agent、MCU 和真机之间验证 48 关节的
   `joint_name ↔ motor_id`、方向、零位和有序到达行为，因此 §3.0 的端到端判据仍
   未完成。源代码同步不等于硬件验收。
2. **台架只有 4 台 CERB**（本板自述），且唯一的模拟器在 48 台下模型失效（见上）。
3. `MESSAGE_LOST` / 就绪时间 / SYNC 错相这三条判据都需要 3×16 同时在线。

**必须 3×16 台架**（当前证据为零）：

1. `motion_states` / `motor_status_array` 在 48 台时**真的发得出来**（§3.0 的判据），
   `publish_fail == 0`。
2. `rx_hw_lost == 0` 且 `ring_drop == 0`，16 台/总线连续运行；记录 ISR 延迟分布。
3. 48 台冷启动第 48 台 `READY_DISABLED` 的时刻（预期 ≈0.2~0.5 s 典型 / ≈4.3 s 最坏）[未验]。
4. 48 台规模的 `heap_free` 稳态最低位与最坏瞬态低点（§5.2 之外，碎片演化只有台架能给）。
5. `config_step` 的 O(n²) WCET 与 1 ms 周期 jitter（TBD-RT-01）。
6. 三总线在 48 台满负载 + 并发 SDO 下的仲裁延迟与负载（SDO 低仲裁优先级会被 MIT/反馈挤）。
7. 跨总线同号 node 的冲突告警与解除路径（可用 4 台人工制造同号，**本板即可做一半**）。

> **台架工具本身的坑**：`micro_ros_agent/tools/fdcan_motor_sim.py` 能枚举 48 个 ID，
> 但它按台错峰 2.2 ms，48 台 = 105.6 ms 的错峰窗口**远大于 20 ms 的 SYNC 周期**
> （`fdcan_motor_sim.py:72-76,248,393-395`）——deadline 会按每台约 5 条堆积、退化成连发。
> **用它压 48 台得到的 CAN 负载与相位结论不可信**；要么修模拟器的错峰模型，
> 要么 48 台判据只能在真设备上取。这条应写进 validation plan。

---

## 7. 与既有决定的关系

| 既有决定 | 本文关系 |
|---|---|
| 决策日志 §6.10：内核与消息容量保持 64，只校准参数面 + 三常量 | **一致**，且 P1 把它们改为派生而非再改数字 |
| 决策日志 §6.10：`MAX_EVENTS` 64 → 128（理由：48 条 DISCOVERED 突发） | **修正**：漏算 `PROFILE_BOUND`/`READY`/`DEV_INIT`，冷启动实际 ≈192 条 → 256 |
| 交接文档：电机参数撤出参数树、配置走块话题、`MAX_PARAMS → 32` | **承接**。本文补上容量数字修正（256 应为 32）与 48 台运行时失效点 |
| 交接文档缺省：决定未写进 ADR/决策日志 | **本文即为写入前的依据**；P3 落地时同步 |
| ARD-0007 §8.1：注册表容量 MUST ≥ 64 | **一致**，不动 |
| ARD-0007「约 40 台，架构容量至少 64」 | **一致**；本文补上「设计上限 48」这一层 |

---

## 附录 A：本板测不出的量（必须台架）

| 量 | 为什么本板测不出 |
|---|---|
| 48 台 `heap_free` 稳态最低位 | 参数面分配与在线上电机数无关，4 台测不出 48 台的注册/同步压力与碎片 |
| 16 台/总线共 SYNC 的上报时间散布 | 需要同总线 16 台真实设备 |
| 48 台冷启动事件峰值 | 需要 48 台同时上电 |
| SDO 真实 RTT | 决定识别/回读每阶段实际耗时，进而决定就绪秒数 |
| `config_step` O(n²) 的 WCET | 需要 n=48 的注册表 |

## 附录 B：关键证据索引

| 事实 | 出处 |
|---|---|
| rclc 按 `max_params` 一次性全量分配、不查 NULL | `.pio/…/rclc_parameter/src/rclc_parameter/parameter_server.c:543,548,555,595` |
| 每槽 328 B（两对点同斜率） | `motor-runtime-validation.md:32-33`；`param_bridge.h:46-50` |
| 堆池 120 KiB、不可扩 | `param_bridge.h:47-48`；`ldscript/stm32h723vgt6.ld:178-213` |
| 注册循环以 `PARAM_FIELD_COUNT` 为界 | `src/param_bridge.cpp:579`；`lib/param_store/include/param_field.h:18-47` |
| RX FIFO0 = 8 深、FIFO1 未用 | `lib/fdcan/src/fdcan.c:68,70` |
| FDCAN ISR 优先级 = FreeRTOS 屏蔽阈值 | `lib/fdcan/src/fdcan.c:144-146`；`lib/board/include/board_irq.h:9`；`FreeRTOSConfig_Default.h:207` |
| 回调体不用 FreeRTOS API | `lib/fdcan/src/can_bsp.c:190-217` |
| 每总线一问一答 | `lib/motor_control/src/motor_control_config.c:358-371` |
| `motor_id = node` 全局键 + 冲突锁存无解除 | `lib/motor_control/src/motor_control_discovery.c:100-118`；`motor_control_internal.h:17` |
| 「16 台/总线」= 48÷3 的算术 | `lib/fdcan/include/can_tx_scheduler.h:31-33`；`motor-runtime-decision-log.md:413` |
| best-effort 传输上限 512 B、单一全局缓冲 | `libmicroros/include/uxr/client/config.h:60`；`rmw-microxrcedds_c/src/rmw_microros_internal/types.h:91`；`rmw_init.c:316` |
| fragmented 回退对 best-effort 无效 | `Micro-XRCE-DDS-Client/src/c/core/session/write_access.c:222-226`；`rmw_publish.c:57-70` |
| publisher / service 槽位余量 | `microros_user.meta:6,8`（`MAX_PUBLISHERS=10` 已用 9、`MAX_SERVICES=5` 已满） |
| 模拟器错峰窗口 > SYNC 周期 | `micro_ros_agent/tools/fdcan_motor_sim.py:72-76,248,393-395` |
| 三层口径 64 / 48 / 40 | `docs/archive/ARD-0007-micro-ros-motor-runtime.md:9`；`motor-runtime-decision-log.md:402` |
| 无「为何用参数服务器」的正面论证 | `motor-runtime-decision-log.md:381-384`（唯一相关陈述是反面约束） |
