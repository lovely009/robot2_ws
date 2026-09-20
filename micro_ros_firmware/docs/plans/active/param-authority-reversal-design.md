# 参数权威方向反转设计：固件记录为权威（2026-09-21）

**状态：Accepted（2026-09-21 实现）。** 反转 2026-09-17 决策 §6.3「设备参数以驱动板为准」。
实现计划：`param-authority-reversal-plan.md`。

**尚未上板验证。** 本文所述新语义（`system.save` → `0x2539` 闭环、`NO_RECORD` 首启行为、
`device_param_report` 低频话题、「写后不保存即掉电」判别）目前只经 host 测试与静态审查，
真机台架项见 `motor-runtime-validation.md`（TBD-CONFIG-01 追加项，**未运行**）。

## 1. 动机

现状把**驱动板**当权威：上电回读设备、与 W25 记录比对、不一致即判未初始化，
恢复口径是「以设备为准重新基线」（2026-09-19 真机台架按此结案）。

owner 2026-09-21 明确反转：**固件内部维护的那一份参数表是权威**，驱动板内的参数
要与固件对齐——初始化语义的来源是固件，不是设备。

这不只是文案问题：它改变了"不一致时该往哪个方向写"，而 2026-09-19 那条结案按新
口径是**错结论**。

## 2. 目标语义（owner 原话流程）

1. 固件内部维护一份参数表（权威）。
2. 上电读驱动器参数。
3. 一致 → 初始化通过。
4. 不一致 → 未初始化，**电机不可使能**。
5. web 界面**同时显示两行**——固件值 / 驱动器值——供人工核对。
6. 核对后「下发修改」→ 两侧一致 → 校验通过、退出未初始化。
7. 要持久化得再点「**保存**」，否则下次上电照样不通过。
8. **首次上电（W25 无记录）**：固件侧显示「无记录」，同样走人工核对。
9. 只点了写、没点保存 → 必须有「**未保存**」标记。

## 3. 设计

### 3.1 判据状态机：三态拆四态

现在 `DEV_INIT_NA` 同时承担两个含义——"本协议无设备侧对象"与"尚未定论"，
且**是放行的**（[motor_control_config.c:179](../../../lib/motor_control/src/motor_control_config.c#L179)
`ready = (state == NA || state == OK)`）。必须拆开：

| 态 | 含义 | 放行 |
|---|---|---|
| `OK` | 有记录且逐槽一致 | ✅ |
| `MISMATCH` | 有记录、某些槽不一致 | ❌ 未初始化 |
| **`NO_RECORD`（新增）** | 设备侧型号但 W25 无该台记录 | ❌ **未初始化**（原来落 NA 放行） |
| `UNVERIFIED` | 回读没成功 | ❌ 未初始化 |
| `NA` | **仅**本协议无设备侧对象（VAXOR） | ✅ |

`ready` 的判据不变，变的是"无记录"不再落进 `NA`。

**受核对的对象**是四个设备管理对象：`0x607D:01`/`:02`（位置上下限）、`0x6080`
（最大转速）、`0x2538`（峰值电流）。
`0x2650:00`（保护开关）与 `0x2541:00`（方向）**没有厂家保存码**：前者上电归一恒写
`0`、后者按记录写回，都是"固件写下去"而非"用户设限值"，所以 §3.2 里它们的写只落
W25、不发 `0x2539`。但它们**仍在 `mismatch_mask` 里各占一位**（bit4/bit5，见
[`motor_control_dev_mismatch_mask()`](../../../lib/motor_control/src/motor_control_config.c#L107)）：
方向尤其没有设备侧基线，记录缺这一槽即判未初始化。**"不参与保存" ≠ "不参与比对"
≠ "不呈现"**——§3.3 报告的六对槽位全部成对呈现、供人核对。

### 3.2 写 / 保存分离

- **写**（参数面 `m<id>.{pos_min,pos_max,vel_limit,torque_limit}`）
  = 写驱动板 + 读回核验 + 更新固件内存记录 → **置 record dirty**。
  **不碰设备 flash、不碰 W25。**
- **保存**（`system.save`）= 对 dirty 的台：写设备 `0x2539`（驱动器落 flash）
  + W25 落盘 → 清 dirty。只处理 dirty 的台；没有 dirty 时空操作（不再像现在
  "写一次就落一次盘"）。

**写事务的读回核验通过后，该台结论立即重判**，不等下次上电——这正是 owner 第 6
条"下发修改后校验应通过、不再是未初始化"。重判用的仍是那套逐槽位比对，不是无脑
转 `OK`：记录写齐了就转 `OK`；还差槽位（典型是方向没有基线，见 2026-09-20 的
"方向无记录不猜"）就仍判 `MISMATCH`，位图点名缺哪一项——**不伪造通过**。

因此现状两处必须拆掉：

1. **写事务末尾那条 `0x2539` 挪到保存**。现在是「设备侧写事务以写 FLASH 收尾」——
   写一次就落盘一次。新模型下 `0x2539` 属于保存这一步。
   （`0x2650`/`0x2541` 本来就不跟 `0x2539`，不受影响。）
2. **storage 的"去抖 500 ms 自动落盘"删掉**。现在"设备侧写成功后记录同步落盘"，
   这正是 owner 说的"只点了写就该是未保存"要消掉的行为。

**机制落在逐表项的脏位图上**：`entry->record_dirty_slots`
（[motor_control_internal.h:141](../../../lib/motor_control/src/motor_control_internal.h#L141)），
写事务只置位、不清零。聚合 API（`motor_control_take_record_dirty()` 与 `ctx->record_dirty`）
**已在 469b16a 刻意删除**——生产侧删掉 500 ms 去抖后没有消费者，保留会让"需要保存"存在
两个可能漂移的真相源。消费方是 `system.save`：`param_bridge` 读到该位图后**分批**下发
`SAVE_DEVICE_FLASH` 事务（`save_slots` = 全量脏位图，见 d4abb37）+ 落 W25，并把 dirty
状态经 §3.3 的话题暴露出去。

**不保存就掉电**：记录只在内存里改过（W25 没落盘），设备侧也没写 `0x2539`
——**两边一起**回到上一轮已保存的值：

- 原来就是 `MISMATCH` / `NO_RECORD`（首次配置）→ 下次上电照旧不通过，与 owner
  第 7 条一致；
- 原来是 `OK`（改的是已保存过的值）→ 下次上电两边仍一致、仍是 `OK`，改动静默
  作废。这正是"不点保存就不持久化"的含义，不是异常。

**哪些槽位要 `0x2539`**：只有四个槽有厂家保存码——位置下限（码 4）、位置上限
（码 2）、速度与力矩限值（码 1）。保护开关（`0x2650:00`）与方向（`0x2541:00`）
是易失对象、**无保存码**：它们靠每次上电的归一从记录重建
（`boot-normalize-design.md`）。所以这两槽的写：

- **要**置未保存标记——它们改了记录内容，W25 待落盘；方向尤其靠记录才能在下次
  上电重建（不落盘 = 下次上电方向回到旧值）；
- `system.save` 对"只有这两槽脏"的台**不发任何 `0x2539`**，只清标记 + 落 W25。

**"有记录"的粒度**：本设计里的 `NO_RECORD` 专指**该台一条设备参数记录都没有**。
判据是独立的 `record_established`（W25 本来就有设备参数记录，或**人**写过；内部伪
request_id 的写不算）——**实现期勘误：不是** `record_raw_present == 0`，上电归一恒写
保护开关，那一笔会把该位图置起来、空板在回读前就"有记录"了（见
`param-authority-reversal-plan.md` 勘误表，提交 385f7ed）。部分槽位有记录时维持既有
规则——**没写过的槽位不参与比对**（刚烧录、Flash 半空的电机不该整体卡住），只有完整
无记录才判 `NO_RECORD`。两者不可混：前者是"还没记到这一项"，后者是"从没核对过这台"。

### 3.3 新只读话题 `/micro_ros_up/device_param_report`

**不走参数面**：`PARAM_BRIDGE_MAX_PARAMS = 72`（[param_bridge.h:39](../../../include/param_bridge.h#L39)
容量注释：10 system + 10 电机 × 6 项，`kMaxMotors = (72 − 10) / 6 = 10`）、每台
**6 项**，再加一列"设备侧影子"（7 项/台）会把支持台数从 10 压到 8。也**不塞进 `motor_status_array`**：那是每台 20 Hz 的流，
加 12 个 float 会让每条状态消息涨约 50 B/台（48 台约 2.4 KB/条），而参数值几乎不变；
"USB CDC 上行带宽余量"本就是验证计划的开放项。低频专用话题也更贴现有先例
（`MotorTrajectoryState` 同为"MCU → 上层、低频"）。

```
# DeviceParamReport.msg —— 参数核对用的双行快照（MCU → 上层，低频）
uint64 sync_time_ns
uint16 count
DeviceParamEntry[<=64] entries

# DeviceParamEntry.msg
uint16 motor_id
uint8 dev_init_state     # OK/MISMATCH/NO_RECORD/UNVERIFIED/NA，同内核枚举
                         # NO_RECORD = W25 无该台记录 → 固件侧整行显示「无记录」
bool record_dirty        # 已写、未保存
uint8 mismatch_mask      # 对不上的槽位位图（口径同 DEV_INIT 事件）
float32 rec_pos_min      # 以下六对：记录侧 / 设备侧回读（含保护开关与方向两槽）
float32 rec_pos_max
float32 rec_vel_limit
float32 rec_torque_limit
float32 rec_safety       # 保护开关（0x2650:00）
float32 rec_direction    # 方向（0x2541:00）
float32 dev_pos_min      # 设备侧无回读时填 NaN（沿用"缺失量一律 NaN"约定）
float32 dev_pos_max
float32 dev_vel_limit
float32 dev_torque_limit
float32 dev_safety
float32 dev_direction
```

发布时机：状态变化时即时发一帧 + 1 Hz 心跳。

单位与参数面保持一致（位置 rad、速度 rad/s、力矩 **A** ——设备对象 `0x2538` 存的就是
电流），换算在固件侧完成，页面不碰 cnt/mA。

### 3.4 给 P2（web 大修）的接口契约

- 双行对照表：固件列 / 驱动板列；`dev_init_state == NO_RECORD` 时固件列整行显示「无记录」。
- `record_dirty == true` 的台标黄 + 「保存」按钮（发 `system.save`）。
- 总览四计数：已核对 / 未初始化 / **未保存** / 离线。
- 写仍走现有参数面写事务（`m<id>.*`，那条链已实测可用——指既有的写事务链，**不是**本次新的保存路径），不动。

## 4. 文档纠偏（已随本设计同期完成，2026-09-21；下表保留为改动记录）

按新口径，下面四处此前是**错的**（留着下一个人就照旧方向干活）——**本次已改正**，
下面引用的是当时的反向原文，列出仅供对照：

1. `micro_ros_firmware/README.md:342` ——「设备参数以驱动板为准（2026-09-17）」
2. `micro_ros_firmware/docs/architecture/motor-runtime-decision-log.md` §6.3 + 决策表条目
   （含"不做开机以设备为准静默覆写记录"那句——方向已反转）
3. `ros2_ws/src/web_host/README.md:23` 与 `:38` ——「以驱动板为准」与「来源列」
4. `micro_ros_firmware/docs/plans/active/motor-runtime-validation.md` TBD-CONFIG-01 ——
   那条「**结案：以设备为准重新基线**」按新口径是错结论，改为"当时只解决了那一台
   设备，方向已由 owner 于 2026-09-21 反转"

## 5. 测试（host CTest，不启硬件）

- 无记录 + 设备侧型号 → `NO_RECORD` → 未初始化、拒使能
- 写 → 校验通过、dirty 置位；**断言 W25 与 `0x2539` 都没发生**
- 保存 → dirty 清除、`0x2539` 发出、W25 落盘
- VAXOR（无 `CAP_LIMIT_DEVICE_SIDE`）仍落 `NA` 且放行——守规矩，别把无此概念的型号一起拦了

> 「**写后不保存即复位 → 下次上电仍不通过**」**不是 host 用例**：host 没有 W25 与
> 掉电语义。它由「写不落盘」的 host 断言（W25 与 `0x2539` 都没发生）+ 台架项
> （`motor-runtime-validation.md` TBD-CONFIG-01 追加项，**未运行**）共同覆盖，不在
> host 里造假复位。

## 6. 不在本次范围

- web 界面的实现（布局重做、拆模块、真机安全面）——P2，另立 spec。
- 位置限值是否要有"固件侧出厂默认"——owner 已定：首次上电固件侧显示「无记录」，
  不引入 profile 默认值字段。
- `0x2539` 在真机上阻塞驱动器自身控制循环的影响（真机台架项，见厂商问题清单）。
