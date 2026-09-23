// motor_control 低频配置：Policy 本地项（增益/软件限位）与 request_id 事务
// （ARD-0007 §19/§20/§26）。
//
// 事务规则（ARD §20）：每电机至多一个未决事务，新请求立即 BUSY（不排队、
// 不覆盖）；分阶段、跨 1ms 服务节拍推进（每节拍至多一个阶段迁移），绝不
// 等待设备；可重试阶段自动重试 ≤2 次；结果码统一七种；读回不一致不得
// 返回 OK（禁止项 9），失败不自动恢复运动授权。
//
// SET_MODE 门禁：事务会驱动厂家使能序列（vaxor stop/start 控制帧），因此
// 仅对已完成身份确认（Profile 已绑定）、处于可配置生命周期且系统已授权的
// 电机受理；模式值域为 mailbox 契约常量（0=MIT 1=velocity 2=position），
// 具体模式是否被型号支持由 adapter 的阶段数回答（0 → UNSUPPORTED）。
//
// OK 语义（禁止项 9）：无响应写阶段无法确认设备状态的协议
// （adapter->config_verify_required）不得返回 OK，统一收敛为 UNCONFIRMED。

#include "motor_control_internal.h"

#include <math.h>
#include <string.h>

// mailbox 冻结契约的 SET_MODE 值域（0=MIT，1=velocity，2=position）。
#define MOTOR_CONTROL_CONFIG_MODE_MAX 2U

// 身份读取事务的伪 request_id。事务不产生 CONFIG_RESULT（不是上层请求），
// 该值只用于 tx_request_id 的确定性初值；取高位置位以避免与 mailbox
// request_id（低频计数器）碰撞。
#define MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE 0x80000000U

// 身份读取失败的重试预算（2026-09-20）：与设备参数回读同口径——读是幂等的只读
// 事务，多问几次没有副作用，而"用尽才判不可识别"能挡住一次总线抖动把某个关节
// 整场锁死。读**成功**（设备答了串）不在此列：目录里没有就是没有。
#define MOTOR_CONTROL_IDENTITY_READ_MAX_ATTEMPTS 3U
#define MOTOR_CONTROL_IDENTITY_READ_RETRY_MS 500U

// 事务发起（定义在提交入口段；config_step 的自动回读 / 身份读取都要用）。
static bool start_tx(motor_control_context_t *ctx, motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms);

// 逐电机失能（Policy enable=false 与 SET_ENABLE(false) 共用）：清授权与目标 →
// 在动则发失能帧 → 仍在 ACTIVE/SAFE_STOP 的退回 READY_DISABLED 并出事件。
// ARD §36 冻结语义：复运动必须显式重发 enable=true，本函数不留自动恢复路径。
static void apply_entry_disable(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  entry->policy_enable = false;
  entry->motion_authorized = false;
  entry->has_target = false;
  if (entry->motion_enabled) {
    (void)motor_control_send_p0(ctx, entry, MOTOR_CONTROL_P0_DISABLE);
    entry->motion_enabled = false;
  }
  entry->enable_pending = false;
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP) {
    entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DISABLE,
        entry->motor_id, entry->bus_index,
        MOTOR_CONTROL_FAULT_NONE, 0U);
  }
}

// ---------------------------------------------------------------------------
// 事务收尾与步进
// ---------------------------------------------------------------------------

// 身份读取收尾：拓扑上等价于配置事务，但语义是"识别"而非"配置"——
// 不回 CONFIG_RESULT（上层没提过请求），只在成功时按身份串绑定型号。
static void finish_identity_read(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint16_t result,
    uint32_t now_ms) {
  entry->tx_active = false;
  entry->tx_awaiting = false;

  const bool read_ok =
      result == MOTOR_PROTOCOL_CONFIG_OK && !entry->identity_read_overflow;
  if (!read_ok) {
    // **读失败**（超时/被 ABORT/回串超容量）：退避重试，用尽才定论。此前这里
    // 直接置 done，于是一次瞬态失败就让该台整场上电周期不可控（真机踩过）。
    entry->identity_read_attempts++;
    if (entry->identity_read_attempts < MOTOR_CONTROL_IDENTITY_READ_MAX_ATTEMPTS) {
      entry->identity_read_retry_tick_ms =
          now_ms + MOTOR_CONTROL_IDENTITY_READ_RETRY_MS;
      return; // done 保持 false → 0) 的不变量复检会重发
    }
  }
  entry->identity_read_done = true;

  const motor_protocol_model_profile_t *profile = NULL;
  if (read_ok) {
    profile = motor_control_profile_by_identity(entry->adapter, entry->identity,
        entry->identity_size);
  }
  if (profile != NULL) {
    (void)motor_control_bind_profile_ptr(ctx, entry->motor_id, profile, now_ms);
    return;
  }
  // 身份不可得（重试用尽 / 目录无此串）：保持未绑定不可控，
  // 出 PROFILE_REJECTED 让上层看见"这台设备不认识"。
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
      entry->motor_id, entry->bus_index,
      MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
}

// ---------------------------------------------------------------------------
// 设备参数初始化（上电/重连回读设备 → 与固件记录比对）
//
// 口径（2026-09-21 owner 反转，此前是 09-17 的"以设备内为准"）：**固件记录是
// 权威**，设备里的参数要与它对齐；不一致即"未初始化"。
//  - 绑定完成后自动回读一次设备六对象（0x607D:01/02、0x6080、0x2538、0x2650:00、0x2541）；
//  - 回读值与记录逐槽位比对，比的是**设备端原值**：关节侧 SI 两端都过同一套
//    Kt×减速比换算，系数配错时两边一起错、比对恒通过，等于没比；
//  - 不一致或回读失败 → 该台"未初始化"：冻结授权、不出运动帧，直到参数被
//    重写（写事务会同时更新设备与记录）或重连后重新回读；
//  - **本机一条记录都没有 → NO_RECORD**（也是"未初始化"，首次上电不得例外：
//    "首次也当不一致，人工核对"）。判据是 record_established，不是记录结构里的
//    位图——上电归一会自己写保护开关（见 entry_has_record）；
//  - 记录里没有的槽位不参与比对（刚烧录、Flash 半空的电机不该整体卡住），
//    **唯一例外是方向槽（0x2541）**：设备侧它易失（每次上电回 0=CW），"设备读到
//    0"不含信息，故记录缺这一槽 = 方向未知 = 未初始化（见 motor_control_dev_mismatch_mask）。
// ---------------------------------------------------------------------------

// 记录与设备回读逐槽位比对（设备端原值）。返回值 = 不一致槽位位图
// （1u << MOTOR_PROTOCOL_PARAM_SLOT_*；0 = 一致）。位图随 DEV_INIT 事件带出去，
// 也经参数视图透出（DeviceParamReport 要能说"是页面哪一格对不上"）——否则
// 操作者按页面写回一个槽位、别的槽位仍不一致，结论还是未初始化，看起来就像
// "设置不生效"。
uint8_t motor_control_dev_mismatch_mask(const motor_control_entry_t *entry) {
  uint8_t mask = 0U;
  // 方向基线**独立于"本机有无记录"判定**：方向是易失对象，设备每次上电都回
  // 0（CW），"设备当前读到 0"零信息量——记录是它唯一的权威来源。无记录 /
  // 记录无此槽 / 槽值非法，一律等于"方向未知"，判未初始化（不猜 CW）。
  // 这是 owner 2026-09-20 的决定，也是 09-19 交接文档"无记录 = 未初始化"的落地。
  const uint8_t dir_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
  if ((entry->record_raw_present & dir_bit) == 0U ||
      entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION] > 1U) {
    mask |= dir_bit;
  }
  if (!entry->record_valid) {
    return mask; // 无记录：除方向基线外没有别的可比项
  }
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((entry->record_raw_present & bit) == 0U) {
      continue; // 记录里没写过这项：不构成不一致
    }
    if ((entry->dev_raw_present & bit) == 0U ||
        entry->dev_raw[slot] != entry->record_raw[slot]) {
      mask |= bit; // 记录有、设备没读到 → 也算对不上（无从确认一致）
    }
  }
  return mask;
}

static bool dev_params_match_record(const motor_control_entry_t *entry) {
  return motor_control_dev_mismatch_mask(entry) == 0U;
}

// 本机对该台有没有基线记录。**不能用 record_raw_present 判**：上电归一恒写保护
// 开关，那一笔会把它的 bit4 置起来，于是空 Flash 的板子在回读前就"有记录"了，
// NO_RECORD 永远判不出来。判据是 record_established（见字段注释）。
static bool entry_has_record(const motor_control_entry_t *entry) {
  return entry->record_established;
}

// 统一的比对结论（自动回读收尾与写事务后的重判共用）：
//   无记录 → NO_RECORD；有记录 → 逐槽位比 → OK / MISMATCH。
// 写事务会就地建立记录，所以"写过一项"就不再是 NO_RECORD——但那不等于通过：
// 记录没写齐（典型是方向无基线）仍判 MISMATCH 并由位图点名，不伪造 OK。
static motor_control_dev_init_t settle_dev_init(
    const motor_control_entry_t *entry) {
  if (!entry_has_record(entry)) {
    return MOTOR_CONTROL_DEV_INIT_NO_RECORD;
  }
  return dev_params_match_record(entry) ? MOTOR_CONTROL_DEV_INIT_OK
                                        : MOTOR_CONTROL_DEV_INIT_MISMATCH;
}

// 冻结为 CONFIG_CONFLICT 的统一入口（设备参数未初始化 / 方向翻转共用）：
// 先把"受控集合"里的电机停下来，再清授权/目标/使能位并写成因。
// 受控集合由 motor_control_soft_stop_entry 自己界定——ACTIVE，或**使能整定窗内
// 的 READY_DISABLED（使能帧已发、motion_enabled=true，生命周期还没到 ACTIVE）**。
// 那个窗内状态必须覆盖：跳过它就会让设备带着上一条命令保持使能，而内核已标记
// 冻结——设备限位拦得住"跑出范围"，拦不住"带着旧零点/旧命令继续走"。
// 事件由各调用方按自己的语义域补出（DEV_INIT 报定论、CONFIG_CONFLICT 报成因码），
// 本助手只管"停/清/写码"。
static void freeze_entry_as_conflict(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint16_t fault,
    uint32_t now_ms) {
  if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_ACTIVE ||
      entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_SAFE_STOP ||
      (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
      entry->motion_enabled)) {
    motor_control_soft_stop_entry(ctx, entry, now_ms);
  }
  entry->motion_enabled = false;
  entry->enable_pending = false;
  entry->has_target = false;
  entry->motion_authorized = false;
  entry->policy_enable = false;
  entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT;
  entry->unified_fault_code = fault;
  entry->last_failure_reason = fault;
}

// 定论落地：写 dev_init，并按结论放行/冻结该台。调用点必须在 tx_active
// 复位之后（冻结路径与故障路径同形，避免事务重入）。
static void apply_dev_init_result(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    motor_control_dev_init_t state,
    uint32_t now_ms) {
  // 位图随事件带出去（1u<<slot；0 = 无此项信息），但两态语义不同：
  //  - MISMATCH：**对不上**的槽位（记录有、设备读到的值不同）；
  //  - UNVERIFIED：**没读到**的槽位（refresh_slots 是本次已落袋的槽位，
  //    取补集即缺失项）。位图是操作者区分"它没答"与"总线抖动"的唯一线索；
  //    曾靠它定位到 0x2650 读错子索引（2026-09-19 真机，TBD-CONFIG-02 见
  //    cerb_adapter.c 的安全槽注释）。
  const uint32_t detail =
      (state == MOTOR_CONTROL_DEV_INIT_MISMATCH)
          ? (uint32_t)motor_control_dev_mismatch_mask(entry)
          : (state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED)
                ? (uint32_t)(MOTOR_CONTROL_DEV_RAW_ALL & ~entry->refresh_slots)
                : 0U;
  const bool ready = (state == MOTOR_CONTROL_DEV_INIT_NA ||
      state == MOTOR_CONTROL_DEV_INIT_OK);
  const bool was_dev_block =
      entry->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM;
  entry->dev_init = state;
  if (ready) {
    if (was_dev_block) {
      entry->unified_fault_code = MOTOR_CONTROL_FAULT_NONE;
      entry->last_failure_reason = MOTOR_CONTROL_FAULT_NONE;
      if (entry->lifecycle == MOTOR_CONTROL_LIFECYCLE_CONFIG_CONFLICT) {
        entry->lifecycle = MOTOR_CONTROL_LIFECYCLE_READY_DISABLED;
        motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_READY,
            entry->motor_id, entry->bus_index,
            MOTOR_CONTROL_FAULT_NONE, 0U);
      }
    }
  } else if (state == MOTOR_CONTROL_DEV_INIT_MISMATCH ||
      state == MOTOR_CONTROL_DEV_INIT_UNVERIFIED ||
      state == MOTOR_CONTROL_DEV_INIT_NO_RECORD) {
    // 未初始化（不一致 / 未能核对 / 本机无记录）：**运动中的走软停，不硬失能**
    // ——上电自检与写入都核验过，真出现不一致属于异常，此时按"停止运动但保留
    // 阻尼"处理（TBD-SAFETY-01 口径），不把力矩瞬间撤掉。
    // 静止态（READY_DISABLED 等）没有要停的东西，直接锁存结论。
    // 与冲突/故障同口径冻结（ARD §36：失败不自动恢复运动授权）：结论未转好
    // 之前不得使能，恢复需显式 SET_ENABLE(true) + 新快照。
    //
    // NO_RECORD 走同一条路（owner 2026-09-21："首次也当不一致，人工核对"）：
    // 此前它落进 NA 而 NA 是放行的，等于首次上电未经核对即可使能。
    freeze_entry_as_conflict(ctx, entry, MOTOR_CONTROL_FAULT_DEV_PARAM, now_ms);
  }
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_DEV_INIT,
      entry->motor_id, entry->bus_index, (uint16_t)state,
      detail);
}

// 回读成功：影子取设备真值（设备为准），六个槽位的 SI 值同步落袋。
static void update_shadow_from_read(motor_control_entry_t *entry) {
  entry->dev_min_rad = entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN];
  entry->dev_max_rad = entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX];
  entry->dev_max_velocity_rad_s =
      entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX];
  entry->dev_max_torque_nm =
      entry->dev_si[MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX];
  entry->dev_limits_valid = true;
  entry->limits_source = MOTOR_CONTROL_LIMITS_SOURCE_DEVICE;
}

// 写事务成功：设备与固件记录一起更新（"写就是设备和驱动板一起写"）。
// 记录落 raw + SI，下次上电比对才不会把刚写过的值判成不一致。
// 只置脏位、**不落盘**（落盘由 SAVE 命令触发，见 record_dirty_slots）。
static void update_record_after_write(motor_control_entry_t *entry) {
  const uint8_t pos_min_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN);
  const uint8_t pos_max_bit =
      (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX);
  // 本次事务实际写到的槽位（只写这些：REFRESH 的只读槽在 tx_write_raw 里恒为
  // 0，按 record_raw_present 全量刷会把它们抹成 0）。
  uint8_t written_mask = 0U;
  switch (entry->tx_kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
    written_mask = (uint8_t)(pos_min_bit | pos_max_bit);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT:
    // 保护开关写成功后，记录与设备同值 → 上电比对不再报 CONFLICT。
    // 注意这同时把"变更检测"的基准挪到了新值：写 1（关保护）再落盘即
    // 基线化为"关"，与既有的语义一致（见决策日志）。
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION:
    written_mask = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
    break;
  default:
    return;
  }
  // 先算"是否真的变了"：值相同且记录已有该位 → 不置脏位。上电归一每次都会写
  // 保护"开"，不做这一步就是每次上电白排一次保存（设备 0x2539 落 flash 会阻塞
  // 自身控制循环）。
  // 必须在 record_raw_present 置位**之前**判，否则首次基线会被误判成不变。
  bool changed = false;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    if ((entry->record_raw_present & bit) == 0U ||
        entry->record_raw[slot] != entry->tx_write_raw[slot]) {
      changed = true; // 首次建立基线也算变化
    }
  }
  entry->record_raw_present |= written_mask;
  entry->dev_raw_present |= written_mask;
  for (uint8_t slot = 0U; slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT; ++slot) {
    const uint8_t bit = (uint8_t)(1U << slot);
    if ((written_mask & bit) == 0U) {
      continue;
    }
    // 刚写并读回核验过：设备与记录同值。
    entry->record_raw[slot] = entry->tx_write_raw[slot];
    entry->dev_raw[slot] = entry->tx_write_raw[slot];
    // 同一笔事务产生的关节侧值，两行（"固件" / "驱动板"）共用的那把尺子。
    // 位置/速度/力矩取 tx_request（落事务的是钳制后的 bounded/clamped），
    // 两个安全槽是枚举/标志位原值直通。
    // 设备侧那一份也在写路径上更新（不只是 REFRESH）：写事务本就是**读回核验过**
    // 的，设备里确实就是写的这个值——上面的 dev_raw[slot] 已经据此把它登记成
    // "设备当前的值"，SI 若还留着写前的旧换算值，报告的"驱动板"那一行就会拿
    // 刚写过的槽位显示一个陈旧数字，两行比对也就没意义了。
    float si = NAN;
    switch (slot) {
    case MOTOR_PROTOCOL_PARAM_SLOT_POS_MIN:
      si = entry->tx_request.position_limits.min_position_rad;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_POS_MAX:
      si = entry->tx_request.position_limits.max_position_rad;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_VEL_MAX:
      si = entry->tx_request.max_velocity_rad_s;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_TORQUE_MAX:
      si = entry->tx_request.max_torque_nm;
      break;
    case MOTOR_PROTOCOL_PARAM_SLOT_SAFETY:
    case MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION:
      si = (float)entry->tx_write_raw[slot];
      break;
    default:
      break;
    }
    entry->record_si[slot] = si;
    entry->dev_si[slot] = si;
  }
  if (changed) {
    // 只置脏位，**不落盘**：设备侧 0x2539 与 W25 都等到人工点「保存」。
    // 不保存就掉电 → 两边一起回到上一轮已保存的值（设计 §3.2）。
    entry->record_dirty_slots |= written_mask;
  }
  // 只有**上层发起的写**才建立基线。上电归一（保护 / 方向）与它共用同一个收尾
  // 路径，但那是固件自己的动作：让它把 record_established 置起来，空 Flash 的
  // 板子上电即"有记录"，NO_RECORD 永远判不出来。
  // 判据 = 内部事务的 request_id 高位置位（MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE）；
  // 上层请求走 mailbox，id 最高位是 0（param_bridge 用 bit30 做标签）。
  if ((entry->tx_request_id & MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE) == 0U) {
    entry->record_established = true;
  }
  entry->record_valid = true;
}

// 设备写事务成功：用内核钳制后的实际请求（tx_request）更新设备影子，
// 保证影子 = 设备已核验值（读回核验通过才进这里）。
static void update_shadow_after_write(motor_control_entry_t *entry) {
  if (entry->profile == NULL ||
      (entry->profile->capability_flags &
      MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
    return;
  }
  switch (entry->tx_kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS:
    entry->dev_min_rad = entry->tx_request.position_limits.min_position_rad;
    entry->dev_max_rad = entry->tx_request.position_limits.max_position_rad;
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
    entry->dev_max_velocity_rad_s = entry->tx_request.max_velocity_rad_s;
    break;
  case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT:
    entry->dev_max_torque_nm = entry->tx_request.max_torque_nm;
    break;
  default:
    return;
  }
  entry->dev_limits_valid = true;
}

void motor_control_finish_tx(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint16_t result,
    uint32_t now_ms) {
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    finish_identity_read(ctx, entry, result, now_ms);
    return;
  }
  // 上电归一收尾：按 request_id 认领（人工写用的是同一个 kind，不能按 kind 判）。
  // 失败沿用回读那套退避重试，用尽才定论 UNVERIFIED——一次总线抖动不该让该台
  // 在"初始化未完成不可使能"的口径下白白卡住。
  bool boot_norm_failed_offline = false;
  if (entry->boot_norm_request_id != 0U &&
      entry->tx_request_id == entry->boot_norm_request_id) {
    entry->boot_norm_request_id = 0U;
    if (result == MOTOR_PROTOCOL_CONFIG_OK) {
      entry->boot_norm_stage =
          entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT
              ? MOTOR_CONTROL_BOOT_NORM_DIRECTION
              : MOTOR_CONTROL_BOOT_NORM_DONE;
    } else {
      entry->boot_norm_attempts++;
      if (entry->boot_norm_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS) {
        boot_norm_failed_offline = true;
      } else {
        entry->boot_norm_retry_tick_ms = now_ms + MOTOR_CONTROL_DEV_READ_RETRY_MS;
      }
    }
  }
  // 设备参数定论（回读结果）延后到 tx_active 复位之后再落地：apply_dev_init_result
  // 会走冻结路径，事务若仍在飞会被重入。
  bool dev_init_settled = false;
  motor_control_dev_init_t dev_state = MOTOR_CONTROL_DEV_INIT_NA;
  if (boot_norm_failed_offline) {
    // 归一写不进去 = 保护/方向没到位。该台按"未核对"收口（UNVERIFIED），不发起
    // 回读、不与记录比对——比对的是"归一的成果"，归一没成就没有成果可比。
    // 结论把 lifecycle 打到 CONFIG_CONFLICT ⇒ 不可使能 ⇒ 0a 不再重试（脱困靠
    // 重连/手动重读）。
    dev_state = MOTOR_CONTROL_DEV_INIT_UNVERIFIED;
    entry->dev_read_done = true;
    dev_init_settled = true;
  } else if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    entry->dev_read_attempts++; // 每次回读收尾都计入（成功也算一次尝试）
    // 回读成功的判据：全部槽位的原值位图到齐（每 ADVANCE 落袋一个槽，
    // 重复/缺失槽位图凑不齐 MOTOR_CONTROL_DEV_RAW_ALL）。
    if (result == MOTOR_PROTOCOL_CONFIG_OK &&
        entry->refresh_slots == MOTOR_CONTROL_DEV_RAW_ALL) {
      // 全部阶段到齐才提交：设备端原值位图一次性换成本次读到的槽位。
      entry->dev_raw_present = entry->refresh_slots;
      update_shadow_from_read(entry);
      dev_state = settle_dev_init(entry);
      entry->dev_read_done = true;
      dev_init_settled = true;
    } else {
      // 读不到设备：先退避重试，用尽尝试次数才定论 UNVERIFIED。读是纯只读
      // 事务，多问几次没有副作用；一次总线抖动就判"未核对"会让该台在
      // "初始化未完成不可使能"的口径下白白卡住。
      if (entry->dev_read_attempts >= MOTOR_CONTROL_DEV_READ_MAX_ATTEMPTS) {
        dev_state = MOTOR_CONTROL_DEV_INIT_UNVERIFIED;
        entry->dev_read_done = true;
        dev_init_settled = true;
      } else {
        entry->dev_init = MOTOR_CONTROL_DEV_INIT_PENDING; // 未定论：等退避重试
        entry->dev_read_retry_tick_ms =
            now_ms + MOTOR_CONTROL_DEV_READ_RETRY_MS;
      }
    }
  } else if (result == MOTOR_PROTOCOL_CONFIG_OK) {
    update_shadow_after_write(entry);
    update_record_after_write(entry);
    // 写后重判：刚核验过的槽位已与记录同值，其余槽位仍按上次回读结论。
    if (entry->dev_read_done) {
      dev_state = settle_dev_init(entry);
      dev_init_settled = true;
    }
  }
  // 保存事务成功：脏位清除（设备侧落盘已发出；W25 由上层在同一动作里落）。
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH &&
      result == MOTOR_PROTOCOL_CONFIG_OK) {
    entry->record_dirty_slots = 0U;
  }
  entry->tx_active = false;
  entry->tx_awaiting = false;
  // 模式事务驱动过厂家使能序列（vaxor stop/start）：无论成败，使能序列
  // 重走——若事务异常停在 stop 态而内核仍记"已使能"，后续快照不再发使能帧，
  // 电机将哑到人工干预。强制复位只多一帧使能，两种结局都安全。
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_SET_MODE) {
    entry->motion_enabled = false;
    entry->enable_pending = false;
  }
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry->motor_id, entry->bus_index, result,
      entry->tx_request_id);
  if (dev_init_settled) {
    apply_dev_init_result(ctx, entry, dev_state, now_ms);
  }
}

// 事务中止（离线 / 冲突 / 厂家故障 / 撤权路径共用）：不是事务自身走出的
// 裁决，而是"设备/会话状态变了，别等了"。
//  - 配置事务：回 UNCONFIRMED 终态，避免上层请求悬挂到自身超时。
//  - 身份读取：不回任何事件、也不置 identity_read_done——中止只说明本轮读
//    没走完，不代表设备不认识自己；表项一旦回到可绑定态（重连）会重新发起。
//    若在这里定论为"识别失败"，一次断链就能让电机在本次上电周期内永久
//    不可控。
void motor_control_abort_tx(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  if (ctx == NULL || entry == NULL || !entry->tx_active) {
    return;
  }
  if (entry->tx_kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY) {
    entry->tx_active = false;
    entry->tx_awaiting = false;
    entry->identity_size = 0U;
    entry->identity_read_overflow = false;
    return;
  }
  motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED, now_ms);
}

// 事务成功收尾：无确认手段的协议不得谎报 OK（ARD §20.5 / 禁止项 9）。
// 例外是读事务：确认就是设备回帧本身，与"写阶段无读回对象"判据无关。
static void finish_tx_success(motor_control_context_t *ctx,
    motor_control_entry_t *entry, uint32_t now_ms) {
  const bool confirmed =
      entry->tx_kind == MOTOR_PROTOCOL_CONFIG_READ_IDENTITY ||
      entry->adapter == NULL || !entry->adapter->config_verify_required;
  const uint16_t result = confirmed ? (uint16_t)MOTOR_PROTOCOL_CONFIG_OK
                                    : (uint16_t)MOTOR_PROTOCOL_CONFIG_UNCONFIRMED;
  motor_control_finish_tx(ctx, entry, result, now_ms);
}

// 同总线上是否已有别的表项"已发未答"（exclude 自身不计）。
//
// 厂家的配置事务（CERB 的 CANopen SDO）是**严格一问一答**：说明书里所有示例
// 都是一条请求对一条应答。并发发问会连环出事——设备按序处理请求，迟到的应答
// 与本阶段索引不符被判 RETRY，重发又把设备压得更久，最后表现为"一直超时"，
// 而重发的请求还会挤掉正常控制帧。故按总线串行：任一时刻一条总线上至多一个
// 未决阶段（新阶段发出前先让上一问落地）。无应答阶段（VAXOR 的本地策略）不受
// 影响；bus_index 相同的表项才是同一物理总线。
static bool bus_stage_in_flight(const motor_control_context_t *ctx,
    const motor_control_entry_t *exclude) {
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    const motor_control_entry_t *other = &ctx->entries[i];
    if (other == exclude || !other->used) {
      continue;
    }
    if (other->bus_index == exclude->bus_index && other->tx_active &&
        other->tx_awaiting) {
      return true;
    }
  }
  return false;
}

// 超时/重试判定先行，随后至多推进一个阶段（跨节拍步进，ARD §20.6）。
void motor_control_config_step(motor_control_context_t *ctx, uint32_t now_ms) {
  if (ctx == NULL) {
    return;
  }

  // 0) 身份识别：不变量式重启——"在线 + 处于可绑定生命周期 + 本轮未定论 +
  // 无事务在飞"即发起（每拍至多一台，避免单拍突发撑开 1ms 节拍）。
  // 不变量而非一次性标志的理由：事务可能被离线/撤权/故障路径中止，中止
  // 不构成识别定论（见 motor_control_abort_tx），此刻若不复检就再没有
  // 发起时机——一次断链会让电机在本次上电周期内永久不可控。
  // 定论过的（identity_read_done）不再重试：认不出的设备不会因为多问几次
  // 就认识，重试只会刷屏事件环并占用总线。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->identity_read_done || entry->tx_active) {
      continue; // adapter==NULL 防御：否则本拍 break 会挡住后续表项的识别
    }
    if (entry->identity_read_attempts > 0U &&
        (int32_t)(now_ms - entry->identity_read_retry_tick_ms) < 0) {
      continue; // 上一次读失败的退避窗内：到点再重试
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // ACTIVE/FAULT/OFFLINE/冲突态不识别（冲突锁存不得被利用）
    }
    motor_control_start_identity_read(ctx, entry, now_ms);
    break;
  }

  // 0a) 上电归一：保护写"生效"、方向写 W25 记录值，**必须在 0b 回读之前**。
  //     与 0b 同为不变量式复检、每拍至多一台、走总线一问一答串行。
  //     记录里没有方向槽（新烧板 / v2 迁移的旧 blob）时跳过方向步：方向只能由
  //     记录说了算，猜一个 CW 会让反装的关节朝反方向冲（比对层会判基线缺失）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->tx_active ||
        entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE) {
      continue;
    }
    if (entry->profile == NULL ||
        (entry->profile->capability_flags &
        MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
      continue; // 未绑定 / 非设备侧型号（VAXOR）：无设备对象可写
    }
    if (!entry->boot_record_offered || entry->motion_enabled) {
      continue; // 记录先落袋；运动中不动安全边界（防御，理论上到不了）
    }
    if (entry->boot_norm_attempts > 0U &&
        (int32_t)(now_ms - entry->boot_norm_retry_tick_ms) < 0) {
      continue; // 失败退避窗内
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // 运动中/异常态不归一（故障恢复回 READY_DISABLED 后会复检补做）
    }

    motor_protocol_config_request_t request = {0};
    request.motor_id = entry->motor_id;
    if (entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_PROTECT) {
      request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 2U;
      request.kind = MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT;
      request.safety_protect_raw = 0U; // 厂家原值 0 = 保护生效
    } else {
      const uint8_t dir_bit =
          (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION);
      const uint32_t dir_raw =
          entry->record_raw[MOTOR_PROTOCOL_PARAM_SLOT_DIRECTION];
      if ((entry->record_raw_present & dir_bit) == 0U || dir_raw > 1U) {
        entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_DONE; // 不猜方向
        continue;
      }
      request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 3U;
      request.kind = MOTOR_PROTOCOL_CONFIG_SET_DIRECTION;
      request.direction_raw = dir_raw;
    }
    entry->boot_norm_request_id = request.request_id;
    (void)start_tx(ctx, entry, &request, now_ms);
    if (!entry->tx_active) {
      // adapter 没有该 kind 的阶段表（设备侧型号却没有保护/方向对象）：
      // start_tx 只出了一条 UNSUPPORTED 事件、没起事务。按"无对象可写"收尾，
      // 否则每个节拍都会重开一次同样的空事务，而且 0b 永远等不到归一完成。
      entry->boot_norm_request_id = 0U;
      entry->boot_norm_stage = MOTOR_CONTROL_BOOT_NORM_DONE;
    }
    break;
  }

  // 0b) 设备参数上电回读：绑定完成 + Flash 记录已落袋 + **上电归一已完成** +
  //     在线 + 无事务在飞 + 本会话未定论 → 自动发起一次 REFRESH_PARAMS。
  //     与身份识别同为不变量式复检（重连后 record/结论复位，自然重跑），
  //     每拍至多一台。记录先落袋再回读：比对基准必须在比对之前就位，否则
  //     "无记录可比"会把不一致静默判成 OK。
  //     归一的门（boot_norm_stage）不可省：比对的是**归一的成果**，归一没走完
  //     就比，读到的是设备上电初值（保护关/方向 CW），每台 CERB 每次上电都会
  //     白报一次不一致（见 docs/plans/active/boot-normalize-design.md §3.5）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->online || entry->adapter == NULL ||
        entry->tx_active || entry->dev_read_done) {
      continue;
    }
    if (entry->profile == NULL ||
        (entry->profile->capability_flags &
        MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
      continue; // 未绑定 / 非设备侧型号（VAXOR）：无设备对象可比
    }
    if (!entry->boot_record_offered) {
      continue; // 2ms/500Hz 边界先套用 Flash 记录，再回读
    }
    if (entry->boot_norm_stage != MOTOR_CONTROL_BOOT_NORM_DONE) {
      continue; // 0a 还没收尾：先写定保护/方向，再回读比对（顺序是契约）
    }
    if (entry->dev_read_attempts > 0U &&
        (int32_t)(now_ms - entry->dev_read_retry_tick_ms) < 0) {
      continue; // 失败退避窗内：到点再重试
    }
    if (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_DISCOVERED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_IDENTIFYING &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED) {
      continue; // 运动中/异常态不主动回读（手动 REFRESH 仍可用）
    }
    motor_protocol_config_request_t request = {0};
    request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE + 1U;
    request.motor_id = entry->motor_id;
    request.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
    (void)start_tx(ctx, entry, &request, now_ms);
    break;
  }

  // 1) 等待响应超时：≤2 次自动重试，耗尽 → TIMEOUT。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->tx_active || !entry->tx_awaiting) {
      continue;
    }
    if ((int32_t)(now_ms - entry->tx_deadline_tick_ms) < 0) {
      continue;
    }
    entry->tx_retries++;
    if (entry->tx_retries > 2U) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_TIMEOUT, now_ms);
      continue;
    }
    entry->tx_awaiting = false; // 下方发送循环重发本阶段
  }

  // 2) 发送/推进：每节拍至多一个阶段迁移（无响应阶段逐节拍推进）。
  for (size_t i = 0; i < ctx->entry_count; ++i) {
    motor_control_entry_t *entry = &ctx->entries[i];
    if (!entry->used || !entry->tx_active || entry->tx_awaiting) {
      continue;
    }
    const motor_protocol_adapter_t *adapter = entry->adapter;
    if (adapter == NULL) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_DEVICE_ERROR, now_ms);
      continue;
    }
    if (bus_stage_in_flight(ctx, entry)) {
      continue; // 本总线还有一问没答：本拍不发，下一拍再看（一问一答）
    }
    motor_protocol_config_stage_info_t info = {0};
    if (adapter->config_stage_info == NULL ||
        !adapter->config_stage_info(entry->tx_stage, entry->profile,
        &entry->tx_request, &info)) {
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_DEVICE_ERROR, now_ms);
      continue;
    }
    if (info.param_raw_valid &&
        info.param_slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT) {
      // 写阶段的设备端原值在此登记（重试重发同值，覆盖无副作用）。
      entry->tx_write_raw[info.param_slot] = info.param_raw;
    }
    motor_protocol_tx_frame_t frame = {0};
    if (adapter->config_encode_stage != NULL &&
        adapter->config_encode_stage(entry->tx_stage, entry->profile, entry->node,
        entry->bus_index, &entry->tx_request,
        &frame) &&
        frame.size > 0U) {
      frame.tx_class = MOTOR_PROTOCOL_TX_CLASS_P2;
      if (!motor_control_push_tx(ctx, &frame)) {
        continue; // 缓冲满：下一节拍重试本阶段
      }
    }
    entry->last_commanded_tick_ms = now_ms;
    if (info.waits_response) {
      entry->tx_awaiting = true;
      entry->tx_deadline_tick_ms = now_ms + info.timeout_ms;
      continue;
    }
    entry->tx_stage++;
    entry->tx_retries = 0U;
    if (entry->tx_stage >= adapter->config_stage_count(entry->profile,
        &entry->tx_request)) {
      finish_tx_success(ctx, entry, now_ms);
    }
  }
}

// 响应帧裁决（on_raw_frame 调用）。返回 true 表示帧被事务消费
// （帧同时是反馈时，解码仍会在调用方继续执行）。
bool motor_control_config_on_frame(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_raw_frame_t *frame) {
  if (ctx == NULL || entry == NULL || !entry->tx_active || !entry->tx_awaiting) {
    return false;
  }
  const motor_protocol_adapter_t *adapter = entry->adapter;
  if (adapter == NULL || adapter->config_on_response == NULL) {
    return false;
  }
  motor_protocol_config_stage_info_t info = {0};
  if (!adapter->config_stage_info(entry->tx_stage, entry->profile,
      &entry->tx_request, &info) ||
      !info.waits_response) {
    return false;
  }
  // 读取类事务的载荷直接落进表项缓冲（容量固定，溢出由 adapter 置位，
  // finish 路径据此判读取失败）。按事务种类路由：身份串→identity，
  // 参数回读→refresh_stages（每阶段一个 motor_protocol_param_stage_t）。
  const bool is_refresh =
      entry->tx_kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
  const size_t param_stage_bytes = sizeof(motor_protocol_param_stage_t);
  motor_protocol_read_buffer_t read;
  if (is_refresh) {
    read.data = (uint8_t *)entry->refresh_stages;
    read.capacity = sizeof(entry->refresh_stages);
    read.size = (size_t)entry->refresh_count * param_stage_bytes;
    read.overflow = false;
  } else {
    read.data = entry->identity;
    read.capacity = MOTOR_CONTROL_IDENTITY_CAPACITY;
    read.size = entry->identity_size;
    read.overflow = entry->identity_read_overflow;
  }
  const motor_protocol_stage_outcome_t outcome =
      adapter->config_on_response(entry->tx_stage, entry->profile,
      &entry->tx_request, frame, &read);
  if (is_refresh) {
    entry->refresh_count = (uint8_t)(read.size / param_stage_bytes);
  } else {
    entry->identity_size = read.size;
    entry->identity_read_overflow = read.overflow;
  }
  if (outcome == MOTOR_PROTOCOL_STAGE_ADVANCE ||
      outcome == MOTOR_PROTOCOL_STAGE_COMPLETE) {
    if (is_refresh && info.param_raw_valid &&
        info.param_slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT &&
        entry->refresh_count > 0U) {
      // 本阶段刚追加的载荷（按阶段顺序追加，槽位由 adapter 声明）。
      const motor_protocol_param_stage_t *stage_payload =
          &entry->refresh_stages[entry->refresh_count - 1U];
      entry->dev_raw[info.param_slot] = stage_payload->raw;
      entry->dev_si[info.param_slot] = stage_payload->value_si;
      entry->refresh_slots |= (uint8_t)(1U << info.param_slot);
    } else if (info.param_raw_valid &&
        info.param_slot < MOTOR_PROTOCOL_PARAM_SLOT_COUNT) {
      // 写阶段：登记本事务实际编码出的设备端原值（成功收尾时同步记录）。
      entry->tx_write_raw[info.param_slot] = info.param_raw;
    }
  }
  switch (outcome) {
  case MOTOR_PROTOCOL_STAGE_ADVANCE:
    entry->tx_stage++;
    entry->tx_retries = 0U;
    entry->tx_awaiting = false;
    if (entry->tx_stage >=
        adapter->config_stage_count(entry->profile, &entry->tx_request)) {
      finish_tx_success(ctx, entry, frame->rx_tick_ms);
    }
    return true;
  case MOTOR_PROTOCOL_STAGE_COMPLETE:
    // 设备提前给出终态（分段传输末段）：剩余阶段不再发送。
    finish_tx_success(ctx, entry, frame->rx_tick_ms);
    return true;
  case MOTOR_PROTOCOL_STAGE_IGNORE:
    // 非事务应答（同节点周期上报等）：不消费本帧，不计重试——调用方仍会
    // 用它走解码/反馈路径。
    return false;
  case MOTOR_PROTOCOL_STAGE_RETRY:
    // 无需回退缓冲：adapter 契约规定只在 ADVANCE/COMPLETE 前追加，判
    // RETRY 前不得写入（见 motor_protocol_adapter.h 的 config_on_response），
    // 因此已收下的前序分段不会被重试重复拼接或丢弃。
    entry->tx_retries++;
    entry->tx_awaiting = false;   // 读回不一致 → 重发本阶段（≤2 次）
    if (entry->tx_retries > 2U) {
      // 读回核验不一致：不得返回 OK（禁止项 9）→ UNCONFIRMED。
      motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED,
          frame->rx_tick_ms);
    }
    return true;
  case MOTOR_PROTOCOL_STAGE_FAIL:
  default:
    motor_control_finish_tx(ctx, entry, MOTOR_PROTOCOL_CONFIG_UNCONFIRMED,
        frame->rx_tick_ms);
    return true;
  }
}

// ---------------------------------------------------------------------------
// 提交入口
// ---------------------------------------------------------------------------

static bool start_tx(motor_control_context_t *ctx, motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  const motor_protocol_adapter_t *adapter = entry->adapter;
  const uint8_t stages =
      adapter->config_stage_count != NULL
          ? adapter->config_stage_count(entry->profile, request)
          : 0U;
  if (stages == 0U) {
    // 厂家对象未决 / 协议不支持：统一 UNSUPPORTED（不猜厂家字节，TBD-CONFIG-01）。
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
        entry->motor_id, entry->bus_index,
        MOTOR_PROTOCOL_CONFIG_UNSUPPORTED, request->request_id);
    return true;
  }
  entry->tx_active = true;
  entry->tx_awaiting = false;
  entry->tx_request_id = request->request_id;
  entry->tx_kind = request->kind;
  entry->tx_request = *request;
  entry->tx_stage = 0U;
  entry->tx_retries = 0U;
  if (request->kind == MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS) {
    entry->refresh_count = 0U;
    entry->refresh_slots = 0U;
  }
  memset(entry->tx_write_raw, 0, sizeof(entry->tx_write_raw));
  return true;
}

// 身份读取发起（由 config_step 的不变量复检调用，每拍至多一台）。两级回退：
//   1) 协议有身份读对象 → 走事务机器读设备自报型号串（本函数只发起）；
//   2) 协议无身份读对象 → 目录内该协议唯一型号时直接绑定，多于一型则留
//      未绑定（不猜，保持不可控安全侧）。
void motor_control_start_identity_read(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    uint32_t now_ms) {
  if (ctx == NULL || entry == NULL || entry->adapter == NULL ||
      entry->identity_read_done) {
    return;
  }
  if (entry->tx_active) {
    return; // 事务在飞：下一拍由 config_step 的不变量复检
  }

  motor_protocol_config_request_t request = {0};
  request.request_id = MOTOR_CONTROL_INTERNAL_REQUEST_ID_BASE;
  request.motor_id = entry->motor_id;
  request.kind = MOTOR_PROTOCOL_CONFIG_READ_IDENTITY;

  const motor_protocol_adapter_t *adapter = entry->adapter;
  if (adapter->config_stage_count != NULL &&
      adapter->config_stage_count(entry->profile, &request) > 0U) {
    entry->identity_size = 0U;
    entry->identity_read_overflow = false;
    (void)start_tx(ctx, entry, &request, now_ms);
    return;
  }

  entry->identity_read_done = true;
  const motor_protocol_model_profile_t *sole =
      motor_control_profile_sole(adapter);
  if (sole != NULL) {
    (void)motor_control_bind_profile_ptr(ctx, entry->motor_id, sole, now_ms);
    return;
  }
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_PROFILE_REJECTED,
      entry->motor_id, entry->bus_index,
      MOTOR_CONTROL_FAULT_PROFILE_UNAVAILABLE, 0U);
}

// 设备侧限位/限值写必须在电机未运动时进行：厂家手册要求设置限位时电机失能；
// 运行中改动位置限位可能使当前位置立即越出新限位而触发厂家保护（0x0040/
// 0x0080）。与换绑门禁同口径——安全边界不得在运动中变动。
static bool device_limit_write_allowed(const motor_control_entry_t *entry) {
  return entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE &&
         entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_SAFE_STOP;
}

// 设备参数回读的前置契约：**不得在该台自己的上电归一（0a）完成之前回读**。
// 归一没走完时设备里还是上电初值（保护关 / 方向 CW），读回来必然与记录不符 →
// 假 MISMATCH → apply_dev_init_result 把 lifecycle 锁到 CONFIG_CONFLICT；而 0a 的
// lifecycle 门又把归一挡在外面，本次会话就再也回不来了（只能靠重连或人工写回
// 参数）。触发面是主机的 system.refresh 与 clear_fault，窗口从记录落袋一直到该台
// 归一收尾（归一失败时还有每次 ≥500ms 的退避窗，多台共享总线时更久）。
// dev_read_done 是逃生口：本会话已有定论（归一成功，或归一失败已定论
// UNVERIFIED）就允许读——运行中复查、以及"归一失败后手动重读"那条恢复路径
// 都靠它。非设备侧型号（VAXOR）没有设备对象，门禁不适用。
static bool device_refresh_allowed(const motor_control_entry_t *entry) {
  if (entry->profile == NULL ||
      (entry->profile->capability_flags &
      MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) == 0U) {
    return true;
  }
  return entry->dev_read_done ||
         entry->boot_norm_stage == MOTOR_CONTROL_BOOT_NORM_DONE;
}

static bool reject_invalid(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  ctx->config_rejected_count++;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry != NULL ? entry->motor_id : request->motor_id,
      entry != NULL ? entry->bus_index : 0U,
      MOTOR_PROTOCOL_CONFIG_INVALID, request->request_id);
  return false;
}

// 事务进行中的统一拒绝（一问一答：同一电机新请求直接 BUSY，不排队）。
static bool reject_busy(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  ctx->config_rejected_count++;
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry->motor_id, entry->bus_index,
      MOTOR_PROTOCOL_CONFIG_BUSY, request->request_id);
  return false;
}

// 该台设备侧保护是否**确知为关**（厂家原值 1 = 关）。未核验过（影子无该位）
// 不算"确知"——那种情况本来也到不了使能路径（设备参数未初始化会先拦）。
static bool motor_control_unprotected_needs_confirm(
    const motor_control_entry_t *entry) {
  const uint8_t bit = (uint8_t)(1U << MOTOR_PROTOCOL_PARAM_SLOT_SAFETY);
  return (entry->dev_raw_present & bit) != 0U &&
         entry->dev_raw[MOTOR_PROTOCOL_PARAM_SLOT_SAFETY] == 1U;
}

// 保护被手动关掉的台仍允许武装，但**每次**都要显式确认：关保护 = 该关节没有
// 任何限位在跑（CERB 是设备侧限位，固件没有软限位兜底）。判据取设备真值影子，
// 拒绝只进 last_failure_reason（不是设备故障，与 STALE_GENERATION 同待遇）。
static bool reject_if_unprotected_unconfirmed(
    motor_control_context_t *ctx, motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request, uint32_t now_ms) {
  if (!request->enable || request->confirm_unprotected ||
      !motor_control_unprotected_needs_confirm(entry)) {
    return false;
  }
  entry->last_failure_reason = MOTOR_CONTROL_FAULT_UNPROTECTED_UNCONFIRMED;
  (void)reject_invalid(ctx, entry, request, now_ms);
  return true;
}

// 策略落地：enable + MIT 增益，一次原子落地。SET_POLICY config kind 与批量
// 入口 motor_control_apply_policy 共用，避免两条路径的门禁与钳制漂移。
//
// 为什么必须原子：增益要先于使能到位，否则存在一个用旧 Kd 使能的窗口；拆成
// SET_GAINS + SET_ENABLE 两次调用会留下它，也会让「两笔只成一半」变成可观测
// 状态。
//
// 代数门（防「周期重发自动复权」）：「未武装 → 武装」是一次新意图，
// policy_generation 必须严格前进（初值 0，故首次武装至少要 1）。冻结态
// （FAULT/OFFLINE/CONFIG_CONFLICT/DEV_PARAM）与显式失能都会清掉
// policy_enable，所以本规则**不需要在那几条路径上额外锁存任何东西**。
// 它挡不住「每次都递增代数的周期发布者」：那与人工逐次授权在行为上不可
// 区分。ARD §36 的「显式」在机制层面到此为止，再往上靠上层自律。
bool motor_control_apply_policy_entry(motor_control_context_t *ctx,
    motor_control_entry_t *entry,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  if (entry->profile == NULL) {
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (!isfinite(request->kp) || !isfinite(request->kd) ||
      request->kp < 0.0f || request->kd < 0.0f) {
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (request->enable && !entry->policy_enable &&
      request->policy_generation <= entry->policy_generation) {
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_STALE_GENERATION;
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (request->enable && !motor_control_dev_params_allow_motion(entry)) {
    entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
    return reject_invalid(ctx, entry, request, now_ms);
  }
  if (reject_if_unprotected_unconfirmed(ctx, entry, request, now_ms)) {
    return false;
  }
  // 增益先在本地落定（钳到型号上限），再动使能位——顺序即语义。
  entry->kp =
      request->kp > entry->profile->kp_max ? entry->profile->kp_max : request->kp;
  entry->kd =
      request->kd > entry->profile->kd_max ? entry->profile->kd_max : request->kd;
  entry->has_kp_kd = true;
  entry->policy_generation = request->policy_generation;
  if (request->enable) {
    entry->policy_enable = true;
    entry->motion_authorized = true;
  } else {
    apply_entry_disable(ctx, entry, now_ms);
  }
  motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
      entry->motor_id, entry->bus_index,
      MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
  return true;
}

bool motor_control_submit_config(motor_control_context_t *ctx,
    const motor_protocol_config_request_t *request,
    uint32_t now_ms) {
  if (ctx == NULL || request == NULL || request->request_id == 0U) {
    return false;
  }
  motor_control_entry_t *entry = motor_control_find_entry(ctx, request->motor_id);
  if (entry == NULL) {
    ctx->config_rejected_count++;
    return false; // 未知电机：无表项可挂事件，仅计数
  }

  switch (request->kind) {
  case MOTOR_PROTOCOL_CONFIG_SET_GAINS: {
    // 增益是低频 Policy（ARD §19.4）：本地生效，上限来自 Profile，
    // 不进入命令快照、不刷新命令 freshness。
    if (entry->profile == NULL) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (!isfinite(request->kp) || !isfinite(request->kd) || request->kp < 0.0f ||
        request->kd < 0.0f) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    entry->kp = request->kp > entry->profile->kp_max ? entry->profile->kp_max
                                                       : request->kp;
    entry->kd = request->kd > entry->profile->kd_max ? entry->profile->kd_max
                                                       : request->kd;
    entry->has_kp_kd = true;
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
        entry->motor_id, entry->bus_index,
        MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
    return true;
  }
  case MOTOR_PROTOCOL_CONFIG_SET_POSITION_LIMITS: {
    if (!isfinite(request->position_limits.min_position_rad) ||
        !isfinite(request->position_limits.max_position_rad) ||
        request->position_limits.min_position_rad >=
        request->position_limits.max_position_rad) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    // 硬件参数表第三条：位置限位不得超过型号表的 MIT 量化范围
    // （CERB = ±π，与设备 0x607D 字典 ±32768 cnt 同域）。0 = 未确认不钳制。
    motor_protocol_config_request_t bounded = *request;
    const float pos_limit = entry->profile != NULL
                                  ? entry->profile->mit_position_limit_rad
                                  : 0.0f;
    if (pos_limit > 0.0f) {
      if (bounded.position_limits.min_position_rad < -pos_limit) {
        bounded.position_limits.min_position_rad = -pos_limit;
      }
      if (bounded.position_limits.max_position_rad > pos_limit) {
        bounded.position_limits.max_position_rad = pos_limit;
      }
    }
    if (entry->profile != NULL &&
        (entry->profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
      // 软件限位协议（Vaxor）：本地保存，打包前钳制（ARD §26.1）。
      entry->soft_min_rad = bounded.position_limits.min_position_rad;
      entry->soft_max_rad = bounded.position_limits.max_position_rad;
      entry->has_soft_limits = true;
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
          entry->motor_id, entry->bus_index,
          MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
      return true;
    }
    // 设备侧限位（CERB）：必须走 request_id 事务 + 读回核验（ARD §26.2），
    // 且仅限未运动状态（见 device_limit_write_allowed）。
    if (!device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, &bounded, now_ms);   // 落事务的是钳制后的请求
  }
  case MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT:
  case MOTOR_PROTOCOL_CONFIG_SET_TORQUE_LIMIT: {
    // 设备侧管理对象（仅 CERB）：0x6080 电机端最大转速 / 0x2538 电机端峰值
    // 电流，走 CANopen 快速 SDO 写 + 读回核验（ARD §20.4、§26.2）。
    // 请求为关节侧 SI；换算到电机端量由 adapter 依 profile 完成。
    if (entry->profile == NULL) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    motor_protocol_config_request_t clamped = *request;
    if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT) {
      if (!isfinite(request->max_velocity_rad_s) ||
          request->max_velocity_rad_s <= 0.0f) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      // 已确认额定转速的型号：限值不得高于额定（与 effort 峰值钳制同口径，
      // 不允许把驱动限值抬到型号标定之上）。
      const float rated = entry->profile->rated_speed_rad_s;
      if (rated > 0.0f && clamped.max_velocity_rad_s > rated) {
        clamped.max_velocity_rad_s = rated;
      }
    } else {
      if (!isfinite(request->max_torque_nm) || request->max_torque_nm <= 0.0f) {
        return reject_invalid(ctx, entry, request, now_ms);
      }
      const float peak = entry->profile->peak_torque_nm;
      if (peak > 0.0f && clamped.max_torque_nm > peak) {
        clamped.max_torque_nm = peak;
      }
      // 硬件参数表第二条：设备侧 0x2538 是**电气**上限，外部写入不得把设备里
      // 的电流推到规格书峰值电流之上。两条上限取更严者——今天两者等价
      // （限值系数就是峰值对），但任一列被改动后这条规则仍然成立。
      const float peak_amp = entry->profile->peak_current_amp;
      if (peak_amp > 0.0f) {
        const float cap_nm =
            peak_amp * motor_protocol_profile_limit_nm_per_amp(entry->profile);
        if (cap_nm > 0.0f && clamped.max_torque_nm > cap_nm) {
          clamped.max_torque_nm = cap_nm;
        }
      }
    }
    // 软件限位型号（VAXOR 无厂家管理对象）：本地保存，打包前钳制
    // （ARD §26.1，与位置软限位同口径）。只影响目标、不触发驱动器保护，
    // 故与增益一样**无生命周期门禁**——门禁只针对设备侧写。
    if ((entry->profile->capability_flags & MOTOR_PROTOCOL_CAP_LIMIT_SOFTWARE) != 0U) {
      if (request->kind == MOTOR_PROTOCOL_CONFIG_SET_VELOCITY_LIMIT) {
        entry->soft_max_velocity_rad_s = clamped.max_velocity_rad_s;
        entry->has_soft_vel_limit = true;
      } else {
        entry->soft_max_torque_nm = clamped.max_torque_nm;
        entry->has_soft_torque_limit = true;
      }
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
          entry->motor_id, entry->bus_index,
          MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
      return true;
    }
    // 设备侧（CERB）：0x6080 / 0x2538，SDO 写 + 读回核验；仅未运动时允许。
    if (!device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, &clamped, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_ZERO: {
    // 设零点（字典 0x2531 写 1）：厂家要求**失能状态**下设置——已使能一律
    // 拒绝，不隐式失能（与设备侧限位同口径：安全边界不得在运动中变动）。
    // 命令式事务：只等写应答，不读回；也不更新设备影子/记录（零点不是本机
    // 记录的参数，它改变的是设备的位置基准）。
    // 注意：零点一变，设备里 0x607D 的限位也随之换了物理位置，操作者需要
    // 重新确认限位与机械行程（见 docs 决策日志）。
    if (entry->profile == NULL || !device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_CLEAR_FAULT: {
    // 故障清除：只清「可清的」。三类故障的判据是**成因**（unified_fault_code）
    // 而不是生命周期——DEV_PARAM 与身份冲突都会把表项带进 CONFIG_CONFLICT，
    // 按生命周期判会把两者混为一谈。
    //  - 身份冲突锁存（latch_config_conflict）：ARD §9.3 刻意不可解除，复位
    //    只能整机重启 → 拒绝。
    //  - 设备参数未初始化（DEV_PARAM）：改走一次 REFRESH_PARAMS 事务，重跑
    //    回读与比对。真不一致仍然由 apply_dev_init_result 判 MISMATCH 保持
    //    CONFIG_CONFLICT——清除只负责「允许重走授权链」，不伪造一致。
    //  - 厂家故障：发起设备侧清错事务（协议无手段则 UNSUPPORTED）。设备清掉
    //    后下一帧状态帧走既有 FAULT → READY_DISABLED 路径收敛。
    // 清错帧是控制类帧（MIT 控制字），运动中的电机不得收到——与设备侧限位
    // 写同口径：安全边界不在运动中变动。
    if (entry->profile == NULL || !device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_CONFIG_CONFLICT) {
      return reject_invalid(ctx, entry, request, now_ms);   // 身份冲突不可解除
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    if (entry->unified_fault_code == MOTOR_CONTROL_FAULT_DEV_PARAM) {
      // 清 DEV_PARAM = 重跑一次回读：与手动 REFRESH **同一门禁**。今天 DEV_PARAM
      // 必然伴随 dev_read_done==true（两边都由 apply_dev_init_result 之外的定论
      // 路径置位），也就是这条转换本来就过得了门；但契约不该靠"非局部不变量"
      // 成立——就在这里显式挡一次。
      if (!device_refresh_allowed(entry)) {
        return reject_busy(ctx, entry, request, now_ms);
      }
      motor_protocol_config_request_t refresh = *request;
      refresh.kind = MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS;
      return start_tx(ctx, entry, &refresh, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_SAFETY_PROTECT: {
    // 设备侧看门狗/限位保护开关（0x2650:00）。
    //  - 关保护 = 该关节**没有任何限位在执行**（CERB 是
    //    CAP_LIMIT_DEVICE_SIDE，固件没有软限位兜底），所以这条与限值写
    //    同口径：**运动中不得变动安全边界**，仅未运动时受理。
    //  - 开保护有自锁风险：手册「看门狗数据帧间隔要小于 500ms，否则电机
    //    自锁」，而固件无命令时只发 50 Hz SYNC —— SYNC 是否算"看门狗数据
    //    帧"。台架实测开保护后停发全部帧静置 5 秒**没有自锁**，但那是单机
    //    空载静态观察。这里不拦（设备侧行为不是本层能保证的），但调用方
    //    （web/上层）必须把它当需要台架实测的动作。
    if (entry->profile == NULL || request->safety_protect_raw > 1U ||
        !device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_DIRECTION: {
    // 方向也是安全边界：与限位/保护同口径，仅未运动时受理。**2026-09-20 真机
    // 实测纠正**：翻向**不**改变位置坐标系（同一物理方向转，0x6064 的计数方向
    // 不变），所以限位与零点都不镜像、也不需要重新设零；真正的风险是驱动器
    // 自持的位置环变成正反馈——使能后会朝远离目标的方向冲到设备限位为止。
    if (entry->profile == NULL || request->direction_raw > 1U ||
        !device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SAVE_DEVICE_FLASH: {
    // 保存 = 设备落盘 + W25 落盘。这里只管设备那一半。
    // 没有设备侧落盘对象的两种台（VAXOR 的软限位只活在记录里；只脏了易失槽
    // 的保护/方向——厂家没给保存码）**不发任何帧**，直接清脏位回 OK：它们要靠
    // 上层落 W25 才算保存完成，走事务机器只会白开一个空事务。
    if (entry->profile == NULL) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    const bool device_side =
        (entry->profile->capability_flags &
        MOTOR_PROTOCOL_CAP_LIMIT_DEVICE_SIDE) != 0U;
    const uint8_t device_slots =
        (uint8_t)(request->save_slots & MOTOR_PROTOCOL_PARAM_SAVE_SLOTS_MASK);
    if (!device_side || device_slots == 0U) {
      entry->record_dirty_slots = 0U;
      motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
          entry->motor_id, entry->bus_index,
          MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
      return true;
    }
    // 设备侧 flash 写会让驱动器阻塞自身控制循环（阻塞量级未实测）。改动前 0x2539
    // 只作为写事务的尾巴发出，而写事务本就被 device_limit_write_allowed 挡着
    // ——ACTIVE 的电机从不会收到它。保存独立成事务后必须自己带这道门，否则
    // 一个正在持位的关节会因为点了一下"保存"被抽掉一秒的控制环。
    // 与限位/保护/方向同口径：安全边界相关的设备侧写，运动中一律拒绝。
    if (!device_limit_write_allowed(entry)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    motor_protocol_config_request_t bounded = *request;
    bounded.save_slots = device_slots;
    return start_tx(ctx, entry, &bounded, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_POLICY:
    return motor_control_apply_policy_entry(ctx, entry, request, now_ms);
  case MOTOR_PROTOCOL_CONFIG_SET_ENABLE: {
    // 逐电机 Enable（ARD §19）：本地策略位，非事务。
    // true = 重立运动授权（仍需新快照才能运动）；false = 立即退出运动：
    // 软停 + 失能帧 + 清授权位（ARD §36 冻结语义）。
    if (request->enable) {
      // 设备参数初始化未完成（未回读/回读失败/与记录不一致）：使能请求直接
      // 拒绝并说明原因，不静默吞掉——页面要能立刻看到"为什么按不下去"。
      if (!motor_control_dev_params_allow_motion(entry)) {
        entry->last_failure_reason = MOTOR_CONTROL_FAULT_DEV_PARAM;
        return reject_invalid(ctx, entry, request, now_ms);
      }
      if (reject_if_unprotected_unconfirmed(ctx, entry, request, now_ms)) {
        return false;
      }
      entry->policy_enable = true;
      entry->motion_authorized = true;
    } else {
      apply_entry_disable(ctx, entry, now_ms);
    }
    motor_control_push_event(ctx, now_ms, MOTOR_CONTROL_EVENT_CONFIG_RESULT,
        entry->motor_id, entry->bus_index,
        MOTOR_PROTOCOL_CONFIG_OK, request->request_id);
    return true;
  }
  case MOTOR_PROTOCOL_CONFIG_REFRESH_PARAMS: {
    // 设备参数回读：换算系数来自 Profile（未绑定拒绝）；上传只读，
    // ACTIVE 态也允许（不加 device_limit_write_allowed 门禁）。
    // 但**必须等该台自己的归一完成**：否则比的是设备上电初值（假不一致），
    // 而假不一致会冻结该台并把归一本身挡死（见 device_refresh_allowed）。
    if (entry->profile == NULL) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (!device_refresh_allowed(entry)) {
      return reject_busy(ctx, entry, request, now_ms);   // 归一未收尾：稍后重试
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  case MOTOR_PROTOCOL_CONFIG_SET_MODE: {
    if (request->mode > MOTOR_CONTROL_CONFIG_MODE_MAX) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    // 事务会驱动厂家使能序列：只对身份已确认、处于可配置生命周期且系统
    // 已授权的电机受理（未识别/未授权设备不得收到厂家使能帧）。
    if (entry->profile == NULL ||
        (entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_READY_DISABLED &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_ACTIVE &&
        entry->lifecycle != MOTOR_CONTROL_LIFECYCLE_SAFE_STOP)) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (!ctx->authorized) {
      return reject_invalid(ctx, entry, request, now_ms);
    }
    if (entry->tx_active) {
      return reject_busy(ctx, entry, request, now_ms);
    }
    return start_tx(ctx, entry, request, now_ms);
  }
  default:
    return reject_invalid(ctx, entry, request, now_ms);
  }
}
