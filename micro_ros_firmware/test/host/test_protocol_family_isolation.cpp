// 协议族间隔离 host test（方案见 docs/plans/active/protocol-family-isolation-tests.md）。
//
// 守两件事：
//   A. vaxor 与 CERB 的帧归属**互不越界**——此前这条只有一句注释
//      （motor_control_discovery.c:52「两协议 ID 段不重叠」），而归属循环是
//      首个命中即 break：真重叠了不报错，只会把电机按错协议解码（量纲错误）。
//      另：motor_protocol_cerb_adapter.match_rx_frame 此前在 host 测试里
//      **一次都没被调用过**，跨族拒绝从未被验证。
//   B. 型号绑定受协议门禁约束，身份串再像也跨不了族。
//
// 只走公共 API，不触碰内部状态。

#include <cassert>
#include <cstdio>
#include <cstring>

#include "motor_control_profile.h"
#include "motor_protocol.h"
#include "motor_protocol_adapter.h"

// ---------------------------------------------------------------------------
// 基建
// ---------------------------------------------------------------------------

static motor_protocol_raw_frame_t make_frame(uint32_t id, size_t len,
    bool extended = false) {
  motor_protocol_raw_frame_t f = {};
  f.bus_index = 0U;
  f.can_id = id;
  f.is_extended = extended;
  f.is_fd = true;
  f.is_brs = true;
  f.dlc = (uint8_t)len;
  f.rx_tick_ms = 100U;
  return f;
}

// 参与扫描的 adapter 全集。**新增协议必须加到这里**，否则 A1 的不相交断言
// 覆盖不到它——这是本文件唯一需要人工维护的点。
static const motor_protocol_adapter_t *const kAdapters[] = {
  &motor_protocol_cerb_adapter,
  &motor_protocol_vaxor_adapter,
};
static const size_t kAdapterCount = sizeof(kAdapters) / sizeof(kAdapters[0]);

static const uint8_t kDlcs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static const size_t kDlcCount = sizeof(kDlcs) / sizeof(kDlcs[0]);

// 归属判定的**完整**谓词：归属循环用的就是「matcher 命中 && valid_node」。
static bool attributed(const motor_protocol_adapter_t *a, uint32_t id, uint8_t dlc) {
  motor_protocol_raw_frame_t f = make_frame(id, dlc);
  uint16_t node = 0U;
  return a->match_rx_frame != NULL && a->match_rx_frame(&f, &node) &&
         motor_protocol_adapter_valid_node(a, node);
}

static const char *adapter_name(const motor_protocol_adapter_t *a) {
  return a == &motor_protocol_cerb_adapter ? "cerb" : "vaxor";
}

// ---------------------------------------------------------------------------
// A1. 全 ID × DLC 扫描：两两认领集不相交
// ---------------------------------------------------------------------------

static void test_claimed_sets_are_pairwise_disjoint() {
  // owner[id] = 认领该 (id, dlc) 的 adapter 下标，-1 表示无人认领。
  static int owner[0x800][sizeof(kDlcs) / sizeof(kDlcs[0])];
  for (size_t i = 0; i < 0x800U; ++i) {
    for (size_t d = 0; d < kDlcCount; ++d) {
      owner[i][d] = -1;
    }
  }

  size_t claimed[kAdapterCount] = {};
  uint32_t min_id[kAdapterCount];
  uint32_t max_id[kAdapterCount] = {};
  for (size_t a = 0; a < kAdapterCount; ++a) {
    // 注意：`= {0xFFFFFFFFU}` 只初始化第 0 个元素，其余被零初始化——
    // 曾因此让 vaxor 的 min 永远停在 0x000 却断言不到（数组聚合初始化陷阱）。
    min_id[a] = 0xFFFFFFFFU;
  }

  for (uint32_t id = 0U; id <= 0x7FFU; ++id) {
    for (size_t d = 0; d < kDlcCount; ++d) {
      for (size_t a = 0; a < kAdapterCount; ++a) {
        if (!attributed(kAdapters[a], id, kDlcs[d])) {
          continue;
        }
        // 核心断言：一格 (id, dlc) 至多被一个协议认领。
        if (owner[id][d] != -1) {
          printf("  !! (id=0x%03X, dlc=%u) 同时被 %s 与 %s 认领\n", id,
              kDlcs[d], adapter_name(kAdapters[owner[id][d]]),
              adapter_name(kAdapters[a]));
        }
        assert(owner[id][d] == -1);
        owner[id][d] = (int)a;
        claimed[a]++;
        if (id < min_id[a]) min_id[a] = id;
        if (id > max_id[a]) max_id[a] = id;
      }
    }
  }

  // 快照：认领集非空，且边界不漂移（基址被改动会在这里炸出来）。
  for (size_t a = 0; a < kAdapterCount; ++a) {
    assert(claimed[a] > 0U);
    printf("  %-5s 认领 %zu 格，id 范围 0x%03X..0x%03X\n",
        adapter_name(kAdapters[a]), claimed[a], min_id[a], max_id[a]);
  }
  // 硬断言而非仅打印：上面那个 min_id 初始化陷阱就是"只打印不断言"放过去的。
  for (size_t a = 0; a < kAdapterCount; ++a) {
    if (kAdapters[a] == &motor_protocol_cerb_adapter) {
      // 反馈 0x301-0x37F（127）+ SDO 应答 0x581-0x5FF（127）
      assert(claimed[a] == 254U);
      assert(min_id[a] == 0x301U && max_id[a] == 0x5FFU);
    } else if (kAdapters[a] == &motor_protocol_vaxor_adapter) {
      // reply n=1..60 → 0x705..0x7F1；status n=1..60 → 0x706..0x7F2
      assert(claimed[a] == 120U);
      assert(min_id[a] == 0x705U && max_id[a] == 0x7F2U);
    }
  }

  // 两族之间必须留有**空带**：CERB 最高 0x5FF、vaxor 最低 0x705。
  uint32_t cerb_max = 0U, vaxor_min = 0xFFFFFFFFU;
  for (uint32_t id = 0U; id <= 0x7FFU; ++id) {
    for (size_t d = 0; d < kDlcCount; ++d) {
      if (owner[id][d] < 0) continue;
      if (kAdapters[owner[id][d]] == &motor_protocol_cerb_adapter) {
        if (id > cerb_max) cerb_max = id;
      } else if (id < vaxor_min) {
        vaxor_min = id;
      }
    }
  }
  assert(cerb_max == 0x5FFU);   // CERB：反馈 0x301-0x37F + SDO 应答 0x581-0x5FF
  assert(vaxor_min == 0x705U);  // vaxor：reply 0x701+4n（n>=1）
  printf("  空带 0x%03X..0x%03X（%u 格宽），隔离有余量\n",
      cerb_max + 1U, vaxor_min - 1U, vaxor_min - cerb_max - 1U);
}

// ---------------------------------------------------------------------------
// A2. 族间真帧拒绝（此前完全缺失的一条）
// ---------------------------------------------------------------------------

static void test_cross_family_frames_are_rejected() {
  uint16_t node = 0U;
  const uint8_t n = 5U;

  // CERB 反馈（0x300+n, DLC 12）→ vaxor 必须拒绝
  motor_protocol_raw_frame_t cerb_fb =
      make_frame(MOTOR_PROTOCOL_CERB_FEEDBACK_ID_BASE + n, 12U);
  assert(!motor_protocol_vaxor_adapter.match_rx_frame(&cerb_fb, &node));

  // CERB SDO 应答（0x580+n, DLC 8）→ vaxor 必须拒绝
  motor_protocol_raw_frame_t cerb_sdo =
      make_frame(MOTOR_PROTOCOL_CERB_SDO_RESPONSE_ID_BASE + n, 8U);
  assert(!motor_protocol_vaxor_adapter.match_rx_frame(&cerb_sdo, &node));

  // vaxor reply（0x701+4n, DLC 12）→ cerb 必须拒绝
  motor_protocol_raw_frame_t vaxor_reply =
      make_frame(motor_protocol_vaxor_reply_id(n), 12U);
  assert(!motor_protocol_cerb_adapter.match_rx_frame(&vaxor_reply, &node));

  // vaxor status（0x702+4n, DLC 20）→ cerb 必须拒绝
  motor_protocol_raw_frame_t vaxor_status =
      make_frame(motor_protocol_vaxor_status_id(n), 20U);
  assert(!motor_protocol_cerb_adapter.match_rx_frame(&vaxor_status, &node));

  printf("  四类跨族真帧全部被对方拒绝\n");
}

// ---------------------------------------------------------------------------
// A3. 边界值
// ---------------------------------------------------------------------------

struct bound_case_t {
  uint32_t id;
  uint8_t dlc;
  bool vaxor_match;
  const char *why;
};

static void test_boundaries() {
  const bound_case_t cases[] = {
    // CERB 反馈段：0x301..0x37F（DLC 12）
    {0x300U, 12U, false, "CERB 反馈基址本身：vaxor 不认（远低于 0x701）"},
    {0x37FU, 12U, false, "CERB 反馈末格"},
    {0x380U, 12U, false, "CERB 反馈越界一格"},
    // CERB SDO 应答段：0x581..0x5FF（DLC 8）
    {0x580U, 8U, false, "CERB SDO 基址本身"},
    {0x5FFU, 8U, false, "CERB SDO 末格"},
    {0x600U, 8U, false, "CERB SDO 越界一格"},
    // vaxor reply 段：0x701+4n；基址本身必须被 > 排除
    {0x701U, 12U, false, "vaxor reply 基址：n=0 非法，用 > 排除"},
    {0x705U, 12U, true, "vaxor reply n=1（首合法）"},
    // vaxor status 段：0x702+4n
    {0x702U, 20U, false, "vaxor status 基址：n=0 非法"},
    {0x706U, 20U, true, "vaxor status n=1（首合法）"},
    // 非 stride 倍数的邻格必须拒绝
    {0x707U, 12U, false, "偏移非 4 的倍数（reply）"},
    {0x707U, 20U, false, "偏移非 4 的倍数（status）"},
  };
  const size_t count = sizeof(cases) / sizeof(cases[0]);
  for (size_t i = 0; i < count; ++i) {
    motor_protocol_raw_frame_t f = make_frame(cases[i].id, cases[i].dlc);
    uint16_t node = 0U;
    const bool got = motor_protocol_vaxor_adapter.match_rx_frame(&f, &node);
    if (got != cases[i].vaxor_match) {
      printf("  !! 0x%03X dlc=%u 期望 %d 实得 %d（%s）\n", cases[i].id,
          cases[i].dlc, (int)cases[i].vaxor_match, (int)got, cases[i].why);
    }
    assert(got == cases[i].vaxor_match);
  }

  // vaxor 的 matcher **不做上界校验**——node 60 是上限，但 0x7F1 之后
  // 算术上仍能命中更大的 node。上界由入口的 valid_node 把关（见 A6），
  // 这里把这个两层分工固化下来，防止有人误以为 matcher 自带边界。
  motor_protocol_raw_frame_t beyond = make_frame(0x701U + 4U * 61U, 12U);
  uint16_t node = 0U;
  assert(motor_protocol_vaxor_adapter.match_rx_frame(&beyond, &node));
  assert(node == 61U);
  assert(!motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, node));

  printf("  边界 %zu 例全部符合预期，且 matcher/valid_node 两层分工已固化\n", count);
}

// ---------------------------------------------------------------------------
// A4. DLC 维度
// ---------------------------------------------------------------------------

static void test_dlc_dimension() {
  // vaxor status 的正确 ID 用错 DLC（12 而非 20）→ 不匹配
  {
    motor_protocol_raw_frame_t f = make_frame(0x722U, 12U);
    uint16_t node = 0U;
    assert(!motor_protocol_vaxor_adapter.match_rx_frame(&f, &node));
  }
  // vaxor reply 的正确 ID 用错 DLC（20 而非 12）→ 不匹配
  {
    motor_protocol_raw_frame_t f = make_frame(0x721U, 20U);
    uint16_t node = 0U;
    assert(!motor_protocol_vaxor_adapter.match_rx_frame(&f, &node));
  }
  // CERB：同一个 ID 在 DLC 8 与 12 下走不同分支，其余 DLC 全不认
  for (size_t d = 0; d < kDlcCount; ++d) {
    const uint8_t dlc = kDlcs[d];
    motor_protocol_raw_frame_t fb = make_frame(0x301U, dlc);
    motor_protocol_raw_frame_t sdo = make_frame(0x581U, dlc);
    uint16_t node = 0U;
    assert(motor_protocol_cerb_adapter.match_rx_frame(&fb, &node) == (dlc == 12U));
    assert(motor_protocol_cerb_adapter.match_rx_frame(&sdo, &node) == (dlc == 8U));
  }
  printf("  DLC 维度 %zu 档全部符合预期\n", kDlcCount);
}

// ---------------------------------------------------------------------------
// A5. 扩展帧
// ---------------------------------------------------------------------------

static void test_extended_frames_rejected() {
  uint16_t node = 0U;
  const uint32_t ids[] = {0x301U, 0x581U, 0x705U, 0x706U};
  const uint8_t dlcs[] = {12U, 8U, 12U, 20U};
  for (size_t i = 0; i < 4U; ++i) {
    motor_protocol_raw_frame_t f = make_frame(ids[i], dlcs[i], true);
    assert(!motor_protocol_cerb_adapter.match_rx_frame(&f, &node));
    assert(!motor_protocol_vaxor_adapter.match_rx_frame(&f, &node));
  }
  printf("  扩展帧两族均拒绝\n");
}

// ---------------------------------------------------------------------------
// A6. 节点号范围（两族数值重叠是**事实**，固化它）
// ---------------------------------------------------------------------------

static void test_node_ranges() {
  assert(motor_protocol_vaxor_adapter.node_id_min == 1U);
  assert(motor_protocol_vaxor_adapter.node_id_max == 60U);
  assert(motor_protocol_cerb_adapter.node_id_min == 1U);
  assert(motor_protocol_cerb_adapter.node_id_max == 127U);

  assert(!motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 0U));
  assert(motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 1U));
  assert(motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 60U));
  assert(!motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, 61U));

  assert(!motor_protocol_adapter_valid_node(&motor_protocol_cerb_adapter, 0U));
  assert(motor_protocol_adapter_valid_node(&motor_protocol_cerb_adapter, 1U));
  assert(motor_protocol_adapter_valid_node(&motor_protocol_cerb_adapter, 127U));
  assert(!motor_protocol_adapter_valid_node(&motor_protocol_cerb_adapter, 128U));

  // 1..60 对两族**同时合法**：节点号互不冲突这件事编码层不提供保护，
  // 必须靠配置层保证唯一（注册表以裸 motor_id 为键，见 motor_control_discovery.c:98）。
  for (uint16_t n = 1U; n <= 60U; ++n) {
    assert(motor_protocol_adapter_valid_node(&motor_protocol_vaxor_adapter, n));
    assert(motor_protocol_adapter_valid_node(&motor_protocol_cerb_adapter, n));
  }
  printf("  节点范围 vaxor 1..60 / cerb 1..127，重叠 1..60 已固化\n");
}

// ---------------------------------------------------------------------------
// B. 绑定协议门禁
// ---------------------------------------------------------------------------

static void test_binding_is_protocol_scoped() {
  const uint8_t r48[] = {'R', '4', '8', 'L', 'I', 'T', 'E', '_', 'V', '2'};
  const uint8_t r58[] = {'R', '5', '8', 'L', 'I', 'T', 'E', '_', 'V', '2'};

  // B1：CERB 的身份串喂给 vaxor → 必须 NULL（跨不了族）
  assert(motor_control_profile_by_identity(&motor_protocol_vaxor_adapter,
      r48, sizeof(r48)) == NULL);
  // B2：同样的串喂给 cerb → 命中 R48
  const motor_protocol_model_profile_t *p =
      motor_control_profile_by_identity(&motor_protocol_cerb_adapter,
      r48, sizeof(r48));
  assert(p != NULL && strcmp(p->model_name, "cerb_r48_101") == 0);
  // R58 也应命中，且与 R48 不同型
  const motor_protocol_model_profile_t *p58 =
      motor_control_profile_by_identity(&motor_protocol_cerb_adapter,
      r58, sizeof(r58));
  assert(p58 != NULL && strcmp(p58->model_name, "cerb_r58_101") == 0);
  assert(p58 != p);

  // B3：容错——长度不符不得误命中
  assert(motor_control_profile_by_identity(&motor_protocol_cerb_adapter,
      r48, 0U) == NULL);
  assert(motor_control_profile_by_identity(&motor_protocol_cerb_adapter,
      r48, 3U) == NULL);

  // B4/B5：唯一型号兜底——vaxor 一型故可得，cerb 两型故必须 NULL（不猜）
  const motor_protocol_model_profile_t *sole_vaxor =
      motor_control_profile_sole(&motor_protocol_vaxor_adapter);
  assert(sole_vaxor != NULL && strcmp(sole_vaxor->model_name, "vaxor_2550") == 0);
  assert(motor_control_profile_sole(&motor_protocol_cerb_adapter) == NULL);

  // B6：vaxor 无身份读对象是兜底路径的**前提**，身份串必须是 NULL
  assert(sole_vaxor->identity_string == NULL);
  // 目录一致性：vaxor 恰好一型、cerb 恰好两型
  size_t vaxor_models = 0U, cerb_models = 0U;
  for (size_t i = 0; i < motor_control_profile_count(); ++i) {
    const motor_protocol_model_profile_t *m = motor_control_profile_at(i);
    if (m->protocol == &motor_protocol_vaxor_adapter) vaxor_models++;
    if (m->protocol == &motor_protocol_cerb_adapter) cerb_models++;
  }
  assert(vaxor_models == 1U);
  assert(cerb_models == 2U);

  printf("  绑定协议门禁成立；vaxor 1 型 / cerb 2 型\n");
}

int main() {
  printf("[A1] 全 ID × DLC 扫描：两两认领集不相交\n");
  test_claimed_sets_are_pairwise_disjoint();
  printf("[A2] 族间真帧拒绝\n");
  test_cross_family_frames_are_rejected();
  printf("[A3] 边界值\n");
  test_boundaries();
  printf("[A4] DLC 维度\n");
  test_dlc_dimension();
  printf("[A5] 扩展帧\n");
  test_extended_frames_rejected();
  printf("[A6] 节点号范围\n");
  test_node_ranges();
  printf("[B] 绑定协议门禁\n");
  test_binding_is_protocol_scoped();
  printf("test_protocol_family_isolation: 全部通过\n");
  return 0;
}
