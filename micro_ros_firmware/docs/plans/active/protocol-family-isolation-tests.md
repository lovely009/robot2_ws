# 族间隔离与重号冲突 —— 测试方案（2026-10-13）

## 背景与问题

当前「vaxor / CERB 不会互相混淆」这条性质**成立，但没有任何东西守着**：

1. 「两协议 ID 段不重叠」只是 `motor_control_discovery.c:52` 的一句注释，而归属
   循环是**首个命中即 break**——真重叠了不会报错，只会把某台电机按错协议解码。
2. `motor_protocol_cerb_adapter.match_rx_frame` 在 host 测试里**一次都没被调用过**
   （`grep -c` = 0）。vaxor 那个用例喂的"陌生 ID"是 `0x123, DLC 8`，
   **不是真实 CERB 帧**——跨族拒绝从未被验证。
3. 重号冲突的检测语义（跨总线 / 跨协议才报，同总线同协议不报）**没有测试固化**。

本方案纯加法：不动实时路径、不改 QoS 与 ID 基址、不烧录、不占台架。
全部是 host 测试，`ctest --test-dir .host-build` 即可跑。

## A. 族间帧归属隔离

新文件 `test/host/test_protocol_family_isolation.cpp`。

### A1 全 ID × DLC 扫描，断言两两认领集不相交（核心）

把每个 adapter 的 `match_rx_frame` 在
`can_id ∈ [0, 0x7FF]` × `dlc ∈ {0..8, 12, 16, 20, 24, 32, 48, 64}` 上全枚举一遍，
收集各自认领的 `(can_id, dlc)` 集合，断言任意两个 adapter 的**交集为空**。

- 把注释变成可执行不变量；新增第三个协议时会立刻炸出来
- 不相交成立即推出"首个命中即 break"的顺序无关，A1 一并覆盖该隐患
- 规模 ~2048×14 ≈ 2.9 万次/adapter，毫秒级

同时快照各自认领集的**基数与边界**，防基址被无意改动。

### A2 族间真帧拒绝（补上现在完全缺失的那条）

用真实帧形状互喂：

| 测试帧 | 形状 | 喂给 | 期望 |
|---|---|---|---|
| CERB 反馈 | `0x300+n`, DLC 12 | vaxor matcher | false |
| CERB SDO 应答 | `0x580+n`, DLC 8 | vaxor matcher | false |
| vaxor reply | `0x701+4n`, DLC 12 | cerb matcher | false |
| vaxor status | `0x702+4n`, DLC 20 | cerb matcher | false |

### A3 边界值（每个区间两侧各取一格）

- CERB 反馈：`0x300`(基址)、`0x301`(首合法)、`0x37F`(末合法)、`0x380`(越界)
- CERB SDO：`0x580`、`0x581`、`0x5FF`、`0x600`
- vaxor reply：`0x701`、`0x705`、`0x7F1`、`0x7F5`
- vaxor status：`0x702`、`0x706`、`0x7F2`、`0x7F6`
- 另断言 vaxor 只认偏移为 `NODE_STRIDE`(4) 倍数的 ID，非倍数的邻格必须拒绝

### A4 DLC 维度

正确 ID + 错误 DLC 必须不匹配（如 `0x722` 用 DLC 12 而非 20）；
覆盖 `{0,8,12,16,20,24,32,48,64}`。

### A5 扩展帧

`is_extended = true` 的帧两族都必须拒绝。

### A6 节点号范围

- vaxor：`0 / 1 / 60 / 61`
- cerb：`0 / 1 / 127 / 128`
- 显式断言**两族范围数值重叠**（`1..60` 同时合法）——不是 bug，但要固化，
  因为它说明"节点号跨协议必须唯一"是配置层的责任，编码层不提供保护

## B. 绑定协议门禁

并入同一文件。

| # | 调用 | 期望 |
|---|---|---|
| B1 | `profile_by_identity(vaxor, "R48LITE_V2", 10)` | **NULL**（CERB 身份串不得绑到 vaxor） |
| B2 | `profile_by_identity(cerb, "R48LITE_V2", 10)` | `cerb_r48_101` |
| B3 | `profile_by_identity(cerb, <vaxor 身份>, …)` | NULL |
| B4 | `profile_sole(vaxor_adapter)` | `vaxor_2550`（唯一） |
| B5 | `profile_sole(cerb_adapter)` | **NULL**（两型，不猜） |
| B6 | vaxor profile 的 `identity_string == NULL` | 兜底路径的前提，固化 |

## C. 重号 → 冲突（白盒，驱动状态机）

`test_motor_control.cpp` 已有白盒基建（`INCLUDES ../../lib/motor_control/src`），
新增用例：

| # | 场景 | 期望 |
|---|---|---|
| C1 | 同节点、**跨总线**、同协议 | CONFIG_CONFLICT |
| C2 | 同节点、同总线、**跨协议** | CONFIG_CONFLICT ← 主用例 |
| C3 | 同节点、同总线、同协议 | **不报冲突**（已知边界，固化它） |
| C4 | 锁存后状态 | `motion_enabled`/`has_target`/`motion_authorized`/`policy_enable` 全 false；推了 P0 失能帧 |
| C5 | 锁存后一致侧帧 | 只刷在线，**不解除** |
| C6 | 锁存后走一轮 OFFLINE→重连 | 冲突**仍不解除**（ARD §9.3） |
| C7 | 冲突事件 | 只推一次，不刷屏 |
| C8 | 孤儿 SDO 应答（可归属、无表项） | `unknown_frame_count++`，**不凭空建档** |

## D. 模拟器侧（扩展 `test/host/test_fdcan_motor_sim.py`）

| # | 场景 | 期望 |
|---|---|---|
| D1 | `--joint-map` 重号 | dict 静默塌缩——把 silent 行为固化，让"模拟器不报重号"这件事可见 |
| D2 | `schedule_feedback_window` | stagger 生效；上一轮未消费的 deadline **不被覆盖** |
| D3 | 墙钟兜底（2026-10-13 修的 bug） | 给定"SYNC 很慢/不来"的场景，窗口仍按 `fb_period` 登记——本次修复的回归测试。**需要先把 `run()` 里的兜底抽成一个小函数**才能驱动 |

## E. 明确不做

- 不改实时路径、QoS、CAN ID 基址、节点号范围
- 不烧录、不占台架、不动正在跑的仿真
- 不改 `clear_fault` / 身份重试策略（安全相邻，另行立项）

## 风险

纯加法。唯一的生产代码改动是 D3 把小段兜底逻辑抽成可测函数（行为等价）。
