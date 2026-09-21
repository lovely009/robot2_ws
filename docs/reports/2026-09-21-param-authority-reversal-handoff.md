# 参数权威方向反转：交接（2026-09-21）

承接 owner 2026-09-21 的决定：**固件内部维护的那份参数记录是权威，驱动板向它对齐**
（反转 2026-09-17 决策 §6.3「设备参数以驱动板为准」）。本文是这一轮的交接：落地了什么、
**哪些验过、哪些没验**、以及下一手要做什么。

- 活规格：[param-authority-reversal-design.md](../../micro_ros_firmware/docs/plans/active/param-authority-reversal-design.md)
- 实现计划（含「实现期勘误」一节，逐条记录计划自身被推翻的写法）：[param-authority-reversal-plan.md](../../micro_ros_firmware/docs/archive/param-authority-reversal-plan.md)
- 提交范围：`0161ea6..7e87f8c`（14 个提交，27 文件，+3390/−205），均在 `dev/robot_ws` 本地，**未 push**。

## 一句话

写与保存拆成了两次动作（写只改设备 RAM + 内存记录并置「未保存」，`system.save` 才写
`0x2539` 落驱动器 flash + 落 W25）；首次上电（无记录）不再静默放行，判 `NO_RECORD`
未初始化；新增只读低频话题 `micro_ros_up/device_param_report` 承载「固件记录 / 驱动板回读」
双行快照供人工核对。

## 验证状态（本文最重要的一节）

| 判据 | 怎么验的 | 结论 |
|---|---|---|
| 编译 | `pio run` | SUCCESS，无警告（RAM 54.7% / Flash 23.9%） |
| 内核/适配器/mailbox 逻辑 | host CTest | **19/19 通过**（含保存帧逐字节、`NO_RECORD` 拒使能、写不落盘、禁写门） |
| 固件确实换了 | `micro_ros_up/*` 话题数 | 8 → **9**（多出 `device_param_report`） |
| 新话题发布率 | `ros2 topic hz` | **~0.995 Hz**（min 1.000s / max 1.009s）= 设计的 1 Hz 心跳 |
| 双行内容 | `ros2 topic echo` | 四台 entry、12 个 float + 状态字段齐备、已核对台上两行逐格相同 |
| 写 → 只标该台未保存 | 写 `m1.vel_limit` 后读话题 | **只有 m1** `record_dirty: true`，另三台 false |
| 写 → 两行都更新（非陈旧） | 同上 | `rec_vel_limit` 与 `dev_vel_limit` **都是 0.5**（证明 `dev_si` 在写路径同步） |
| 写不发保存帧 | 模拟器日志 SDO 增量 | 只 +1 写应答 +1 读（写 + 读回核验两阶段）。旧固件此处是三阶段 |
| **写后不保存 = 不持久化** | 写 0.5 → 重启模拟器（设备掉电）+ 复位板子 → 读回 | **两边一起回到 0.6221**，仍判 `OK`、无脏位。旧行为在此会变 `MISMATCH` —— **此测试能区分新旧** |
| 初始化结论与堆 | `ros2 param dump` | `last_error: 0`、`heap_free: 79384`（与文档预估 ~79.6 KB / 72 槽吻合） |
| **`system.save` → `0x2539` 落盘** | — | **未验，见下** |
| 首次上电 `NO_RECORD` / `last_error=42` | — | **未验，见下** |

### 唯一缺口：`system.save` → `0x2539` 没有上板证据

板上固件跑的是新版，但触发保存的那条命令 `ros2 param set /micro_ros_node system.save true`
**被权限分类器拦下**（理由 "Remote Shell Writes"；同一条 `ros2 param set` 换成写
`m1.vel_limit` 却放行）。没有绕道（例如用网页「保存」按钮触发同一固件动作）——那属于规避该拒绝。

补验只需一行（跑完读话题应见脏位翻假、模拟器日志多出 SDO 写响应）：

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 param set /micro_ros_node system.save true
```

host 用例 `test_cerb_save_flash_transaction` 逐字节断言过 `0x2539` 帧（码 1/2/4 按脏槽选表）
与阶段数，内核受理路径也经静态审查；但**「点保存设备真的落盘」这件事没有在硬件上被观察到**。

### 未验：首次上电判 `NO_RECORD`

四台 W25 都已有记录，要造出「无记录」必须**擦净 W25**（破坏性操作），未做。这条**仅有 host 覆盖**
（`test_dev_init_no_record_blocks_enable`）。上真机新板时应能直接看到 `last_error=42`。

## 台架限制与正确用法（踩过的坑）

- **模拟器没有 flash/RAM 之分**（`fdcan_motor_sim.py` 把所有 SDO 对象放在一个 RAM dict，
  且不建模 `0x2539`，落到 abort）。所以「设备掉电」必须靠**重启模拟器**来模拟，**不是**复位板子：
  只复位板子时模拟器进程继续跑、值不会回退，结果会与设计预测相反、被误读成固件 bug。
  上面那条鉴别性测试就是这么做的。
- **烧录顺序是硬约束**：先停模拟器（MCU 挂起期间它仍发帧而无人 ACK，会把 CANable 打进
  bus-off 且不自动恢复）→ `pio run -t upload` → 起模拟器 `--gazebo`（让它成为 ACK 节点）
  → OpenOCD `reset run` 让板子带着模拟器在线启动。
- **改了 `.msg` 之后 host 侧要重建消息包**，否则 `ros2 topic echo` 报
  "message type ... is invalid"（板子在广播新话题，而本机类型支持是旧的）：
  `cd ros2_ws && colcon build --symlink-install --packages-select can_bridge_msgs`。
- **模拟器日志只记它自己发出的帧**（无 `dir:rx`），所以「板上到底发了什么」不能直接看，
  只能用 SDO 增量指纹推断；逐字节证据在 host 用例里。

## 对操作者：运行方式变了

以前写一项设备参数会**自动落盘**（写事务末尾跟 `0x2539` + 500ms 后自动落 W25）。
现在**必须点「保存」**才持久化 —— 这正是 owner 要的（「只点了写就该是未保存」）。
未保存的台在 `device_param_report.record_dirty` 上可见、会在页面上标黄（P2）。

另外：**保存被拒**（电机处于 ACTIVE/SAFE_STOP 时内核的运动禁写门会拒）会留下
「W25 已落、设备未落」的状态 ⇒ 下次上电该台判 `MISMATCH`、需重新核对。这是**已接受的设计行为**
（任何部分保存都必然在某一侧留下不一致，真正消除它需要逐台完成跟踪），已写进规格 §3.2。
拒绝本身非静默：脏位仍亮 + 内核 `CONFIG_RESULT`(INVALID) 进事件流。

## 本次执行中发现并修掉的四个真缺陷（教训在此）

1. **`save_slots` 的值域矛盾 → `system.save` 整条失效**（`0edbd82`）。mailbox 校验器要求位图不含
   四个可保存槽之外的位，而生产者传的是**全量脏位图**（必然含保护/方向两个易失槽）⇒ 而归一
   恒写保护 ⇒ **每台电机的首次保存都被挡在 mailbox 外**，内核「零帧清脏位」的短路根本到不了。
   *教训*：跨任务接口只核**类型**不核**值域**是会漏的。
2. **保存循环一次 poll 全量下发 → FIFO 溢出 → 假 `MISMATCH`**（`d4abb37`）。配置 FIFO 只有 16 深、
   消费侧每 5ms 抽 4 条。被拒的台设备侧没落盘，而 `storage_task_request_save()` 是**无条件整份快照**
   ⇒ W25 新、设备旧 ⇒ 下次上电判 `MISMATCH`。改为游标按拍分批、扫完一轮才落 W25。
   *教训*：**不能拿脏位当游标** —— 被禁写门拒掉的台脏位永远不清，「还有脏台吗」判「扫完了吗」永不收敛。
3. **保存分支丢了运动中禁写门**（`f38e856`）。改动前 `0x2539` 只作为写事务的尾巴发出，而写事务
   本就被 `device_limit_write_allowed` 挡着 ⇒ ACTIVE 的电机收不到它；保存独立成事务后这个性质
   **不会自动继承**，会让正在持位的关节因点了一下保存被抽掉一秒（驱动器写 flash 期间阻塞自身控制环）的控制环。
4. **报告力矩列单位与契约相反**（`f1028d3`）。发布 N·m，而规格、`.msg`、web README 三处都写 **A**，
   参数面 `m<id>.torque_limit` 也确实是 A ⇒ 同一屏同一个量两个数（R58：45 vs 10.8）。
   **上板实测坐实**：`m1.torque_limit = 1.0`（A）对 `rec_torque_limit = 4.1666665`（N·m）。

另有若干计划自身的写法在实现期被推翻（`record_raw_present` 判无记录不可用、`dev_si` 在写路径从不更新、
新 `.msg` 并不自动可见需列进 `CMakeLists`、测试断言写成不可证伪的形状等），逐条记在计划的
「实现期勘误」表里，带提交号。

## P2（web 大修）还欠什么

固件这版可用，但**面向它的界面还没做**——「两行对照人工核对」目前只能 `ros2 topic echo` 看。

1. **双行对照表**：固件列 / 驱动板列，同槽位同单位（位置 rad、速度 rad/s、力矩 **A**）；
   `dev_init_state == NO_RECORD` 时固件列整行显示「无记录」。
2. `record_dirty == true` 的台标黄 + 「保存」按钮（发 `system.save`）；
   **保存进行中（`system.save` 读 true）时禁用该按钮** —— 扫描窗口内落在游标之后的写会被 W25
   快照却没发设备帧（残留 Minor），这条零固件成本地封住它。
3. 保存被拒**必须从事件流**（`CONFIG_RESULT` 的 INVALID）显示成因，不能只依赖 `system.last_error`
   —— 那条按设计只承载初始化结论码（40/41/42）。
4. 总览四计数：已核对 / 未初始化 / **未保存** / 离线。
5. 页面现有两处 2 行映射要补：`DEV_INIT_STATE` 加第 6 项（`NO_RECORD`）、`why` 加错误码 42；
   否则新板首启那台的状态列渲染成空白（数据在话题里是对的）。

## 复现要点

```bash
# host 测试（改任何内核/适配器/mailbox 代码后必跑）
cd micro_ros_firmware
cmake -S test/host -B .host-build && cmake --build .host-build -j2
ctest --test-dir .host-build --output-on-failure

# 编译
~/.platformio/penv/bin/pio run

# 台架（顺序不可颠倒）
# 1) 停模拟器  2) 烧录  3) 起模拟器（--gazebo，成为 ACK 节点）  4) OpenOCD reset run
python3 micro_ros_agent/tools/fdcan_motor_sim.py --tty /dev/ttyACM1 --gazebo
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 topic hz /micro_ros_up/device_param_report        # 期望 ~1 Hz
ros2 topic echo --once /micro_ros_up/device_param_report
```

## 工作区里未提交的东西（不是本轮的）

工作树还压着**另一条工作流**的未提交改动（34 个已跟踪文件：`motor_rt_task.cpp` 的
5ms/200Hz 边界、P1 20ms 发送门、`pump_p0_p2`、`MotorTrajectoryState.msg`、若干 `tools/*.py`
与注释 churn 等）。本轮 15 个提交**刻意逐 hunk 把它们排除在外**（每次都已核对它们仍在工作区
且未暂存），因为一条署名 `SAVE_DEVICE_FLASH` 的提交不该把别人的在飞改动捆进来。
（Codex 留下的 `.firefox-profile.*` 两个目录在本轮结束时已不在磁盘上；非本轮删除。）
