# 全部分支覆盖与整合结果（2026-11-01）

已按扩大后的授权刷新远端全部真实分支。核查起点为本地 `integration/mcu-agent-web` 的 `32840319467982ae843dd955565787c6c00b2b48`，之前 UI、统一 Agent/租约入口、JTC 与固件修复均保留。

**结论：除保护主线外，远端六个分支的提交均已在起点中，没有新增提交需要合并，也没有新增文本或接口冲突。** 不重复 cherry-pick、不制造空合并。原有墙钟闭环验收失败仍保留，本轮没有宣称完整工程已验收。

## 逐分支处理

| 真实远端分支 | 本轮远端 SHA | 起点未覆盖提交数 | 处理 |
| --- | --- | ---: | --- |
| `feat/firmware-agent-topics-v3` | `e410243ae2dbe6eca62bc49c3347dfc8fa8d7dbf` | 0 | 原 B 分支，经 `3610450` 第二父链包含 |
| `feat/mcu-platform-integration` | `412f19d3f83a018dcf6942f0025914f39e18ba25` | 0 | 原 A 分支，经 `3610450` 第一父链包含 |
| `feat/managed-motion` | `6265a6259d9732ad7994445365b08cda551b4c21` | 0 | A 的祖先，已包含，保留现有主机 JTC 适配 |
| `feat/mcu-lease-v3` | `a8b62a261e8bbfebe407bad1eeb60e88b472eca4` | 0 | A 的祖先，已包含，保留整合后的 MCU 租约屏障 |
| `feat/shared-robot-description` | `9561cef092b91286df54d251e5a059d9dcb50429` | 0 | A 的祖先，已包含，共享模型保留 |
| `fix/control-reliability` | `f7211b32dc839dbc22ce34a3ce3304045253efc8` | 0 | A 的祖先，已包含，可靠性修复保留 |
| `dev/robot2_ws` | `525e3a946b30a2fabd0a6b07cf57a7544ddab251` | — | 保护主线，只核查，不修改/合入 |
| `main` | `6d3e92a1f271c339ee4d9afaa7c00b4a0f19c1eb` | — | 保护主线，排除；有1个未包含提交（删除 README），未合入 |

远端没有独立命名的 `dev` 分支。`origin/<名称>` 是上述真实分支的本地远程跟踪引用，不是另一条开发线；本地 `work` 位于 dev 同一 SHA，是原工作树的分支名，不是额外远端分支。本地 `integration/mcu-agent-web` 为当前交付目标，不作为源分支重复整合。仓库非 shallow，祖先判断不依赖截断历史。

新增核查分支所在链：

```text
dev/robot2_ws
  -> 36b00c6（SLCAN 可靠性）
  -> f7211b3 fix/control-reliability
  -> 9561cef feat/shared-robot-description
  -> a8b62a2 feat/mcu-lease-v3
  -> 6265a62 feat/managed-motion
  -> e00e39e（MCU 唯一实机后端）
  -> 412f19d feat/mcu-platform-integration
  -> 3610450（与 B 合并）-> 运行时/UI修复 -> 3284031
```

机器可读证据见 [all-branch-coverage.json](all-branch-coverage.json)。每个非保护分支都实际执行 `git merge-base --is-ancestor <源SHA> 3284031`（全部返回0），以及 `git rev-list --count 3284031..<源SHA>`（全部为0）。祖先证据证明提交已纳入，不代表源分支每项功能均已通过整合后的运行验收。

## 冲突与语义

本轮不需要额外代码合并，没有新增冲突或产品语义选择。之前两线整合的解决仍适用：B 主机 JTC/MicroRosSystem 经 A supervisor/租约进入唯一 Agent 执行路径；同名异构消息已统一、主机直连写入口不进入生产路径、共享模型安装单一来源。原 A 的 MCU Action 实现由统一主机标准 Action 路径适配，不重新恢复第二执行入口。详见 [UI与运行时记录](ui-and-runtime-review.md) 及 [后续修复/限制](wall-clock-followup.md)。

保持37轴和主机→MCU 50Hz、MCU→主机50Hz、MCU→电机200Hz。没有改动30/100ms安全期限、原始目标时间戳、显式授权要求或引入前瞻协议。取消与失能继续分离。

## 本轮实际验证

| 项目 | 结果 |
| --- | --- |
| `git ls-remote --heads origin` + fetch 全部分支 | 成功，8个真实分支；源 A/B/dev SHA 与此前一致 |
| 六个非保护分支祖先与未覆盖提交核查 | 6/6已覆盖，未覆盖提交均为0 |
| 原工作树 | 干净，HEAD保持 `525e3a946b30a2fabd0a6b07cf57a7544ddab251` |
| 固件 host 增量构建 | 通过，沿用原 Debug 构建目录 |
| `ctest --test-dir /workspace/robot2-integration-build/firmware --output-on-failure` | **37/37通过，1.02秒** |

本轮源码变化仅为分支证据与说明，没有重跑 ROS/网页/浏览器、ARM 或墙钟压力测试。先前 ARM 构建、前端100项、浏览器4项等为历史阶段结果，不计为本轮新运行。

## 仍有的失败与未覆盖

完整墙钟闭环曾单次通过，但最新稳定性验收未通过；迁移运行夹具的授权、连续目标与取消/重力等组合仍有失败或未达到断言，见墙钟记录中的逐项证据。本轮不重复无新诊断价值的随机重跑，也不改变测试期望掩盖失败。

下一步需要在具有可测调度/传输延迟上界的隔离环境验证持续运动；50Hz的20ms名义周期在原30ms目标期限内只剩不足10ms的总延迟余量。如果普通非实时环境必须容忍更大抖动仍持续运动，应另行授权评审前瞻协议，当前未实现。真实USB/CAN吞吐、停机时延、WCET、抱闸、装配及37轴分配未验收。

因此：**全部非保护分支的提交覆盖完成；完整工程验收未完成，尚不可合入 dev/main。** 未推送、删除源分支、部署、连接真实硬件或刷写固件。

交付压缩包包含独立整合分支的增量 Git bundle（前置基线为 dev 的 `525e3a946b30a2fabd0a6b07cf57a7544ddab251`）、从 dev 到交付 HEAD 的 patch、当前报告、分支快照、构建与测试日志，以及此前墙钟定位证据包。Git bundle保留基线以后的历史与本地提交，不包含构建缓存、凭据或固件二进制；patch不包含Git历史。恢复时建议在已有上述基线的独立克隆中先执行 `git bundle verify <bundle>`，再读取整合分支，不向 dev/main应用补丁。
