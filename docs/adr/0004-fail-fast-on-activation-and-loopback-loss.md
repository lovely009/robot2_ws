# 激活失败与回环终止采用 fail-fast 语义

**Status**: accepted

驱动侧硬件激活失败或运行期反馈超时会让 `controller_manager` 进程立即终止（Humble
`resource_manager` 的默认行为）。这是预期的安全语义：电机不可达就不能运行，人工
诊断后重新 bring-up，不做自动重试或重启。仿真回环的启动竞态由 launch 时序消除
（模拟器就绪后才拉起驱动侧）；模拟器退出时 `driver_loopback` 整个终止并给出可
操作提示，避免 spawner 无限重试。

## Considered Options

- **自动重试/重启（模拟器 respawn、激活有界重试）**：会掩盖“电机真的连不上”或
  “vcan 未创建”这类根因。安全系统失败应快而可预测，故否决。
- **fail-fast（已选）**：失败立即暴露，重跑前先诊断；回环额外把“模拟器退出”
  转化为整个 launch 的终止。

## Consequences

- 真机：激活失败/反馈超时 = 安全停机信号，人工排查总线、电源、标定后重跑。
- 回环：vcan 未创建等启动期错误会在模拟器退出时终止 launch 并打印提示；
  运行中模拟器掉线同样终止回环，不会静默挂死。
