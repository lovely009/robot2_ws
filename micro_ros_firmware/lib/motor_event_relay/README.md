# motor_event_relay

内核事件中继环（审查 H3 抽取）：`motor_rt_task`（生产侧，1ms 服务节拍）把
`motor_control` 内核事件环的条目搬入本环，ROS 桥任务（消费侧，轮询）经
`motor_rt_drain_events` 取走上报。纯 C、静态内存、无动态分配；只依赖
`lib/motor_control` 的事件类型 `motor_control_event_t`（无 ROS/HAL/RTOS 依赖）。

## API

| 函数 | 语义 |
|---|---|
| `motor_event_relay_init` | 复位为空环（仅初始化期） |
| `motor_event_relay_push` | 入队一条事件；**满则丢最旧保最新**（覆盖 head 槽后前移）并累计 `dropped_count` |
| `motor_event_relay_drain` | FIFO 取走至多 capacity 条，返回实际条数 |
| `motor_event_relay_dropped_count` | 生命周期累计丢弃计数（诊断可见） |

容量 `MOTOR_EVENT_RELAY_CAPACITY`(64)，编译期固定（ARD §29）。

## 并发契约

与 `control_mailbox` 同款：库内无锁、无原子、无 volatile——每次调用都必须
位于调用方临界区内，且**生产/消费两侧共用同一互斥域**（单核 FreeRTOS
`taskENTER_CRITICAL` 即满足）；「单侧持锁」不构成互斥。仅支持任务上下文。

现有点位：生产侧 `forward_kernel_events`（motor_rt 任务，逐条短临界区）；
消费侧 `motor_rt_drain_events` / `motor_rt_event_dropped`（micro_ros 桥任务，
逐调用临界区）。

## 设计约束：环形序永不破坏

旧实现满队时 memmove 平移数组，隐含 head==0 假设——head≠0 时丢错元素且
破坏 FIFO 序（审查 H3）。本模块满队只做「覆盖 head 槽 + head 前移」，任何
head 状态下序都保持严格 FIFO；禁止重新引入数组平移。

## 构建与测试

固件侧：PlatformIO LDF 经 `motor_rt_task.cpp` 的 include 链接入
（library.json 显式声明依赖 `motor_control`）。

host 单测：`test/host/test_motor_event_relay.cpp`（CTest 目标
`motor_event_relay`，C17 + cassert）：

```bash
cmake -S test/host -B .host-build && cmake --build .host-build --clean-first -j2
ctest --test-dir .host-build --output-on-failure -R motor_event_relay
```

已覆盖：FIFO 序 + 部分取走、满丢最旧保最新（CAP+3 → dropped==3）、
head≠0 起点跨回绕填满再溢出、NULL/零容量防御。
