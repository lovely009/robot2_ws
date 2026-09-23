# session_sync

## 审查结论

**符合纯逻辑边界，host 覆盖有效。** 该库只把 mailbox 的会话边沿/电平按固定顺序转发给 `motor_control`，不创建任务、不持有锁、不访问 CAN。

## 职责与接口

唯一公共入口是：

```c
void session_sync_apply(motor_control_context_t *ctx,
                        const control_mailbox_events_t *events,
                        bool session_authorized, uint32_t now_ms);
```

处理顺序固定为：先转发 `agent_state_pending` 连接边沿，再按会话授权电平调用内核。这样断连时能触发内核的 P0 撤权路径；每个控制边界都可重放当前电平。

## 依赖与调用契约

- 依赖 `control_mailbox` 的事件类型和 `motor_control` 的状态接口。
- 依赖方向是 `motor_rt_task -> session_sync -> (control_mailbox, motor_control)`，没有反向 include。
- `take_events()` 与授权电平读取必须由调用方放在同一临界区；本库不提供锁。
- `ctx` 或 `events` 为 NULL 时安全返回，但调用方仍应保证正常路径数据有效。

## 验证

`test_session_sync.cpp` 通过真实 mailbox、内核和 adapter 对象验证连接、断连、授权和事件顺序；PlatformIO 固件构建通过。`library.json` 已显式列出 `control_mailbox` 和 `motor_control` 依赖。
