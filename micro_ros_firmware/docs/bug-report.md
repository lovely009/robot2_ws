# micro_ros_firmware 问题修复清单

> 生成日期：2026-08-31
> 
> 本文件列出固件工程中需要修复的问题，按优先级排序。修复前请先阅读各包 `README.md` 和 `AGENTS.md`。

---

## 🔴 P0：严重 Bug（必须修复）

### 1. `motor_control_reconfigure` 悬空指针 Bug

**文件**：`lib/motor_control/src/motor_control.c`

**问题描述**：

`motor_control_reconfigure()` 函数在重配置电机表时，没有将 `ctx->motors[i].joint_name` 指针重定向到内部缓冲区 `ctx->joint_names[i]`。

对比 `motor_control_init()` 的正确实现：

```c
// ✅ motor_control_init（第 443-455 行）
memcpy(ctx->joint_names[index], joint_name, name_size);
ctx->motors[index].joint_name = ctx->joint_names[index];  // ← 关键：指针重定向
```

而 `motor_control_reconfigure()`（第 593-603 行）：

```c
// ❌ motor_control_reconfigure（缺失指针重定向）
memcpy(ctx->joint_names[index], motors[index].joint_name, name_len);
ctx->joint_names[index][name_len] = '\0';
// 缺失：ctx->motors[index].joint_name = ctx->joint_names[index];
```

**后果**：

1. `ctx->motors[i].joint_name` 仍然指向调用方传入的 `g_pending_motors[]` 静态缓冲区
2. 当参数服务器收到新配置时，`g_pending_motors[]` 被覆盖
3. `motor_control_accept_command()` 中的 `strcmp()` 读取已失效内存
4. 可能导致：命令匹配失败、内存越界、HardFault

**修复方案**：

在 `motor_control_reconfigure()` 的第 5 行循环中，添加指针重定向：

```c
// 5. 复制关节名到内部缓冲区并重定向指针
for (size_t index = 0; index < motor_count; ++index) {
    size_t name_len = strlen(motors[index].joint_name);
    if (name_len >= MOTOR_CONTROL_JOINT_NAME_CAPACITY) {
      name_len = MOTOR_CONTROL_JOINT_NAME_CAPACITY - 1U;
    }
    memcpy(ctx->joint_names[index], motors[index].joint_name, name_len);
    ctx->joint_names[index][name_len] = '\0';
    ctx->motors[index].joint_name = ctx->joint_names[index];  // ← 添加此行
    ctx->runtime[index].state = MOTOR_CONTROL_DISABLED;
}
```

**验证**：已修复（2026-08-31）。确认 `motor_control_init` 和 `motor_control_reconfigure` 行为一致。

---

## 🟡 P1：中等问题（建议修复）

### 2. FDCAN 扩展 ID 过滤器缺失

**文件**：`lib/fdcan/src/can_bsp.c`

**问题描述**：

`can_filter_init()` 中只配置了标准 ID 过滤器（全收），但全局过滤器将非匹配的扩展帧配置为拒绝：

```c
// 第 187-197 行
filter.IdType = FDCAN_STANDARD_ID;  // 只配标准 ID
filter.FilterID1 = 0x00;
filter.FilterID2 = 0x00;  // mask=0，全收

HAL_FDCAN_ConfigGlobalFilter(kCanHandles[i], 
    FDCAN_REJECT,      // 非匹配标准帧：拒绝
    FDCAN_REJECT,      // 非匹配扩展帧：拒绝 ← 问题
    FDCAN_REJECT_REMOTE,
    FDCAN_REJECT_REMOTE);
```

**影响**：

- 标准 ID 帧（11位）：通过标准过滤器，正常接收
- **扩展 ID 帧（29位）**：无对应过滤器 → 全局过滤器拒绝
- 如果 RP/vaxor 协议使用扩展 ID，FDCAN2/3 会收不到

**建议**：

1. 确认 RP/vaxor 协议是否使用扩展 ID
2. 如需支持，添加扩展 ID 过滤器（FilterIndex 1-3）
3. 或将全局过滤器改为 `FDCAN_ACCEPT` 以接收所有帧

---

### 3. `motor_control_motor_config_t.joint_name` 指针语义

**文件**：`src/motor_bridge.cpp` + `lib/motor_control/include/motor_control.h`

**问题描述**：

`motor_control_motor_config_t.joint_name` 声明为 `const char *`（指针），但实际使用中：

```c
// motor_bridge.cpp 第 320-324 行
motor_control_motor_config_t *motor = &g_pending_motors[index];
// 直接 memcpy 写入指针值
memcpy((void *)motor->joint_name, (const void *)value, len);
```

这里 `memcpy` 实际上是拷贝了字符串首地址（指针值），而非字符串内容。由于 `g_pending_motors` 是静态存储区，这在当前实现中是安全的，但语义上容易误解。

**建议**：

1. 添加注释说明：`joint_name` 是指针，必须指向有效且持久的存储
2. 或考虑将字段改为 `char joint_name[MOTOR_CONTROL_JOINT_NAME_CAPACITY]`（值类型）
3. 注意：修改字段类型会影响 `motor_control_init` 和 `motor_control_reconfigure` 的实现

---

### 4. 缺少 FreeRTOS 栈溢出检测

**文件**：`platformio.ini`

**问题描述**：

当前未启用 FreeRTOS 的栈溢出检测机制，在栈溢出时无法得到及时警告。

**建议**：

在 `platformio.ini` 的 `build_flags` 中添加：

```ini
build_flags =
    ...
    -DconfigCHECK_FOR_STACK_OVERFLOW=2
```

并实现钩子函数（可放在 `src/main.cpp`）：

```c
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    ws2812_set_rgb(1, 0, 0);  // 红灯常亮
    while (1) {}
}
```

---

## 🟢 P2：改进建议（可选优化）

### 5. `Error_Handler` 无恢复机制

**位置**：多处调用（`main.cpp`、`system_clock.cpp`、`fdcan.c`、`can_bsp.c`）

**现状**：`Error_Handler()` 进入死循环，无错误码记录。

**建议**：

- 在 RAM 中记录错误码和调用位置
- 闪烁特定 LED 模式指示错误类型
- 生产固件考虑 IWDG 超时复位

---

### 6. `control_task_reconfigure` 的 500ms 固定延时

**文件**：`src/control_task.cpp` 第 370 行

**现状**：

```c
vTaskDelay(pdMS_TO_TICKS(500));  // 等待电机停止
```

**建议**：

- 添加电机实际停止的反馈检测（如位置/速度阈值）
- 或由上层 ROS 节点控制：先发送停止命令 → 等待反馈确认 → 再下发重配置

---

### 7. `motor_control_reconfigure` 缺少计数器重置

**文件**：`lib/motor_control/src/motor_control.c` 第 608-611 行

**现状**：

```c
// 6. 重置统计计数
ctx->valid_command_count = 0U;
ctx->invalid_command_count = 0U;
ctx->valid_feedback_count = 0U;
ctx->invalid_feedback_count = 0U;
```

**建议**：考虑是否需要保留历史统计（便于调试），或添加一个 `reconfigure_count` 计数器记录重配置次数。

---

## 📋 修复检查清单

修复完成后，请逐项验证：

- [ ] `motor_control_reconfigure` 中 `ctx->motors[i].joint_name` 指向 `ctx->joint_names[i]`
- [ ] `motor_control_init` 和 `motor_control_reconfigure` 行为一致
- [ ] 参数服务器动态配置后，电机命令能正确匹配
- [ ] FDCAN 扩展 ID 帧（如需要）能正常收发
- [ ] FreeRTOS 栈溢出检测已启用并测试
- [ ] 所有测试通过：`colcon test --packages-select micro_ros_firmware`

---

## 🔗 参考文档

- `micro_ros_firmware/README.md` — 权威文档
- `micro_ros_firmware/AGENTS.md` — Agent 即用指南
- `docs/adr/` — 架构决策记录
