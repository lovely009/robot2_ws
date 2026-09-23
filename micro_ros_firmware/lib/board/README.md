# board

## 审查结论

**符合。** 这是一个有意的纯头文件板级约束库，不需要 `src/`。PlatformIO 已在固件依赖图中发现并参与构建；`library.json` 显式声明了 `include/` 目录。它不应放设备驱动、任务或 FreeRTOS 头。

## 职责与边界

- `include/board_irq.h` 提供 `BOARD_ISR_SYSCALL_THRESHOLD`，作为板级 ISR 优先级的唯一共享常量。
- `lib/dma`、`lib/fdcan` 等板级库引用它做编译期约束。
- 不拥有中断处理器、外设句柄、任务或业务状态。
- 不反向依赖 `src/`、HAL、FreeRTOS 或 ROS。

## 文件与接口

| 文件 | 作用 |
|---|---|
| `library.json` | PlatformIO 库元数据 |
| `include/board_irq.h` | ISR syscall 阈值常量和板级约束注释 |

公共接口只有一个宏：`BOARD_ISR_SYSCALL_THRESHOLD = 5U`。它必须与 FreeRTOS 的 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 保持一致；`main.cpp` 负责跨侧反向断言。

## 依赖与验证

该库没有 host 单测目标，因为没有可执行逻辑。验证方式是：

```bash
cd micro_ros_firmware
~/.platformio/penv/bin/pio run
```

当前固件构建成功，说明头文件被正确解析。新增板级常量时，应同时检查 `main.cpp` 的反向断言和所有使用者。

## 后续建议

保持 header-only 形态；若将来出现函数或状态，不要把实现塞进本库，应先重新划分所有权并补 `src/` 与测试。
