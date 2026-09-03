# 固件入口与任务模块化设计

## 目标

将 `micro_ros_firmware/src/main.cpp` 收缩为清晰的 FreeRTOS 任务编排入口，
并将板级初始化、micro-ROS 连接状态机和 LED 显示职责拆分到独立模块。
代码布局参考项目现有 FreeRTOS 示例风格，但保留 STM32duino 必需的
`setup()`/`loop()` 入口，不改为裸机 `int main()`。

## 模块与职责

### `main.cpp`

`main.cpp` 只保留：

- start task 的优先级、栈大小和任务句柄；
- `start_task()` 任务编排；
- Arduino `setup()` 和非阻塞 `loop()`。

`setup()` 调用 `initialize_system()`，创建 `start_task`，然后启动 FreeRTOS
调度器。`start_task()` 依次启动 LED、micro-ROS、sensor 和 control 任务，
全部成功后删除自身。创建过程不包裹长期临界区，避免自删除时遗留
临界区嵌套状态。

### `system.h` / `system.cpp`

`system.h` 是系统级接口，统一包含 Arduino 与 FreeRTOS 的公共头文件，
并声明 `initialize_system()`。它不包含电机协议、控制器或业务 bridge
等头文件。每个实现文件仍显式包含自己直接使用的模块接口。

`system.cpp` 负责：

- 提供 FreeRTOS idle/timer 静态任务内存回调；
- 初始化 WS2812 驱动；
- 初始化 USB CDC `Serial` 并设置 micro-ROS serial transport；
- 等待 USB 枚举稳定。

实现文件使用 `.cpp`，因为 STM32duino `Serial` 与
`set_microros_serial_transports()` 依赖 C++ 接口。

### `micro_ros_task.h` / `micro_ros_task.cpp`

该模块封装 agent 探测、ROS 实体创建、executor 服务和断连清理状态机。
外部接口为：

- `bool micro_ros_task_start(void)`；
- `micro_ros_link_state_t micro_ros_task_get_link_state(void)`；
- `bool micro_ros_task_ever_connected(void)`。

任务句柄、栈大小、优先级和状态机数据属于模块实现，不对外暴露。
状态读写使用短临界区，使 LED 任务可以得到一致快照。

### `led_task.h` / `led_task.cpp`

WS2812 由 LED 任务单一写入，任务每 50 ms 读取 micro-ROS 连接快照：

- 首次连接前的 `WAIT_AGENT`：红灯；
- `CONNECT`：保持上一颜色；
- `RUN`：绿灯 500 ms 亮、500 ms 灭；
- `LOST` 或曾连接后回到 `WAIT_AGENT`：蓝灯。

只在目标颜色改变时写 WS2812，以减少时序敏感的单线总线更新。

## 任务创建与错误处理

`start_task()` 只调用各模块的 `*_task_start()` 接口。sensor 和 control
模块继续在内部完成信号量、mailbox 和状态机初始化，不向入口暴露
裸任务函数。任一启动接口失败时调用 `Error_Handler()`，不在部分任务已运行时
继续启动系统。

## W25Q64 范围

删除入口中的 W25Q64 启动探测、`g_w25q64_init_status`、
`g_w25q64_jedec_id` 和 `ENABLE_W25Q64` 构建开关。`lib/w25q64` 驱动和对应
纯逻辑测试保留，后续若使用 Flash，由明确的专用任务或模块管理其生命周期。

## 依赖规则

- `src/` 实现文件显式 `#include` 其直接使用的业务头文件；
- `system.h` 只聚合系统级公共头文件，不作为业务头文件大全；
- `lib/A` 使用 `lib/B` 时，在 A 的 `library.json` 声明 B；
- PlatformIO 对项目 `src/` 的库发现不替代编译器所需的头文件包含。

## 测试与验收

- 为连接状态到 LED 颜色的映射提供 host 可测的纯逻辑，覆盖红、绿闪、
  蓝和 `CONNECT` 保持颜色；
- 运行现有 host CTest，已知 `motor_control` 在
  `test_motor_control.cpp:191` 的既存失败单独记录；
- 检查 `main.cpp` 不再包含 LED 实现、micro-ROS 状态机或 W25Q64 探测；
- 执行 PlatformIO clean build，确认目标板编译、链接和内存统计成功；
- 同步 README、AGENTS 与代码结构说明。

## 不在范围内

- 不改变 ROS 话题、节点名或 executor 实体；
- 不改变 sensor/control 的周期、优先级或内部初始化；
- 不恢复已删除的 supervisor 子系统；
- 不将已弃用的 RP 协议重新接入默认电机表。
