# motor_control

## 审查结论

**符合核心库定义；型号目录和实时安全语义已落在本库，仍有明确 TBD 项。** 它是纯 C、ID 驱动的运行时内核，不依赖 HAL、FreeRTOS、ROS 或 `control_mailbox`。

## 职责与边界

- 固定容量的动态电机注册表和事件环。
- Profile 目录、协议绑定、生命周期（ABSENT 到 ACTIVE/FAULT/OFFLINE 等）和配置事务。
- 完整快照、轨迹、策略、freshness、软停/失能和 P0 重试。
- 只通过 `motor_protocol_adapter_t` 使用厂家协议；不保存 `joint_name`，不拼厂家字节。

## 文件与接口

| 文件 | 作用 |
|---|---|
| `include/motor_control.h` | 不透明上下文、生命周期、实时路径、快照/状态/事件 API |
| `include/motor_control_profile.h` | 编译期型号目录查询 |
| `src/motor_control.c` | 主状态机、快照、轨迹、策略和输出 |
| `src/motor_control_discovery.c` | 反馈发现、建档和重连 |
| `src/motor_control_config.c` | 配置事务和设备参数回读 |
| `src/motor_control_params.c` | 参数影子、限值和持久化视图 |
| `src/motor_control_profile.c` | Profile 表实现 |
| `src/motor_control_internal.h` | 仅库内实现使用的布局和辅助接口 |

关键入口是 `motor_control_init()`、`motor_control_on_raw_frame()`、`motor_control_service()`、`motor_control_apply_command_snapshot()`、`motor_control_apply_trajectory()`、`motor_control_apply_policy()`、`motor_control_collect_tx()` 和各类只读快照复制函数。

## 实时与依赖

- `motor_rt_task` 是唯一调用者；1ms 服务和 2ms/500Hz 控制边界都由调用方驱动。
- 所有容量在编译期固定；不等待设备、不分配动态内存、不读取 ROS 时间。
- 唯一库依赖是 `motor_protocol` 的 adapter 接口；`library.json` 已声明该依赖。

## 验证与缺口

- `test_motor_control.cpp` 覆盖注册、生命周期、授权、freshness、快照、配置和事件。
- `test_session_sync.cpp` 通过真实内核验证会话同步。
- `test_motor_protocol_adapter.cpp` 覆盖 Profile/adapter 交互。
- PlatformIO 固件构建通过。

当前 `Vaxor 80` 不进入目录是有意安全策略；`TBD-PROFILE-01`、`TBD-CONFIG-01` 和部分安全语义仍未冻结。它们属于产品/协议待决项，不是库边界违规。修改容量宏时必须重新做链接 RAM 和实时性验证。
