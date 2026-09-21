# 模块边界与并发所有权

**状态：代码结构基线（2026-09-22）。**

本文把“哪个上下文可以调用什么、谁拥有哪块数据、哪些依赖被禁止”集中记录，
用于审查新代码和决定是否值得物理迁移目录。它不改变公开 API、任务周期、协议
语义或硬件初始化顺序；实现事实以源码为准，验证状态以
../plans/active/motor-runtime-validation.md 为准。

## 1. 当前分层

~~~text
板级/硬件层
  lib/board       共享板级约束（不持有业务状态）
  lib/fdcan       HAL、IRQ、RX ring、TX 调度
  lib/dma         DMA 控制器级资源登记
  lib/imu         BMI088/SPI2 与 IMU 域状态
  lib/w25q64      W25Q64 原始读写与地址几何
  lib/ws2812      SPI6/BDMA LED 物理发送

纯逻辑与协议层
  lib/motor_protocol     厂家字节布局、CAN ID、编解码、事务描述
  lib/motor_control      注册表、Profile、生命周期、安全与轨迹执行
  lib/control_mailbox    固定容量的命令/策略/轨迹/配置交换
  lib/session_sync       会话/授权状态的纯派生
  lib/storage_projection 参数视图与 Flash blob 的纯投影
  lib/param_store        blob 布局、CRC、版本迁移和参数单位

应用与任务层
  src/motor_rt_task.cpp  唯一电机实时所有者
  src/motor_bridge.cpp   ROS 电机话题 ↔ mailbox/只读状态副本
  src/param_bridge.cpp   参数话题/系统命令 ↔ mailbox/storage 请求
  src/storage_task.cpp   W25Q64 唯一所有者
  src/ros_bridge.cpp      ROS 实体生命周期与域桥编排
  src/imu_bridge.cpp      IMU/温度发布
  src/sensor_task.cpp    BMI088/加热任务
  src/led_task.cpp       链路状态到 WS2812
~~~

依赖方向应保持为“应用调用库、协议调用纯数据接口、硬件库不反向调用任务”：

~~~text
src/ros_bridge ──> src/imu_bridge
                ├─> src/motor_bridge ──> src/motor_rt_task
                └─> src/param_bridge ──> src/motor_rt_task + src/storage_task
                                                   │
                                                   ├── control_mailbox
                                                   ├── motor_control ──> motor_protocol
                                                   ├── session_sync
                                                   └── fdcan
       │
src/storage_task ──> storage_projection ──> param_store
       └───────────> w25q64
src/sensor_task ──> imu
src/led_task ────> ws2812
~~~

control_mailbox、motor_control、motor_protocol、session_sync、
storage_projection 和 param_store 不依赖 ROS、HAL 或 FreeRTOS。它们可以被
主机测试直接编译；调用方提供并发保护，不在库内创建任务、锁或动态内存。
lib/fdcan、lib/imu、lib/w25q64、lib/ws2812 是硬件边界；纯逻辑库不能反向依赖
这些硬件库，应用任务只能按本页的所有权入口调用它们。

## 2. 上下文、所有权和允许操作

| 上下文 | 唯一职责/所有权 | 允许的副作用 | 明确禁止 |
|---|---|---|---|
| FDCAN ISR：HAL_FDCAN_RxFifo0Callback | 从硬件 FIFO 取原始帧，写对应 SPSC RX ring；记录硬件丢帧 | 固定容量拷贝、单调时戳、计数 | 解码、ROS、普通 TX、等待、阻塞、任务 API |
| motor_rt_task（优先级 3） | RX ring 唯一消费者；motor_control 唯一运行时调用者；正常 CAN TX 唯一所有者 | 每 1ms 有界服务；每 5ms/200Hz 消费 mailbox、轨迹和配置；独立喂约 500ms MCU IWDG | 等待 Agent/设备/W25；动态分配；把原始 ring 暴露给 ROS |
| micro_ros_task/ROS bridge（优先级 2） | ROS 实体生命周期；写 mailbox；读取状态副本和事件 | 消息校验、短临界区、发布状态 | 直接读 RX ring、直接调用 HAL/CAN 发送、修改内核内部状态 |
| storage_task（优先级 1） | W25Q64、双槽 blob、启动参数加载和保存结果 | 允许在本任务阻塞擦写；使用任务通知传递 request_id | 在 motor_rt 或 ROS 上下文访问 W25；改变运动授权或控制周期 |
| sensor_task（优先级 2） | BMI088 SPI2、加热和 IMU snapshot | 设备库约束内的阻塞 SPI；失败降级和重试 | 触碰电机运行时所有权或 ROS 实体 |
| led_task（优先级 1） | WS2812 运行期唯一写入者 | 读取链路状态，提交固定 LED 帧 | 参与安全停机或 FDCAN 调度 |
| 启动上下文 | 初始化板级资源、加载 boot 参数、创建任务 | 只执行一次的硬件/任务初始化 | 在调度器启动前调用需要任务延时的 IMU 路径 |

### 2.1 关键共享数据

- FDCAN RX ring：生产者是对应 FDCAN ISR，消费者只有
  motor_rt_task::drain_rx_rings()；环满丢新帧。任何调试或 ROS 需求只能读取
  解码后的状态/诊断副本。
- mailbox：ROS 侧通过 motor_rt_submit_* 写入，motor_rt_task 在边界取走；
  mailbox 本身无锁，双方必须使用同一短临界区。策略和轨迹也遵循同一所有权边界。
- 内核上下文：只由 motor_rt_task 写；ROS 只通过 motor_rt_copy_*() 和
  motor_rt_drain_events() 取得副本。
- W25 缓冲：storage_task 独占；boot 加载与保存复用 scratch 的前提是两条路径在
  任务创建前后不重叠。保存结果与 request_id 在同一短临界区内成对取走。
- 参数投影：storage_projection 只转换值，不拥有 Flash、不触发保存；设备运行时参数
  快照由 motor_rt 提供给 storage 任务。

## 3. 接口和头文件规则

1. include/ 只放应用任务和 ROS bridge 的稳定入口；硬件/协议库公共头文件放在
   lib/<name>/include/。
2. lib/<name>/src/ 内部头文件只供本库实现使用；不得从 src/ 或其他库直接
   include motor_control_internal.h 等布局头。主机白盒测试如需访问内部布局，
   必须在 test/host/CMakeLists.txt 中显式标注该测试边界。
3. 新增厂家字节、CAN ID、DLC/FD 属性只能进入 lib/motor_protocol；新 ROS
   消息校验只能进入 src/*_bridge；新安全/生命周期裁决只能进入 lib/motor_control。
4. src/motor_rt_task.cpp 是唯一允许把内核待发帧交给 can_bsp_submit_tx() 的应用文件。
   ROS bridge 不得绕过 mailbox，storage task 不得绕过任务通知直接操作 W25。
5. 不因为“目录看起来整齐”移动稳定的硬件相关源文件。只有当所有权、公共头和
   PlatformIO/host 构建边界可以同时保持时，才单独提出迁移切片。

## 4. 测试边界

| 层 | 无硬件可验证内容 | 仍需台架/HIL |
|---|---|---|
| motor_protocol/motor_control | 编解码、事务描述、生命周期、安全级联、轨迹拼接/非均匀 Hermite | 真实设备应答、软停观感、协议耐久语义 |
| control_mailbox/session_sync | 双缓冲、FIFO、分段去重/冲突、会话边沿和授权派生 | 中断与任务真实抢占下的延迟分布 |
| storage_projection/param_store | 投影、CRC、版本迁移、槽位校验 | 掉电、真实擦写时长、跨介质非原子性 |
| fdcan/任务编排 | 编译、静态检查、容量契约 | ISR 延迟、RX overflow、TX 仲裁、BUS_OFF |
| ROS/上层转发 | 消息生成、分段序号和绝对时间线 | rosbridge/Agent/MCU 顺序、USB CDC 余量和真机连续性 |

当前“编译通过”只证明构建边界完整；它不替代 200Hz 控制帧、约 500ms
驱动器看门狗、Flash 掉电和硬件急停验证。

## 5. 结构变更准则

本轮不做物理目录迁移，原因是现有所有权已经由 motor_rt_task、storage_task
和各纯逻辑库的接口表达，且硬件初始化顺序属于不可变约束。后续改动按以下顺序
提出：

1. 先在纯逻辑库增加可测试的契约或适配器；
2. 再由一个任务层入口接入，不增加第二个所有者；
3. 先跑对应 host target，再跑最窄 PlatformIO/ROS 构建；
4. 最后补充本页的上下文、资源和验证记录；
5. 若需要改变公开头、消息、时序、所有权或 Flash 语义，必须另建 ADR，
   不能以“结构整理”名义隐含改变行为。
