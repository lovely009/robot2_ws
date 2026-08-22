# Firmware Control Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将固件的 ROS 电机命令、100 Hz 控制执行、CAN 收发和故障恢复改造成确定、可测试、可观察的开发阶段安全闭环。

**Architecture:** 新增不依赖 RTOS/HAL/rclc 的 `motor_control` 深模块，ROS callback 只投递每关节 mailbox，`control` task 成为唯一正常控制发送方和 CAN RX 消费者。FDCAN ISR 写静态有界队列，IMU 使用一致快照，低优先级 supervisor 统一记录任务健康状态。

**Tech Stack:** C17/C++17、STM32duino、STM32FreeRTOS、STM32 HAL FDCAN、micro-ROS Humble、PlatformIO、CMake/CTest host tests。

**Spec:** `docs/superpowers/specs/2026-08-22-firmware-control-safety-design.md`

## Global Constraints

- 目标 MCU 为 STM32H723VGT6；当前 SYSCLK 480 MHz、RAM 320 KB、Flash 1 MB。
- 正常控制路径为 MIT，命令周期 10 ms；100 ms soft-stop，300 ms hard-stop。
- soft-stop 固定为 `kp=0`、保留电机默认 `kd`、velocity=0、feedforward=0。
- position-only/velocity-only 调试模式保持兼容；不增加机械限位假设。
- RP 64 B 命令按总线每周期聚合一次，各 slot 独立；单电机故障不得停止其他 slot。
- Agent 断连、bus-off 和 manual-disable 是总线级严重故障；恢复不得重放旧命令。
- 所有新增控制状态、mailbox 和队列使用静态有界存储，不在周期路径动态分配。
- 开发阶段不启用 IWDG，不新增 diagnostics topic，不向 USB CDC 混入文本日志。
- `dev` 上完成代码和自动测试；无负载真机验证留给 `dev-real`。
- 每个任务开始前重读固件 `README.md`、`AGENTS.md` 和本计划引用的接口。

---

## File Structure

### 新建文件

- `micro_ros_firmware/lib/motor_control/include/motor_control.h`：纯逻辑控制接口和固定容量类型。
- `micro_ros_firmware/lib/motor_control/src/motor_control.c`：命令校验、状态机、RP 聚合、反馈状态和诊断实现。
- `micro_ros_firmware/lib/motor_control/library.json`：PlatformIO 库元数据，依赖 `motor_protocol`。
- `micro_ros_firmware/include/supervisor_task.h`：任务健康位和 heartbeat 接口。
- `micro_ros_firmware/src/supervisor_task.cpp`：100 ms software supervisor。
- `micro_ros_firmware/test/host/CMakeLists.txt`：host 测试构建入口。
- `micro_ros_firmware/test/host/test_motor_protocol.cpp`：现有协议回归测试。
- `micro_ros_firmware/test/host/test_motor_control.cpp`：状态机、校验、恢复和 RP 聚合测试。
- `micro_ros_firmware/test/host/test_can_rx_ring.cpp`：队列溢出策略的纯逻辑测试替身。
- `micro_ros_firmware/test/host/test_imu_snapshot.cpp`：双缓冲一致性测试。

### 修改文件

- `micro_ros_firmware/include/control_task.h`、`src/control_task.cpp`：mailbox、100 Hz循环、唯一发送方。
- `micro_ros_firmware/include/motor_bridge.h`、`src/motor_bridge.cpp`：ROS Adapter和`motor_enable`订阅。
- `micro_ros_firmware/include/ros_bridge.h`、`src/ros_bridge.cpp`：executor容量和通信heartbeat。
- `micro_ros_firmware/lib/fdcan/include/can_bsp.h`、`src/can_bsp.c`：静态RX队列和诊断。
- `micro_ros_firmware/lib/fdcan/src/fdcan.c`：FreeRTOS安全的IRQ priority。
- `micro_ros_firmware/lib/imu/include/imu.h`、`src/imu.c`：单次初始化尝试接口。
- `micro_ros_firmware/include/sensor_task.h`、`src/sensor_task.cpp`：重试、heartbeat、一致快照。
- `micro_ros_firmware/include/imu_bridge.h`、`src/imu_bridge.cpp`：消费快照而非共享指针。
- `micro_ros_firmware/src/main.cpp`：创建supervisor并上报micro-ROS状态推进。
- `micro_ros_firmware/README.md`、`AGENTS.md`：任务、timeout、故障语义和验证命令。

---

### Task 1: 建立 Host 测试入口并固定协议行为

**Files:**
- Create: `micro_ros_firmware/test/host/CMakeLists.txt`
- Create: `micro_ros_firmware/test/host/test_motor_protocol.cpp`
- Modify: `micro_ros_firmware/test/README`
- Test: `micro_ros_firmware/test/host/test_motor_protocol.cpp`

**Interfaces:**
- Consumes: `motor_protocol_pack_rp_slot()`、`motor_protocol_unpack_rp_state()`、vaxor pack/unpack接口。
- Produces: `cmake -S test/host -B .host-build && cmake --build .host-build && ctest --test-dir .host-build --output-on-failure`。

- [ ] **Step 1: 写现有协议的host回归测试**

使用标准`assert`和单个`main()`，至少固定RP零命令、RP极值限幅、vaxor 12 B命令round-trip和非法长度拒绝：

```cpp
#include <cassert>
#include <cmath>
#include "motor_protocol.h"

static void test_rp_zero_round_trip() {
  motor_protocol_joint_command_t input{};
  uint8_t bytes[8]{};
  motor_protocol_pack_rp_slot(&input, bytes);
  motor_protocol_joint_command_t output{};
  assert(motor_protocol_unpack_rp_slot(bytes, sizeof(bytes), &output));
  assert(std::abs(output.position) < 0.01);
}

int main() {
  test_rp_zero_round_trip();
  return 0;
}
```

- [ ] **Step 2: 写CMake/CTest入口**

```cmake
cmake_minimum_required(VERSION 3.16)
project(firmware_host_tests LANGUAGES C CXX)
enable_testing()
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
add_executable(test_motor_protocol
  test_motor_protocol.cpp
  ../../lib/motor_protocol/src/motor_protocol.c)
target_include_directories(test_motor_protocol PRIVATE
  ../../lib/motor_protocol/include)
target_link_libraries(test_motor_protocol PRIVATE m)
add_test(NAME motor_protocol COMMAND test_motor_protocol)
```

- [ ] **Step 3: 运行基线测试**

Run:

```bash
cd micro_ros_firmware
cmake -S test/host -B .host-build
cmake --build .host-build
ctest --test-dir .host-build --output-on-failure
```

Expected: configure/build成功，`motor_protocol`测试PASS。

- [ ] **Step 4: 在`test/README`记录统一命令和`.host-build/`清理方式**

文档必须说明host测试不模拟HAL、FreeRTOS或rclc，且`.host-build/`不得提交。

- [ ] **Step 5: 提交协议测试基线**

```bash
git add micro_ros_firmware/test
git commit -m "test: 建立固件 host 协议测试基线"
```

---

### Task 2: Test-first实现纯逻辑`motor_control`

**Files:**
- Create: `micro_ros_firmware/lib/motor_control/include/motor_control.h`
- Create: `micro_ros_firmware/lib/motor_control/src/motor_control.c`
- Create: `micro_ros_firmware/lib/motor_control/library.json`
- Create: `micro_ros_firmware/test/host/test_motor_control.cpp`
- Modify: `micro_ros_firmware/test/host/CMakeLists.txt`
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: `motor_protocol_joint_command_t`和`motor_protocol` pack/unpack函数。
- Produces:

```c
#define MOTOR_CONTROL_MAX_MOTORS 8U
#define MOTOR_CONTROL_MAX_TX_ACTIONS 16U

typedef enum { MOTOR_CONTROL_RP, MOTOR_CONTROL_VAXOR } motor_control_protocol_t;
typedef enum {
  MOTOR_CONTROL_DISABLED,
  MOTOR_CONTROL_ACTIVE,
  MOTOR_CONTROL_SOFT_STOP,
  MOTOR_CONTROL_FAULT_LATCHED
} motor_control_state_t;

typedef struct {
  const char *joint_name;
  motor_control_protocol_t protocol;
  uint8_t id;
  uint8_t bus_index;
  double gear_ratio;
  double gear_efficiency;
  double torque_constant_nm_per_amp;
  double default_kp;
  double default_kd;
} motor_control_motor_config_t;

typedef struct {
  char joint_name[32];
  uint8_t mode;
  double position;
  double velocity;
  double effort;
} motor_control_command_t;

typedef struct {
  uint8_t bus_index;
  uint32_t can_id;
  uint8_t size;
  bool is_fd;
  bool is_extended;
  uint8_t data[64];
} motor_control_tx_action_t;

typedef struct motor_control_context motor_control_context_t;

void motor_control_init(motor_control_context_t *ctx,
                        const motor_control_motor_config_t *motors,
                        size_t motor_count);
bool motor_control_accept_command(motor_control_context_t *ctx,
                                  const motor_control_command_t *command,
                                  uint32_t now_ms);
size_t motor_control_step(motor_control_context_t *ctx, uint32_t now_ms,
                          motor_control_tx_action_t *actions,
                          size_t action_capacity);
void motor_control_set_agent_connected(motor_control_context_t *ctx,
                                       bool connected);
void motor_control_set_bus_fault(motor_control_context_t *ctx,
                                 uint8_t bus_index, bool active);
void motor_control_set_motor_fault(motor_control_context_t *ctx,
                                   size_t motor_index, bool active);
bool motor_control_set_authorization(motor_control_context_t *ctx,
                                     bool authorized);
```

`motor_control_context_t`必须在header中以固定容量完整定义，使调用方可以静态分配；不得使用opaque heap对象。

- [ ] **Step 1: 写失败的timeout边界测试**

```cpp
static void test_timeout_boundaries() {
  motor_control_context_t ctx{};
  const motor_control_motor_config_t motor = {
      "joint", MOTOR_CONTROL_RP, 1, 0,
      1.0, 1.0, 0.0, 30.0, 1.0};
  const motor_control_command_t command = {
      "joint", MOTOR_PROTOCOL_WRIST_MODE_MIT,
      0.25, 0.0, 0.0};
  motor_control_tx_action_t actions[MOTOR_CONTROL_MAX_TX_ACTIONS]{};
  motor_control_init(&ctx, &motor, 1);
  assert(motor_control_accept_command(&ctx, &command, 0));
  motor_control_step(&ctx, 99, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_ACTIVE);
  motor_control_step(&ctx, 100, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_SOFT_STOP);
  motor_control_step(&ctx, 300, actions, 16);
  assert(ctx.runtime[0].state == MOTOR_CONTROL_DISABLED);
}
```

- [ ] **Step 2: 运行新测试并确认因接口不存在而失败**

Run: `cmake --build .host-build`

Expected: FAIL，编译器报告`motor_control`类型或函数未定义。

- [ ] **Step 3: 定义固定容量context、命令校验和状态转换的最小实现**

实现时使用无符号差值处理`millis()`回绕：

```c
static uint32_t elapsed_ms(uint32_t now, uint32_t then) {
  return now - then;
}

if (age >= 300U) {
  runtime->state = MOTOR_CONTROL_DISABLED;
} else if (age >= 100U) {
  runtime->state = MOTOR_CONTROL_SOFT_STOP;
} else {
  runtime->state = MOTOR_CONTROL_ACTIVE;
}
```

- [ ] **Step 4: 增加非法命令不续命测试并实现finite/name校验**

测试必须先接受0 ms有效命令，再在90 ms提交NaN，最后验证100 ms仍进入SOFT_STOP；使用`isfinite()`拒绝NaN/Inf并增加`invalid_command_count`。

- [ ] **Step 5: 增加严重故障、重新授权和旧命令失效测试**

覆盖Agent首次未连接不锁存、已连接后断开锁存、故障消失后单独授权不运行、授权后的新命令才能ACTIVE。

- [ ] **Step 6: 增加RP多slot聚合测试并实现每总线单帧输出**

配置同一bus的两个RP电机，分别提交不同position；断言`motor_control_step()`只产生一帧CAN ID `MOTOR_PROTOCOL_RP_COMMAND_ID`、size 64，解码slot 0/1后均保留各自命令。再锁存一个motor fault，断言只对应slot变neutral。

- [ ] **Step 7: 增加vaxor MIT及调试模式转换测试**

固定首次使能、模式切换必须先`run=false`再`run=true`，并验证position/velocity乘`gear_ratio`。

- [ ] **Step 8: 运行完整host测试**

Run: `cmake --build .host-build && ctest --test-dir .host-build --output-on-failure`

Expected: `motor_protocol`和`motor_control`全部PASS。

- [ ] **Step 9: 构建固件确认PlatformIO能发现新库**

Run: `~/.platformio/penv/bin/pio run`

Expected: SUCCESS；dependency graph包含`motor_control`和`motor_protocol`。

- [ ] **Step 10: 提交控制内核**

```bash
git add micro_ros_firmware/lib/motor_control micro_ros_firmware/test/host
git commit -m "feat: 增加可测试的电机控制状态机"
```

---

### Task 3: 将ROS命令改为Mailbox并建立100 Hz唯一发送路径

**Files:**
- Modify: `micro_ros_firmware/include/control_task.h`
- Modify: `micro_ros_firmware/src/control_task.cpp`
- Modify: `micro_ros_firmware/src/motor_bridge.cpp`
- Modify: `micro_ros_firmware/src/ros_bridge.cpp`
- Test: `micro_ros_firmware/test/host/test_motor_control.cpp`

**Interfaces:**
- Consumes: Task 2的`motor_control_*`接口。
- Produces:

```c
bool control_task_submit_command(const motor_control_command_t *command,
                                 uint32_t received_ms);
void control_task_set_authorization(bool authorized);
bool control_task_get_joint_state(size_t index,
                                  control_joint_state_t *out);
```

- [ ] **Step 1: 为部分JointState和无效字段补充纯逻辑测试**

验证两个关节中只更新一个时，另一个关节的时间戳不变；无效关节不影响同消息内有效关节。

- [ ] **Step 2: 把`control_task`默认电机表转换为`motor_control_motor_config_t`**

保留当前joint name、ID、bus、gear ratio、efficiency、torque constant、kp/kd数值，不扩展完整拓扑。

- [ ] **Step 3: 实现每关节静态overwrite mailbox**

每项包含`motor_control_command_t command`、`uint32_t received_ms`、`uint32_t sequence`和`bool pending`。`control_task_submit_command()`只在短critical section内覆盖对应项；找不到关节返回false。

- [ ] **Step 4: 将control loop改成绝对周期调度**

```cpp
TickType_t last_wake = xTaskGetTickCount();
for (;;) {
  drain_command_mailboxes();
  drain_can_rx_queues();
  run_motor_control_cycle();
  supervisor_heartbeat(SUPERVISOR_TASK_CONTROL);
  vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
}
```

- [ ] **Step 5: 从ROS callback移除所有FDCAN发送**

`joint_state_sub_cb()`只做数组边界、finite和name转换，然后调用`control_task_submit_command()`。用`rg -n "fdcan|HAL_FDCAN" src/motor_bridge.cpp`验证无命中。

- [ ] **Step 6: 新增`micro_ros/motor_enable` Bool订阅**

`false`调用`control_task_set_authorization(false)`；`true`请求清除已经消失的可恢复故障。将executor handle数量增加1，并为订阅消息使用静态存储。

- [ ] **Step 7: 将JointState反馈改为读取最新状态表**

ROS 20 ms timer只复制`control_task_get_joint_state()`返回的最新快照，不再直接调用`can1_rx_snapshot()`。

- [ ] **Step 8: 运行host测试和固件构建**

Run:

```bash
cmake --build .host-build
ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run
```

Expected: 全部PASS/SUCCESS，编译无新增warning。

- [ ] **Step 9: 提交命令路径重构**

```bash
git add micro_ros_firmware/include/control_task.h micro_ros_firmware/src/control_task.cpp micro_ros_firmware/src/motor_bridge.cpp micro_ros_firmware/src/ros_bridge.cpp
git commit -m "refactor: 由控制任务统一执行电机命令"
```

---

### Task 4: 用静态队列替换CAN最新帧快照

**Files:**
- Modify: `micro_ros_firmware/lib/fdcan/include/can_bsp.h`
- Modify: `micro_ros_firmware/lib/fdcan/src/can_bsp.c`
- Modify: `micro_ros_firmware/lib/fdcan/src/fdcan.c`
- Modify: `micro_ros_firmware/src/control_task.cpp`
- Modify: `micro_ros_firmware/src/can_debug_bridge.cpp`
- Create: `micro_ros_firmware/test/host/test_can_rx_ring.cpp`
- Modify: `micro_ros_firmware/test/host/CMakeLists.txt`

**Interfaces:**
- Consumes: `can_rx_frame_t`和control task唯一消费模型。
- Produces:

```c
typedef struct {
  uint32_t rx_count;
  uint32_t overflow_count;
  uint32_t hal_receive_error_count;
} can_rx_diagnostics_t;

bool can_rx_queue_pop(uint8_t bus_index, can_rx_frame_t *out);
void can_rx_get_diagnostics(uint8_t bus_index, can_rx_diagnostics_t *out);
```

- [ ] **Step 1: 写队列策略测试**

用host侧固定ring替身注入容量+1帧，断言弹出序列从第2帧开始，`overflow_count==1`；分别覆盖容量32和8。

- [ ] **Step 2: 运行测试确认新队列接口尚不存在**

Run: `cmake --build .host-build`

Expected: FAIL，缺少队列实现或声明。

- [ ] **Step 3: 为三路CAN创建静态FreeRTOS Queue**

CAN1容量32，CAN2/3容量8；使用`StaticQueue_t`和按`sizeof(can_rx_frame_t)`计算的静态storage。满时在ISR先`xQueueReceiveFromISR()`丢最旧，再`xQueueSendFromISR()`写最新，并累计overflow。

- [ ] **Step 4: 修改ISR为逐帧入队**

`HAL_FDCAN_RxFifo0Callback()`读取对应FIFO中的新帧，填充bus index和元数据后入队；不解析协议、不发布ROS。

- [ ] **Step 5: 将FDCAN IRQ library priority从0改为5**

三路IT0/IT1均设置priority 5。核对STM32FreeRTOS的`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，确保5不高于允许调用`FromISR`接口的优先级；保留`configASSERT`检查。

- [ ] **Step 6: control task成为唯一主队列消费者**

每个10 ms周期有界地drain队列，并把帧交给`motor_control_accept_feedback()`；若单周期达到预算仍有积压，留给下周期并增加backlog计数，禁止无界while饿死周期发送。

- [ ] **Step 7: 调试桥改为消费control task复制的低优先级最新原始帧**

控制任务在解析时覆盖每总线一个debug snapshot和sequence；`can_debug_bridge_poll()`读取该snapshot。调试路径允许采样，不与控制路径争抢主队列。

- [ ] **Step 8: 运行host测试和clean固件构建**

Run:

```bash
cmake --build .host-build
ctest --test-dir .host-build --output-on-failure
~/.platformio/penv/bin/pio run -t clean
~/.platformio/penv/bin/pio run
```

Expected: 全部PASS/SUCCESS；clean build确认HAL中断相关改动实际重编译。

- [ ] **Step 9: 提交CAN接收重构**

```bash
git add micro_ros_firmware/lib/fdcan micro_ros_firmware/src/control_task.cpp micro_ros_firmware/src/can_debug_bridge.cpp micro_ros_firmware/test/host
git commit -m "refactor: 使用静态队列承接 FDCAN 接收"
```

---

### Task 5: 修复IMU快照一致性并允许初始化降级

**Files:**
- Modify: `micro_ros_firmware/lib/imu/include/imu.h`
- Modify: `micro_ros_firmware/lib/imu/src/imu.c`
- Modify: `micro_ros_firmware/include/sensor_task.h`
- Modify: `micro_ros_firmware/src/sensor_task.cpp`
- Modify: `micro_ros_firmware/include/imu_bridge.h`
- Modify: `micro_ros_firmware/src/imu_bridge.cpp`
- Create: `micro_ros_firmware/test/host/test_imu_snapshot.cpp`
- Modify: `micro_ros_firmware/test/host/CMakeLists.txt`

**Interfaces:**
- Produces:

```c
bool imu_try_init(void);
bool sensor_task_get_imu_snapshot(imu_data_t *out, uint32_t *sequence);
uint32_t sensor_task_get_imu_init_error_count(void);
bool sensor_task_imu_healthy(void);
```

- [ ] **Step 1: 写双缓冲切换测试**

构造两个带不同哨兵值的`imu_data_t`，验证消费者只会得到完整A或完整B，不会得到字段混合体。

- [ ] **Step 2: 把无限初始化循环改为单次尝试接口**

```c
bool imu_try_init(void) {
  if (BMI088_init() != BMI088_NO_ERROR) {
    return false;
  }
  reset_attitude_state();
  g_ready = true;
  return true;
}
```

保留调用必须发生在调度器启动后的约束。

- [ ] **Step 3: sensor task按1 s间隔重试并继续heartbeat**

初始化失败增加计数、关闭加热输出、更新heartbeat并`vTaskDelay(1000 ms)`；成功后才挂DRDY中断并进入采样循环。

- [ ] **Step 4: 实现双缓冲和短临界切换**

producer完整写inactive buffer，再在critical section更新active index和sequence。`sensor_task_get_imu_snapshot()`在短critical section复制active buffer及sequence。

- [ ] **Step 5: imu bridge改为主动读取快照**

删除长期共享数据指针`imu_bridge_set_source()`；`imu_bridge_poll()`先调用`sensor_task_get_imu_snapshot()`，没有有效样本时静默跳过。

- [ ] **Step 6: 运行host测试和固件构建**

Run: `cmake --build .host-build && ctest --test-dir .host-build --output-on-failure && ~/.platformio/penv/bin/pio run`

Expected: 全部PASS/SUCCESS。

- [ ] **Step 7: 提交IMU一致性修复**

```bash
git add micro_ros_firmware/lib/imu micro_ros_firmware/include/sensor_task.h micro_ros_firmware/src/sensor_task.cpp micro_ros_firmware/include/imu_bridge.h micro_ros_firmware/src/imu_bridge.cpp micro_ros_firmware/test/host
git commit -m "fix: 保证 IMU 快照一致并支持降级重试"
```

---

### Task 6: 增加Software Supervisor和诊断计数

**Files:**
- Create: `micro_ros_firmware/include/supervisor_task.h`
- Create: `micro_ros_firmware/src/supervisor_task.cpp`
- Modify: `micro_ros_firmware/src/main.cpp`
- Modify: `micro_ros_firmware/include/ros_bridge.h`
- Modify: `micro_ros_firmware/src/ros_bridge.cpp`
- Modify: `micro_ros_firmware/src/control_task.cpp`
- Modify: `micro_ros_firmware/src/sensor_task.cpp`

**Interfaces:**
- Produces:

```c
typedef enum {
  SUPERVISOR_TASK_CONTROL = 0,
  SUPERVISOR_TASK_SENSOR = 1,
  SUPERVISOR_TASK_MICRO_ROS = 2,
  SUPERVISOR_TASK_COUNT = 3
} supervisor_task_id_t;

typedef struct {
  uint32_t fault_bitmask;
  uint32_t heartbeat[SUPERVISOR_TASK_COUNT];
  uint32_t miss_count[SUPERVISOR_TASK_COUNT];
  uint32_t last_check_ms;
} supervisor_diagnostics_t;

bool supervisor_task_start(void);
void supervisor_heartbeat(supervisor_task_id_t task);
void supervisor_get_diagnostics(supervisor_diagnostics_t *out);
```

- [ ] **Step 1: 定义静态heartbeat和fault bit**

至少包含control missed、sensor missed、micro-ROS state machine stalled、IMU degraded和FDCAN not ready；IMU degraded是诊断位，不等同于电机严重故障。

- [ ] **Step 2: 实现100 ms supervisor task**

每周期比较heartbeat快照；control必须每个检查窗口推进，sensor在初始化重试状态也必须推进，micro-ROS无Agent时状态机循环仍必须推进。开发阶段只更新计数，不复位。

- [ ] **Step 3: 在三个业务任务的确定位置上报heartbeat**

control在完成一次10 ms cycle后上报；sensor在初始化重试和每次采样循环上报；micro-ROS在每次状态机迭代末尾上报。

- [ ] **Step 4: 由start task创建supervisor**

supervisor优先级1、建议初始栈512 words。创建失败沿用`Error_Handler()`；创建成功后纳入README任务表。

- [ ] **Step 5: 用GDB可读结构聚合现有计数**

保留模块局部计数，同时提供`supervisor_diagnostics_t`读取任务健康；不新增ROS diagnostics实体。

- [ ] **Step 6: 构建并检查静态资源变化**

Run:

```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t size
```

Expected: SUCCESS；记录RAM/Flash并与规格前基线RAM 32,852 B、Flash 151,228 B比较。

- [ ] **Step 7: 提交supervisor**

```bash
git add micro_ros_firmware/include/supervisor_task.h micro_ros_firmware/src/supervisor_task.cpp micro_ros_firmware/src/main.cpp micro_ros_firmware/include/ros_bridge.h micro_ros_firmware/src/ros_bridge.cpp micro_ros_firmware/src/control_task.cpp micro_ros_firmware/src/sensor_task.cpp
git commit -m "feat: 增加固件任务健康监督"
```

---

### Task 7: 文档、完整验证和`dev-real`交接

**Files:**
- Modify: `micro_ros_firmware/README.md`
- Modify: `micro_ros_firmware/AGENTS.md`
- Modify: `micro_ros_firmware/test/README`
- Modify: `.gitignore` only if `.host-build/` is not already ignored

**Interfaces:**
- Consumes: Tasks 1–6全部稳定接口和实际构建资源数据。
- Produces: 开发者命令、故障语义、任务表和无负载真机验证清单。

- [ ] **Step 1: 更新任务和数据流文档**

写明control 100 Hz、supervisor 100 ms、sensor DRDY、micro-ROS 20 ms发布；同步实际stack words、优先级和职责。

- [ ] **Step 2: 更新安全状态和恢复语义**

明确100/300 ms、MIT阻尼、无抱闸风险、单RP slot故障、总线级失能、`micro_ros/motor_enable`和不重放旧命令。

- [ ] **Step 3: 更新CAN与IMU诊断说明**

记录CAN1=32、CAN2/3=8、丢最旧策略、IRQ priority约束、GDB计数，以及BMI088每秒重试和降级行为。

- [ ] **Step 4: 运行完整host测试**

Run:

```bash
cd micro_ros_firmware
cmake -S test/host -B .host-build
cmake --build .host-build --clean-first
ctest --test-dir .host-build --output-on-failure
```

Expected: 所有测试PASS，0 failed。

- [ ] **Step 5: 运行clean固件构建**

Run:

```bash
~/.platformio/penv/bin/pio run -t clean
~/.platformio/penv/bin/pio run
```

Expected: SUCCESS，无新增warning；记录最终RAM/Flash。

- [ ] **Step 6: 执行静态结构检查**

Run:

```bash
rg -n "fdcan|HAL_FDCAN" src/motor_bridge.cpp
rg -n "vTaskDelayUntil" src/control_task.cpp
rg -n "xQueueCreateStatic|xQueueSendFromISR" lib/fdcan/src/can_bsp.c
git diff --check
```

Expected: 第一条无命中；其余分别命中预期实现；`git diff --check`无输出。

- [ ] **Step 7: 编写但不自动执行无负载真机清单**

清单包含100 Hz MIT、100/300 ms timeout、Agent断连/重连、`motor_enable`授权、非法数值、CAN无ACK、RX burst、RP多slot和BMI088异常。注明只能在`dev-real`、无负载、硬件急停可用时执行。

- [ ] **Step 8: 提交文档和最终验证结果**

```bash
git add micro_ros_firmware/README.md micro_ros_firmware/AGENTS.md micro_ros_firmware/test/README .gitignore
git commit -m "docs: 更新固件控制安全与验证指南"
```

- [ ] **Step 9: 检查最终提交范围**

Run:

```bash
git status --short
git log --oneline --decorate -8
git diff HEAD~7 --stat
```

Expected: 工作区干净；提交只涉及规格中列出的固件、测试和文档路径，不修改`ros2_ws`业务代码。
