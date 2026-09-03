# Firmware Entry and Task Modularization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将固件入口收缩为系统初始化与任务编排，并把 micro-ROS 状态机、LED 状态显示和板级初始化拆为独立模块。

**Architecture:** `main.cpp` 保留 Arduino 入口和 start task；`system.cpp` 提供板级初始化及 FreeRTOS 静态内存回调；`micro_ros_task.cpp` 独占 agent 连接状态机；`led_task.cpp` 作为 WS2812 唯一写入者。状态到颜色的映射抽成 host 可测的纯 C 逻辑。

**Tech Stack:** STM32H723VGT6, STM32duino, STM32FreeRTOS, PlatformIO, C/C++17, CMake/CTest

**Spec:** `docs/superpowers/specs/2026-09-03-firmware-entry-task-modularization-design.md`

## Global Constraints

- 保留 STM32duino 必需的 `setup()`/`loop()`，不定义项目自有 `int main()`。
- `system.h` 只聚合 Arduino/FreeRTOS 系统头文件；业务模块仍显式包含直接依赖。
- sensor/control 任务的优先级、栈、周期和初始化顺序不变。
- LED 任务是 WS2812 的唯一运行期写入者，每 50 ms 轮询连接状态。
- 首连前红灯，连接中保持原颜色，运行时绿灯 1 Hz 闪烁，断连或重连等待时蓝灯。
- 删除 W25Q64 启动探测和构建开关，保留 `lib/w25q64` 及 geometry host test。
- 不恢复 supervisor，不改 ROS 话题、节点名或 executor 实体。
- 当前工作树有 CREB 和 supervisor 的未提交改动；每次提交只 stage 本任务列出的文件。

---

### Task 1: Host-Testable LED State Policy

**Files:**
- Create: `micro_ros_firmware/include/micro_ros_link_state.h`
- Create: `micro_ros_firmware/include/led_status.h`
- Create: `micro_ros_firmware/src/led_status.c`
- Create: `micro_ros_firmware/test/host/test_led_status.cpp`
- Modify: `micro_ros_firmware/test/host/CMakeLists.txt`

**Interfaces:**
- Produces: `micro_ros_link_state_t`
- Produces: `led_color_t led_status_select(micro_ros_link_state_t state, bool ever_connected, bool blink_on, led_color_t previous)`

- [ ] **Step 1: Write the failing policy test**

Create literal assertions for: first wait = `{1,0,0}`; run/blink-on = `{0,1,0}`; run/blink-off = `{0,0,0}`; lost = `{0,0,1}`; reconnect wait with `ever_connected=true` = blue; connect = exact previous color.

```cpp
#include <cassert>
#include "led_status.h"

int main() {
  const led_color_t red = led_status_select(
      MICRO_ROS_LINK_WAIT_AGENT, false, false, {9U, 8U, 7U});
  assert(red.r == 1U && red.g == 0U && red.b == 0U);
  const led_color_t held = led_status_select(
      MICRO_ROS_LINK_CONNECT, false, false, {9U, 8U, 7U});
  assert(held.r == 9U && held.g == 8U && held.b == 7U);
  const led_color_t green = led_status_select(
      MICRO_ROS_LINK_RUN, true, true, {0U, 0U, 0U});
  assert(green.r == 0U && green.g == 1U && green.b == 0U);
  const led_color_t off = led_status_select(
      MICRO_ROS_LINK_RUN, true, false, {0U, 1U, 0U});
  assert(off.r == 0U && off.g == 0U && off.b == 0U);
  const led_color_t lost = led_status_select(
      MICRO_ROS_LINK_LOST, true, false, {0U, 1U, 0U});
  assert(lost.r == 0U && lost.g == 0U && lost.b == 1U);
  const led_color_t reconnect_wait = led_status_select(
      MICRO_ROS_LINK_WAIT_AGENT, true, false, {1U, 0U, 0U});
  assert(reconnect_wait.r == 0U && reconnect_wait.g == 0U &&
         reconnect_wait.b == 1U);
}
```

- [ ] **Step 2: Verify RED**

Run: `cmake -S micro_ros_firmware/test/host -B micro_ros_firmware/.host-build && cmake --build micro_ros_firmware/.host-build --target test_led_status`

Expected: FAIL because `led_status.h`/`led_status_select` does not exist.

- [ ] **Step 3: Implement the minimal pure policy**

Define fixed `uint8_t r/g/b`, return `previous` for `CONNECT`, choose green from `blink_on` for `RUN`, blue for `LOST` and post-connect wait, otherwise red. Do not include Arduino or FreeRTOS headers.

- [ ] **Step 4: Verify GREEN**

Run: `cmake --build micro_ros_firmware/.host-build --target test_led_status && micro_ros_firmware/.host-build/test_led_status`

Expected: build succeeds and process exits 0.

- [ ] **Step 5: Commit**

```bash
git add micro_ros_firmware/include/micro_ros_link_state.h micro_ros_firmware/include/led_status.h micro_ros_firmware/src/led_status.c micro_ros_firmware/test/host/test_led_status.cpp micro_ros_firmware/test/host/CMakeLists.txt
git commit -m "test: define firmware LED state policy"
```

### Task 2: Extract the micro-ROS Task

**Files:**
- Create: `micro_ros_firmware/include/micro_ros_task.h`
- Create: `micro_ros_firmware/src/micro_ros_task.cpp`
- Modify: `micro_ros_firmware/src/main.cpp`

**Interfaces:**
- Consumes: `micro_ros_link_state_t` from Task 1; existing `ros_bridge_create/destroy/spin`
- Produces: `bool micro_ros_task_start(void)`, `micro_ros_link_state_t micro_ros_task_get_link_state(void)`, `bool micro_ros_task_ever_connected(void)`

- [ ] **Step 1: Change the caller first**

Remove the local LinkState, LED rendering and `micro_ros_task()` implementation from `main.cpp`; include `micro_ros_task.h` and call `micro_ros_task_start()` from `start_task()`.

- [ ] **Step 2: Verify RED**

Run: `cd micro_ros_firmware && ~/.platformio/penv/bin/pio run`

Expected: FAIL at compile/link because the new task interface has no implementation.

- [ ] **Step 3: Implement the task module**

Use `MICRO_ROS_TASK_PRIO=2`, `MICRO_ROS_STK_SIZE=8192`, a private task handle and the existing four-state loop. Preserve ping `(200, 3)`, 10 ms loop delay, bridge creation/destruction order and `Error_Handler()` behavior. Guard state snapshots and `ever_connected` with short `taskENTER_CRITICAL()`/`taskEXIT_CRITICAL()` sections; never hold a critical section across ping/spin/create/destroy.

- [ ] **Step 4: Verify GREEN**

Run: `cd micro_ros_firmware && ~/.platformio/penv/bin/pio run`

Expected: firmware links successfully; no duplicate `micro_ros_task` or LinkState remains in `main.cpp`.

- [ ] **Step 5: Commit**

```bash
git add micro_ros_firmware/include/micro_ros_task.h micro_ros_firmware/src/micro_ros_task.cpp micro_ros_firmware/src/main.cpp
git commit -m "refactor: extract micro-ROS task"
```

### Task 3: Add the Independent LED Task

**Files:**
- Create: `micro_ros_firmware/include/led_task.h`
- Create: `micro_ros_firmware/src/led_task.cpp`
- Modify: `micro_ros_firmware/src/main.cpp`

**Interfaces:**
- Consumes: `micro_ros_task_get_link_state()`, `micro_ros_task_ever_connected()`, `led_status_select()`, `ws2812_set_rgb()`
- Produces: `bool led_task_start(void)`

- [ ] **Step 1: Add the caller before the implementation**

Include `led_task.h` in `main.cpp` and make `start_task()` call `led_task_start()` before `micro_ros_task_start()`.

- [ ] **Step 2: Verify RED**

Run: `cd micro_ros_firmware && ~/.platformio/penv/bin/pio run`

Expected: FAIL because `led_task_start()` is missing.

- [ ] **Step 3: Implement the LED task**

Use `LED_TASK_PRIO=1`, `LED_STK_SIZE=256`, a private handle and a 50 ms `vTaskDelayUntil` loop. Compute `blink_on` from `millis()/500U`, pass the last emitted color as `previous`, and call `ws2812_set_rgb()` only when RGB changes. Initialize `previous` to an impossible `{0xFF,0xFF,0xFF}` value so the first iteration emits red.

- [ ] **Step 4: Verify GREEN and the policy regression**

Run: `cmake --build micro_ros_firmware/.host-build --target test_led_status && micro_ros_firmware/.host-build/test_led_status && cd micro_ros_firmware && ~/.platformio/penv/bin/pio run`

Expected: policy test passes and firmware links.

- [ ] **Step 5: Commit**

```bash
git add micro_ros_firmware/include/led_task.h micro_ros_firmware/src/led_task.cpp micro_ros_firmware/src/main.cpp
git commit -m "feat: run link status LED in dedicated task"
```

### Task 4: Extract System Initialization and Minimize main.cpp

**Files:**
- Create: `micro_ros_firmware/include/system.h`
- Create: `micro_ros_firmware/src/system.cpp`
- Modify: `micro_ros_firmware/src/main.cpp`
- Modify: `micro_ros_firmware/platformio.ini`

**Interfaces:**
- Produces: `void initialize_system(void)`
- Consumes: `ws2812_init()`, STM32duino `Serial`, `set_microros_serial_transports()`

- [ ] **Step 1: Move declarations and make the build fail at the new seam**

Make `main.cpp` include `system.h`; remove local serial constants, static-allocation callbacks and `initialize_system()`. Remove the W25Q64 conditional include, probe and diagnostic globals. Remove `-DENABLE_W25Q64=0` from `platformio.ini`.

- [ ] **Step 2: Verify RED**

Run: `cd micro_ros_firmware && ~/.platformio/penv/bin/pio run`

Expected: FAIL because `system.h` and `initialize_system()` are not yet provided.

- [ ] **Step 3: Implement `system.h/.cpp`**

`system.h` includes `<Arduino.h>` and `<STM32FreeRTOS.h>` and declares `initialize_system()`. `system.cpp` includes `system.h`, `<micro_ros_platformio.h>` and `ws2812.h`; defines idle/timer static memory callbacks with 128/256-word stacks; initializes WS2812, starts `Serial` at placeholder 115200, installs serial transport and delays 1000 ms. It must not set an LED color.

- [ ] **Step 4: Normalize the final main style**

Use `START_TASK_PRIO`, `START_STK_SIZE`, `TaskHandle_t StartTask_Handler`, and a forward declaration for `start_task(void *pv_parameters)`. `setup()` performs only system initialization, start-task creation and scheduler start. Check every `*_task_start()` return and call `Error_Handler()` on failure; delete the start task with `vTaskDelete(StartTask_Handler)`. Keep `loop()` empty and non-blocking. Do not wrap task creation in a critical section.

- [ ] **Step 5: Verify GREEN**

Run: `cd micro_ros_firmware && ~/.platformio/penv/bin/pio run -t clean && ~/.platformio/penv/bin/pio run`

Expected: dependency graph excludes the removed W25 startup probe path; firmware links successfully.

- [ ] **Step 6: Commit**

```bash
git add micro_ros_firmware/include/system.h micro_ros_firmware/src/system.cpp micro_ros_firmware/src/main.cpp micro_ros_firmware/platformio.ini
git commit -m "refactor: reduce firmware entry to task orchestration"
```

### Task 5: Documentation and Final Verification

**Files:**
- Modify: `micro_ros_firmware/README.md`
- Modify: `micro_ros_firmware/AGENTS.md`
- Modify: `micro_ros_firmware/include/README`
- Modify: `micro_ros_firmware/test/README`

**Interfaces:**
- Consumes: final module names and task parameters from Tasks 1–4
- Produces: accurate operator and maintainer documentation

- [ ] **Step 1: Update documentation**

Document the four business tasks, LED priority/stack/50 ms period, the new file layout, direct-include/library dependency rule, removal of W25 startup probing, and the unchanged retained `lib/w25q64`. Change the host-test count to include `led_status` and remove any statement that `main.cpp` owns the micro-ROS state machine or LED rendering.

- [ ] **Step 2: Run structural checks**

```bash
git diff --check
test -z "$(rg -l 'g_w25q64_init_status|g_w25q64_jedec_id|ENABLE_W25Q64' micro_ros_firmware --glob '!.pio/**' --glob '!build/**')"
test -z "$(rg -l 'rmw_uros_ping_agent|ws2812_set_rgb' micro_ros_firmware/src/main.cpp)"
```

Expected: all commands exit 0.

- [ ] **Step 3: Run all host tests**

Run: `cmake -S micro_ros_firmware/test/host -B micro_ros_firmware/.host-build && cmake --build micro_ros_firmware/.host-build --clean-first && ctest --test-dir micro_ros_firmware/.host-build --output-on-failure`

Expected: `led_status` and all existing tests pass except the already documented `motor_control` assertion at `test_motor_control.cpp:191`. Run `ctest --test-dir micro_ros_firmware/.host-build --output-on-failure -E '^motor_control$'` and require 100% pass for the unaffected set.

- [ ] **Step 4: Run the target build**

Run: `cd micro_ros_firmware && ~/.platformio/penv/bin/pio run -t clean && ~/.platformio/penv/bin/pio run`

Expected: SUCCESS; record RAM/Flash bytes and confirm no new warnings beyond the existing `HAL_OSPI_MODULE_ENABLED` redefinition warning.

- [ ] **Step 5: Request code review and address findings**

Review against the linked spec, with special attention to Arduino entry compatibility, task-create failure paths, WS2812 single-writer ownership, critical-section duration and direct include dependencies. Re-run Steps 2–4 after any production change.

- [ ] **Step 6: Commit**

```bash
git add micro_ros_firmware/README.md micro_ros_firmware/AGENTS.md micro_ros_firmware/include/README micro_ros_firmware/test/README
git commit -m "docs: describe modular firmware task entry"
```
