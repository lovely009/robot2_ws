# Firmware Architecture Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `executing-plans` to execute these steps inline. Each task ends with a focused verification checkpoint.

**Goal:** Reshape the firmware around high-cohesion, low-coupling modules, with explicit interface and formatting constraints that are testable on the host.

**Architecture:** Add a pure-C `motor_tx_router` above the FDCAN queue scheduler and below the HAL. The router owns P0/P1/P2 budgets, physical-frame accounting, and retry policy; `motor_protocol` adapters own protocol-specific P1 aggregation and encoding. Extract the asynchronous SAVE phases into a pure-C `save_transaction` module whose effects are injected through `param_bridge` callbacks. Keep mailbox and protocol config kinds as separate enum types and make their numeric contract exhaustive at the bridge seam.

**Tech Stack:** C17, C++17, PlatformIO + STM32duino + FreeRTOS, CMake host targets, self-contained assert-based C++ test executables, Uncrustify, EditorConfig, Markdown.

**Scope:** The user selected all four reviewed architecture candidates: TX routing and protocol aggregation, SAVE transaction extraction, the config-kind contract, and documentation/format constraints. Acceptance details and ownership constraints are recorded below and in `docs/architecture/module-boundaries.md`, `docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md`, and the firmware README.

## Global Constraints

- Firmware is STM32H723VGT6, built by PlatformIO; do not use `colcon` for this workspace.
- `motor_rt_task` remains the sole runtime owner of normal CAN TX and the sole caller of `can_bsp_submit_tx()`.
- Manufacturer byte layouts, CAN identifiers, DLC, and FD/BRS frame attributes remain owned by `lib/motor_protocol` adapters.
- The FDCAN queue scheduler remains protocol-agnostic and free of HAL/FreeRTOS dependencies.
- The ROS bridge communicates with the motor runtime through mailboxes/events and with storage through `storage_task`; it never sends raw CAN or accesses W25 directly.
- C and C++ use 2-space indentation; Python uses 4-space indentation; host tests never start Gazebo, flash firmware, or drive motors.
- Preserve every pre-existing worktree change. Only edit the listed firmware paths and the root context map.
- Run host tests and PlatformIO compile as the verification for this implementation; do not run `clean_microros`.

## File Structure

- `lib/motor_protocol/include/motor_protocol_adapter.h`, `lib/motor_protocol/src/motor_protocol_registry.c`, and `lib/motor_protocol/src/cerb_adapter.c` own the adapter contract, shared registry, and CERB multi-axis aggregation.
- `lib/motor_protocol/src/motor_protocol_batch.c` dispatches P1 batching through registered protocol adapters.
- `lib/motor_tx_router/` owns queue dequeue budgets, logical-to-physical frame accounting, HAL callback invocation, and P0 retry ordering.
- `lib/save_transaction/` owns asynchronous SAVE state, bounded submission, request correlation, storage handoff, and timeouts; it has no ROS, RTOS, motor-control, or flash dependency.
- `src/motor_rt_task.cpp` and `src/param_bridge.cpp` remain orchestration adapters that connect the new modules to existing owners.
- `include/motor_config_contract.hpp` and `test/host/test_config_kind_contract.cpp` lock the two enum domains together without collapsing their types.
- `CONTEXT.md`, root `CONTEXT-MAP.md`, README/AGENTS/architecture docs, `.editorconfig`, `.uncrustify.cfg`, and `tools/format_firmware.sh` establish navigable documentation and formatting rules.

---

### Task 1: Add the protocol-owned P1 aggregation seam

**Files:**
- Modify: `lib/motor_protocol/include/motor_protocol_adapter.h`
- Modify: `lib/motor_protocol/src/cerb_adapter.c`
- Create: `lib/motor_protocol/src/motor_protocol_registry.c`
- Create: `lib/motor_protocol/src/motor_protocol_batch.c`
- Modify: `test/host/CMakeLists.txt`
- Modify: `test/host/test_motor_protocol_adapter.cpp`

**Interface:**
- Runtime binding and P1 batching consume one adapter registry owned by `motor_protocol`.
- Add `uint8_t p1_batch_max` and an optional `aggregate_p1_frames(frames, available, out)` callback to `motor_protocol_adapter_t`.
- Add `size_t motor_protocol_aggregate_p1_frames(const motor_protocol_tx_frame_t *frames, size_t available, motor_protocol_tx_frame_t *out)`; it returns the number of input frames consumed, returns `1` for pass-through, and returns `0` only for invalid arguments.
- The CERB callback combines only adjacent, same-bus P1 single-axis MIT frames, at most `MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES`; it returns pass-through for one frame or an invalid/duplicate-node group. It builds the full `0x210` FD+BRS frame within `motor_protocol`.

- [x] Add adapter tests for pass-through of Vaxor, pass-through of a single CERB command, aggregation of 2–6 CERB commands, split at a non-CERB frame, and duplicate-node fallback.
- [x] Add `motor_protocol_batch.c` to the host adapter object library and run the focused `test_motor_protocol_adapter` executable to prove the new tests fail before implementation.
- [x] Implement the adapter callback and generic dispatch function; initialize `p1_batch_max=1` for Vaxor and `MOTOR_PROTOCOL_CERB_MULTI_MAX_AXES` for CERB.
- [x] Re-run the focused adapter executable and confirm the aggregate has a 64-byte payload with node IDs in bytes 56–61 and zero-filled unused bytes.

### Task 2: Deepen TX routing behind a host-testable module

**Files:**
- Create: `lib/motor_tx_router/include/motor_tx_router.h`
- Create: `lib/motor_tx_router/src/motor_tx_router.c`
- Create: `lib/motor_tx_router/library.json`
- Create: `lib/motor_tx_router/README.md`
- Modify: `lib/fdcan/include/can_tx_scheduler.h` only if the router needs a clarified dequeue contract; keep its queue policy protocol-agnostic.
- Modify: `src/motor_rt_task.cpp`
- Modify: `test/host/CMakeLists.txt`
- Create: `test/host/test_motor_tx_router.cpp`

**Interface:**
- `motor_tx_router_t` contains a fixed 40-frame logical pump buffer and a fixed protocol-batch lookahead buffer; no heap allocation or HAL/RTOS state.
- `motor_tx_router_pump(router, scheduler, bus_index, submit, context)` performs exactly one scheduler pump and returns counters for dequeued P0/P1/P2 logical frames, physical attempts, rejections, and P0 requeues.
- The injected submit function has signature `bool (*)(void *context, uint8_t bus_index, const can_tx_frame_t *frame)`.
- If the pump starts with P0 queued, it dequeues P0 only. The first physical rejection stops submission for that pump; the rejected and unattempted P0 frames return to the FIFO front in order. P1 is latest-wins and may be coalesced only through the protocol adapter seam. P2 keeps its per-pump quota on pumps that start with P0 empty.
- A lone CERB MIT frame is sent unchanged. Two or more compatible CERB frames become one protocol-built multi-axis frame. A rejected aggregated P1 is dropped until the next latest-wins update, matching current P1 retry semantics.

- [x] Add fake-submit tests for P0 ordering/requeue, stop-on-rejection and preservation of queued P1/P2, P1 pass-through/aggregation, P2 reservation, the 16-frame physical ceiling, and no aggregation across incompatible frames; first run to establish failures.
- [x] Implement scheduler selection that accounts for physical batches before dequeue, while respecting the 40 logical-frame cap, 16 physical-frame cap, and P0/P2 priority.
- [x] Replace `p1_pump_limit`, `submit_cerb_multi`, and the pump loop in `motor_rt_task.cpp` with the router and a thin `can_bsp_submit_tx` adapter.
- [x] Add a local PlatformIO library manifest and host target dependencies for `fdcan` and `motor_protocol`; run focused router and scheduler tests.

### Task 3: Extract the asynchronous SAVE transaction

**Files:**
- Create: `lib/save_transaction/include/save_transaction.h`
- Create: `lib/save_transaction/src/save_transaction.c`
- Create: `lib/save_transaction/library.json`
- Create: `lib/save_transaction/README.md`
- Modify: `src/param_bridge.cpp`
- Modify: `test/host/CMakeLists.txt`
- Create: `test/host/test_save_transaction.cpp`

**Interface:**
- A target is `{ uint16_t motor_id; uint8_t dirty_slots; }`; module capacity is fixed at 64 devices.
- Ports inject request-ID allocation, device config submission, storage-save request/result polling, and error reporting. Storage status is represented by a module-local enum and mapped by `param_bridge`.
- Public operations are `save_transaction_init`, `save_transaction_request`, `save_transaction_poll`, `save_transaction_on_device_result`, `save_transaction_disconnect`, and read-only `save_transaction_in_progress`.
- `save_transaction_poll` starts a queued request from a snapshot of dirty targets, submits no more than four device requests per poll, waits for only matching config-result IDs, starts W25 only after all device results succeed, and times out either phase after 5000 ms using wrap-safe unsigned elapsed time.
- Duplicate SAVE requests coalesce into at most one queued follow-up. Disconnect clears active and queued work but does not alter the externally owned `save_seq`.

- [x] Add fake-port tests for dirty-target filtering, four-per-poll submission, unmatched/late result IDs, device rejection and failure drain, storage request only after all device successes, storage ID correlation, each phase timeout, duplicate-request coalescing, and disconnect cleanup; run to prove failures.
- [x] Implement the pure-C state machine and test it without Arduino, ROS, motor-control, FreeRTOS, or W25 headers.
- [x] Replace the SAVE globals/helpers in `param_bridge.cpp` with one `save_transaction_t`, thin port callbacks, and `save_seq`/status integration.
- [x] Build and run the focused SAVE transaction executable, then compile the entire host target set.

### Task 4: Make config-kind synchronization exhaustive

**Files:**
- Create: `include/motor_config_contract.hpp`
- Modify: `src/motor_rt_task.cpp`
- Modify: `test/host/CMakeLists.txt`
- Create: `test/host/test_config_kind_contract.cpp`
- Modify: `AGENTS.md`

**Interface:**
- Keep `control_mailbox_config_kind_t` and `motor_protocol_config_kind_t` separate.
- `motor_config_contract.hpp` lists every mirrored enumerator from `SET_MODE` through `SAVE_DEVICE_FLASH` and applies one `static_assert` per pair; protocol-only `READ_IDENTITY=7` remains outside the mailbox mapping.
- The host contract target compiles the shared assertions and pins all public numeric values, including `SET_POLICY=11`.

- [x] Add compile-time assertions for every mirrored value and host assertions for the complete numeric set; run the focused contract target and ensure the test covers the previously omitted `SET_POLICY`.
- [x] Replace the partial assertion block in `motor_rt_task.cpp` with the shared contract header; keep the typed payload conversion switch at the application seam.
- [x] Update the firmware agent guide to say a new mirrored kind must update both enum declarations and this exhaustive contract.

### Task 5: Establish documentation and formatting constraints

**Files:**
- Create: `CONTEXT.md`
- Create: `../CONTEXT-MAP.md`
- Create: `.editorconfig`
- Create: `.uncrustify.cfg`
- Create: `tools/format_firmware.sh`
- Create: `docs/architecture/lib-library-audit.md`
- Modify: `docs/plans/active/motor-runtime-validation.md`
- Modify: `lib/motor_tx_router/README.md`
- Modify: per-library `README.md` files for modules missing owner/contract documentation
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `docs/architecture/module-boundaries.md`
- Modify: `docs/architecture/motor-runtime-decision-log.md`
- Modify: `docs/adr/0007-micro-ros-discovery-driven-motor-runtime.md`

- [x] Add the firmware glossary (`CAN TX router`, `P0/P1/P2`, `device SAVE`, `W25 SAVE`, `dirty slot`) and a root context map covering ROS2, firmware, and micro-ROS Agent workspace authorities.
- [x] Update the firmware README module table/data flow/directory tree, module-boundaries test rows, decision log, and ADR current-state timing so they describe the router, adapter batching seam, SAVE state machine, and 2 ms/500 Hz control boundary.
- [x] Add EditorConfig rules for UTF-8/LF/final newline, 2-space C/C++/Markdown indentation, and 4-space Python indentation; add Uncrustify rules and a format script supporting `--check` and `--write` over firmware C/C++ source, public/internal headers, libraries, and host tests only.
- [x] Apply the formatter to the firmware source set; inspect the complete diff for accidental non-format changes and do not format sibling workspaces.

### Task 6: End-to-end verification and self-review

**Files:**
- Review: all files changed by Tasks 1–5.

- [x] Run `cmake -S test/host -B .host-build` and `cmake --build .host-build --clean-first -j2`.
- [x] Run `ctest --test-dir .host-build --output-on-failure` and record the final test count.
- [x] Run `~/.platformio/penv/bin/pio run` from `micro_ros_firmware/`.
- [x] Run `tools/format_firmware.sh --check` and confirm no formatted file changes.
- [x] Review `git diff --check`, verify the root and sibling worktree changes remain untouched, and inspect the final firmware diff for API ownership, stale timing comments, duplicated SAVE state, and P0 reject/priority ordering.


## Verification record

- Host CMake clean-first build completed successfully.
- CTest passed all 22 registered tests, including the router, SAVE, and config-kind contract targets.
- PlatformIO compiled and linked `firmware.elf` successfully for `stm32h723vgt6`.
- `tools/format_firmware.sh --check` reported clean formatting for 120 firmware C/C++ files.
- `git diff --check` passed.
- Read-only review found and closed a P0 rejection ordering defect with a regression test; a follow-up review confirmed P1/P2 remain queued while P0 drains.
- The unverified single-bus VAXOR capacity is recorded as `TBD-CAN-02` in `docs/plans/active/motor-runtime-validation.md`.
