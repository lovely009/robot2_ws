# Firmware domain context

The firmware owns device discovery, motor safety and execution, CAN transport,
IMU acquisition, and persistent device parameters. ROS bridges translate messages
at the application boundary; they do not own hardware state. The architecture
authority is `docs/architecture/module-boundaries.md`, while hardware facts,
interfaces, and commands are maintained in `README.md`.

## Terms and ownership

| Term | Meaning and owner |
|---|---|
| CAN TX router | `lib/motor_tx_router`: chooses bounded P0/P1/P2 work, accounts for the physical-frame budget, and calls an injected submit port. A pump that starts with queued P0 handles only P0; the first rejection halts submission and restores that frame plus unattempted P0 frames in order. P1/P2 run on a pump that starts with an empty P0 queue. `motor_rt_task` owns the router instance and is the only normal HAL submitter. |
| P0 / P1 / P2 | Scheduler classes: safety actions, latest-wins control, and discovery/configuration traffic. `lib/fdcan` owns their protocol-neutral queues; the router owns dequeue and physical submission policy. |
| P1 aggregation | `lib/motor_protocol` adapter behavior. CERB combines compatible adjacent MIT commands; Vaxor and incompatible or singleton input pass through. The scheduler does not know protocol layouts. |
| Device SAVE | A request for dirty, save-capable device parameter slots to be persisted by the motor itself. `lib/save_transaction` waits for correlated device results before proceeding. |
| W25 SAVE | Persistence of the firmware's parameter projection to the external W25Q64. `storage_task` alone accesses the flash; it runs only after the device-save phase succeeds. |
| Dirty slot | A parameter slot whose live device value differs from the firmware's persisted record. Dirty state is supplied by the motor runtime snapshot and filtered by the transaction coordinator. |

## Change constraints

Keep protocol bytes and CAN identifiers in `lib/motor_protocol`, queue storage
and P0/P1/P2 class mechanics in `lib/fdcan`, and their bounded join in
`lib/motor_tx_router`. Keep SAVE sequencing in the pure-C transaction module;
ROS and storage callbacks remain in `src/param_bridge.cpp`.

Append protocol adapters to the registry: its order is also the numeric protocol
ID in topology snapshots, so existing entries keep their positions.

The mailbox and protocol config-kind enums remain distinct types. Their shared
numeric values are exhaustively checked by `include/motor_config_contract.hpp`;
protocol-only values stay outside that mapping. Preserve single-writer ownership
for the motor runtime, normal CAN TX, and W25Q64.
