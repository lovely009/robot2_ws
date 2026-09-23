# motor_tx_router

`motor_tx_router` owns the join between the protocol-agnostic per-bus queue and
physical CAN submission policy. It selects one bounded P0/P1/P2 pump, accounts
for protocol adapters that reduce several P1 commands to one physical frame,
and reserves the P2 pump quota when a pump starts with an empty P0 queue. If P0
is queued at pump start, the router dequeues only P0. A hardware-rejected P0
stops that pump; it and all unattempted P0 frames return to the FIFO front in
their original order.

The module is pure C. It depends on `fdcan` queue types and the
`motor_protocol` aggregation interface, but has no HAL, FreeRTOS, ROS, or heap
dependency. A submit callback is the only hardware seam. `motor_rt_task` owns one
router instance and calls it serially for each bus; the fixed scratch buffer is
reused between buses.

P1 remains latest-wins. A rejected aggregated P1 frame is dropped until the
next latest-wins update, while each P0 frame is retried. A lone CERB MIT command
passes through unchanged; compatible adjacent CERB commands are encoded by the
CERB adapter as one multi-axis frame. The router itself does not contain CAN
IDs or manufacturer payload layouts.
