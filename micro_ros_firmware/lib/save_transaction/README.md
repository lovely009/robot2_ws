# save_transaction

`save_transaction` coordinates one whole-system save without owning any motor,
ROS, RTOS, or flash implementation. It snapshots dirty device slots, submits a
bounded number of device-save requests per poll, correlates their result IDs,
and requests W25 persistence only after every device save succeeds.

The module exposes narrow injected ports for request-ID allocation, device
submission, storage request/result, and error reporting. The two phases use a
wrap-safe 5-second watchdog. A failed device phase drains already accepted
requests but never starts W25; storage failure cannot roll back device flash.
Duplicate SAVE requests merge into one queued follow-up, and disconnect clears
active and queued work without changing the ROS-owned completion sequence.
