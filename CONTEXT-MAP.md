# Context map

Load the context document for the workspace you are changing:

| Scope | Context | Authority |
|---|---|---|
| ROS 2 robot workspace | [`CONTEXT.md`](CONTEXT.md) | Package `README.md` files and `docs/adr/` in `ros2_ws/` |
| STM32 micro-ROS firmware | [`micro_ros_firmware/CONTEXT.md`](micro_ros_firmware/CONTEXT.md) | `micro_ros_firmware/README.md`, its `AGENTS.md`, and `docs/adr/` |
| micro-ROS Agent workspace | — | [`micro_ros_agent/README.md`](micro_ros_agent/README.md) |

The ROS 2, firmware, and agent workspaces have independent build and runtime
boundaries. Read the relevant context and package README before changing an area.
