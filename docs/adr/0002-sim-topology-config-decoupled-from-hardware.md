# 仿真拓扑配置与真机解耦（motor_canfd_sim 自持 motor_canfd.yaml）

**Status**: accepted

仿真不再复用 `motor_hardware_interface/config/motor_canfd.yaml`：`motor_canfd_sim` 自带
`config/motor_canfd.yaml`，其电机拓扑来源于仿真模型（URDF 与 Gazebo effort controller
关节顺序），并作为 `motor_canfd_sim`、`model_sim`、`driver_loopback` 三个 launch 的
默认 `motor_config_file`。真机配置仍由 `motor_hardware_interface` 持有；两侧共用
`load_driver_config()` 解析 API，只解耦配置数据，不重复实现 YAML 解析。本决策取代
ADR 0001 中“真机与仿真共享同一份 motor_canfd.yaml”的部分。

## Considered Options

- **继续共享单一配置**：仿真侧免维护，但仿真模型的拓扑差异必须挤进同一份真机
  配置文件；改仿真模型会牵动真机配置，违背“仿真的电机拓扑来自仿真模型”的边界。
- **仿真自持配置（已选）**：两侧独立演进，代价是两份配置可能漂移，需要依靠
  README 标注的“仿真模型 ↔ 仿真拓扑”同步约束与 launch 静态测试兜底。

## Consequences

- 新增/删除仿真关节或电机：修改 URDF、controllers 关节顺序与
  `motor_canfd_sim/config/motor_canfd.yaml`，真机配置不受影响。
- 新增电机家族仍需在 `motor_canfd_drive` 中扩展协议策略，并按需在两侧配置声明。
- `bus_override.*` 保留：配置写“默认接口”，launch 在启动时把逻辑总线映射到任意
  SocketCAN 接口（默认路径下与配置一致；HIL 时把 vcan 重定向到真实 can 接口）。
  这是逻辑总线与接口解耦的运行期映射，不是冗余，不在两侧之间造成复制。

## 仿真参数与真机参数的边界

两侧配置只共享解析 API 与 YAML 结构，参数语义按下表划分：

| 参数类别 | 归属 | 边界规则 |
|---|---|---|
| 拓扑参数（joint/type/id/bus） | 两侧 | 同一台机器人必须一致，差异即漂移，由拓扑一致性测试兜底 |
| 控制参数（`motor_types.command` 的 kp/kd） | 驱动侧 | 模拟器从 CAN 命令帧读取，不读配置，天然不重复 |
| 物理换算参数（`torque_constant_nm_per_amp`） | 两侧各自持有 | 真机侧为标定值（驱动换算反馈用）；仿真侧为 plant 假设（只影响 Gazebo 动力学）。数值不同是设计使然，不是漂移 |
| 仿真专属参数（限幅、模式增益、频率、命令超时） | 仿真侧 | 真机无对应物 |
| 时序安全参数（`feedback_timeout_s` / `initial_feedback_wait_s`） | 两侧 | 驱动侧语义相同，数值可各自调（真机更保守） |

防混用**不引入运行时机制**（不加 YAML 标记或启动拒绝）：真机首次标定是人工流程，
靠配置注释、本 ADR 与拓扑一致性测试兜底即可。

### 仿真假设参数清单

接真机前必须替换为标定/实测值的仿真假设：

- `motor_2530.torque_constant_nm_per_amp`（当前 0.35 Nm/A）
- RP 扭矩上限 `rp_torque_limit_nm`（当前 60 Nm）
- 腕部位置/速度模式增益（`motor_2530_position_*` / `motor_2530_velocity_mode_*`）
- URDF 各连杆质量与惯量
