# robot_calibration

为特定装配保存配置摘要和人工标定验收记录，启动时只读核验。节点不会改零位、限位、方向或任何电机参数，也不会代替物理测量。

配置摘要覆盖驱动 YAML、URDF、`motor_specs_file` 和 `transmissions_file` 的原始内容；没有显式传动文件时，包含共享传动库实际读取的同目录 `transmissions.yaml`。路径本身不参与摘要，相同内容从源码安装到 ROS share 后摘要保持一致。配置、传动或限位变化后，旧验收记录失效。

显式相对文件存在时始终优先使用。隔离安装树中，本仓库已有的跨包 `motor_hardware_interface/config/motor.yaml` 引用允许通过 ament share 定位；其他缺失引用不会自动猜测替代文件。真实被解析文件的内容参与摘要。`resolve_reference(driver_file, reference)` 与 `load_model(driver_file)` 提供相同解析规则给 supervisor 共用；临时加载 selector 不改动原配置。

## 离线检查

先 source 包含 `robot_transmission` 的 ROS overlay。CLI 不启动 ROS 节点、不访问设备。

```bash
ros2 run robot_calibration calibration_check fingerprint \
  --driver-file /path/config/motor_driver.yaml --urdf-file /path/robot.urdf

ros2 run robot_calibration calibration_check template \
  --driver-file /path/config/motor_driver.yaml --urdf-file /path/robot.urdf \
  --model-name my_robot --robot-id assembly-001 --output /path/assembly.yaml

ros2 run robot_calibration calibration_check check \
  --driver-file /path/config/motor_driver.yaml --urdf-file /path/robot.urdf \
  --model-name my_robot --robot-id assembly-001 --manifest-file /path/assembly.yaml
```

`fingerprint` 只打印 SHA-256。`template` 独占创建输出文件，拒绝覆写；模板所有验收开关均为 false，MCU 总线索引留空。`check` 输出 JSON，有任何阻止运行的原因时退出码为 1。不得通过编辑 `commissioned` 或批量勾选开关冒充实际验收。

## 验收记录 schema_version 1

- `robot_id`、`model_name`：调用方选择的装配和模型，必须一致。
- `calibration_id`：非空的验收记录版本标识；`config_hash` 必须等于当前配置摘要。
- `commissioned`：只有实际完成验收后才能显式设为布尔 true。
- `acceptance.operator/date/evidence`：操作者、`YYYY-MM-DD` 验收日期及报告或测量数据位置。未来日期不被接受。
- `buses`：逻辑总线名到 MCU `bus_index`（0..2）的人工核对映射。索引不可重复，不根据串口顺序猜测。
- `motors`：必须恰好包含配置内全部执行器。每项记录 `motor_id`、`bus_index`、`protocol`（`vaxor` 或 `cerb`）、`model_name` 和 `identity_verified: true`。
- `joints`：必须恰好包含共享传动模型的全部活动关节。记录 `zero_offset_rad`、`lower_rad`、`upper_rad`、`velocity_limit_rad_s`，均须与当前配置一致；`zero_verified`、`direction_verified`、`limits_verified` 必须由验收确认。每关节还必须有非空 `evidence`。
- 每关节 `direction` 是其传动组中所有执行器的符号映射。共享传动库计算关节正向单位变化对应的执行器方向；验收记录必须一致。例如差动组中一个关节可能为 `{motor_a: 1, motor_b: 1}`，另一个为 `{motor_a: 1, motor_b: -1}`。模板展示配置期望值，`direction_verified: false` 表示尚未进行物理方向核对。

当前核验支持具有有限位置、速度和力矩限位的独立 `revolute`/`prismatic` 关节。未知插件、缺失文件、重复裸电机 ID、缺失关节、非有限数、无界关节和不完整验收记录均阻止运行。编码器自校准成功不等于装配零位及传动方向已验收。

线上 `MotorTopology` 目前没有设备序列号。`identity_verified` 是人工对该 ID、总线、协议和型号的验收声明；本包不能自动核验 SN，也不把任意字符串证据视为硬件安全认证。

## ROS 节点

```bash
ros2 run robot_calibration calibration_validator --ros-args \
  -p driver_file:=/path/config/motor_driver.yaml -p urdf_file:=/path/robot.urdf \
  -p manifest_file:=/path/assembly.yaml -p model_name:=my_robot -p robot_id:=assembly-001
```

每 0.5 秒重读文件，在 `/robot/calibration_state` 发布 `robot_system_msgs/CalibrationState`，QoS 为 reliable、transient local、depth 1。文件删除、配置变化、格式错误和验收缺失都发布 `valid=false` 及 `blockers`。该状态只有结合当前 MCU 会话、拓扑及新鲜健康反馈，才能由 supervisor 用于运行授权。

同时在标准 `/diagnostics` 话题发布 `diagnostic_msgs/DiagnosticArray`。`calibration_validator/commissioning` 的 OK 表示配置与人工验收记录通过，WARN 表示有阻止运行的原因；值中包含装配／模型标识、标定版本、配置摘要及原因。常态约 1 Hz，状态变化最多每 0.5 秒发布一次，可直接接入 `diagnostic_aggregator` 或 `rqt_robot_monitor`，不另建 JSON 诊断协议。

纯 Python API：

```python
from robot_calibration.validation import validate_configuration

report = validate_configuration(driver_file, urdf_file, manifest_file, model_name, robot_id)
print(report.asdict())
```
