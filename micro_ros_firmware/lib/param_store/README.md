# param_store

## 审查结论

**符合纯参数存储库定义；v3 有意保留未实现字段。** 本库不访问 W25Q64、不决定保存时序，也不参与运动控制策略。

## 职责与边界

- `param_field.*`：参数字段枚举、设备原值槽序和值域校验。
- `param_units.h`：N·m/A 的边界换算纯函数。
- `param_store_blob.*`：v3 固定布局、present 位图、CRC32、记录读写和 v1/v2 迁移。
- Flash 擦写、A/B 槽选择和任务通知由 `src/storage_task.cpp` 与 `w25q64` 负责。

## 文件与接口

公共接口包括 `param_value_ok()`、`param_position_limits_ok()`、`param_store_blob_reset()`、`param_store_blob_validate()`、`param_store_blob_set/get()`、设备原值读写和 `param_store_blob_migrate_v1/v2()`。

v3 当前布局是 32B 头 + 64×120B 记录；记录内的 `model_name`、`identity_hash`、MIT 标度和 `profile_version` 是预留字段，写入路径尚未实现，值为零不能解释为已核验身份。

## 依赖与安全约束

- 仅依赖 C 标准库；C++ 通过 `extern "C"` 暴露函数。
- blob 缓冲区由调用方提供，生产路径使用满足对齐要求的静态存储。
- `enable` 不持久化；`limit_protect` 和方向使用 `device_raw` 槽，不能误当作普通 float 字段。
- `param_store_blob_finalize()` 是落盘前重算 CRC 的最后一步。

## 验证与缺口

`test_param_store.cpp` 覆盖字段值域、v3 布局、CRC、容量、设备原值、版本迁移和单位换算；PlatformIO 固件构建通过。实际 Flash 读写由 `w25q64` 和 storage 任务承担，不在本库 host 测试中重复模拟。

## 后续建议

新增字段必须同步更新枚举顺序、位图语义、记录大小/迁移和 `storage_projection` 的跨库静态断言；不要把预留字段写成“已保存”。
