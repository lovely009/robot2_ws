# storage_projection

## 审查结论

**职责符合；投影规则和空指针边界均有 host 测试。** 这是持久化字段语义的唯一归属，不负责 Flash 时序或任务调度。

## 职责与接口

- `storage_projection_views_to_blob()`：把内核参数视图写入 `param_store_blob_t`。
- `storage_projection_blob_to_boot()`：把 blob 记录恢复为内核启动参数。
- CERB 只落设备核验过的原值；位置上下限必须成对；未回读时退回记录原值。
- VAXOR 使用有限的本地软限位值；`enable` 不持久化。
- 文件中的 `_Static_assert` 锁住 `param_store` 设备槽序与 `motor_protocol` 参数对象序号。

## 依赖边界

依赖 `motor_control` 的参数视图/启动记录类型和 `param_store` 的 blob API；不 include Arduino、FreeRTOS、HAL，也不拥有 W25Q64。调用链是 `storage_task -> storage_projection -> (motor_control, param_store)`。

## 验证与发现

`test_storage_projection.cpp` 覆盖 CERB/VAXOR 投影、设备原值、安全/方向槽、成对限位、启动恢复、容量和 motor_id=0 拒绝；PlatformIO 固件构建通过。

两个公共函数在 `blob == NULL` 时安全返回；`views == NULL` 且 `count > 0` 时不写入，`out == NULL` 且 `capacity > 0` 时返回 0。空容量调用保持原有语义。`test_storage_projection.cpp` 已覆盖这些边界。

## 后续建议

调用方仍应传入有效且足够大的缓冲区；本库只检查空指针和容量组合，不负责验证内存区间或 blob CRC，后者由存储任务在调用前完成。
