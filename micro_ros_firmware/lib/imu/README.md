# imu

## 审查结论

**职责符合；纯逻辑可 host 测试，SPI/HAL 路径只能目标板验证。** 该库集中拥有 BMI088、SPI2 阻塞传输、Mahony 融合和 IMU 双缓冲，不承担 ROS 发布。

## 职责与边界

- `bmi088_driver.*`：厂商寄存器驱动和原始量读取。
- `bmi088_middleware.*`、`imu_spi2_transport.*`：SPI2、片选、延时和 HAL 传输错误适配。
- `imu.c`、`MahonyAHRS.c`：采样更新、SI 单位转换和姿态融合。
- `imu_snapshot.*`：固定双缓冲快照。
- 不使用 DMA；不依赖电机、ROS 或消息桥接。

## 文件与接口

公共入口包括 `imu_try_init()`、`imu_update()`、`imu_get_data()`、`imu_ready()`、`imu_snapshot_*()` 和 `BMI088_transport_error_*()`。厂商 API（如 `BMI088_init()`、`BMI088_read()`）保留原名，以便追溯数据手册和原始例程。

## 所有权与实时约束

- `sensor_task` 是初始化和采样的唯一调用者；`imu_bridge` 只读取发布快照。
- SPI2 是阻塞传输，不能放进电机实时任务；传输失败锁存到当前操作并由任务重试。
- `imu_data_t` 对外使用 rad/s、m/s²、°C 和四元数；库内不创建 ROS 消息。

## 验证与缺口

- `test_imu_snapshot.cpp`：双缓冲发布/复制。
- `test_imu_units.cpp`：SI 单位和 Mahony 聚合。
- `test_bmi088_transport_status.cpp`、`test_bmi088_register_config.cpp`：错误锁存和寄存器配置。
- PlatformIO 固件构建通过。

host 没有编译 `bmi088_middleware.c`、`imu_spi2_transport.c` 和完整 BMI088 读写；GPIO、FreeRTOS 延时、SPI 时序和 EXTI 唤醒仍需台架验证。`bmi088_driver` 的厂商拼写是兼容性约束，不应在文档审查中误判为新 API 设计。

## 后续建议

保持传感器任务单一所有者；若修改 SPI 速率、ODR 或融合采样频率，应同时更新 `platformio.ini`/系统时钟说明和 host 单位测试。
