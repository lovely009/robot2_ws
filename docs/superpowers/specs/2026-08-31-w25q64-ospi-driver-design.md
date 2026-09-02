# W25Q64 OCTOSPI2 驱动移植设计

## 目标

参考 `CtrBoard-H7_W25Q64` 例程，将 W25Q64JV 的 OCTOSPI2 Quad 驱动移入
`micro_ros_firmware/lib/w25q64`，保持现有 BMI088、FDCAN、USB 和 WS2812 功能，
并完成固件编译验证。此次只建立 Flash 底层驱动，不实现参数记录格式、磨损均衡或
持久化配置存储。

## 硬件映射

沿用官方例程的 OCTOSPI2 Port 1 Quad 接线：

- `PB2`：OCTOSPIM_P1_CLK，AF9
- `PE11`：OCTOSPIM_P1_NCS，AF11
- `PD11`：OCTOSPIM_P1_IO0，AF9
- `PB0`：OCTOSPIM_P1_IO1，AF4
- `PA3`：OCTOSPIM_P1_IO2，AF6
- `PA1`：OCTOSPIM_P1_IO3，AF9

BMI088 使用 SPI2：`PB13=SCK`、`PC1=MOSI`、`PC2=MISO`、`PC0=ACC_CS`、
`PC3=GYRO_CS`，与 OCTOSPI2 无引脚冲突。

## 库结构

新增：

```text
lib/w25q64/
├── library.json
├── include/
│   ├── octospi.h
│   └── w25q64.h
└── src/
    ├── octospi.c
    └── w25q64.c
```

文件以官方例程为来源，但移除 CubeMX `main.h`、全局 `Error_Handler()` 和生成器注释
依赖；直接包含 STM32H7 HAL 头文件。对外 C 接口支持 C++ 调用。

## OCTOSPI2 初始化

配置与例程保持一致：

- OCTOSPI2、OCTOSPIM 与所需 GPIO 时钟
- OSPI 时钟源 `D1HCLK`
- FIFO threshold 8
- device size 23（8 MiB）
- clock mode 3
- prescaler 3
- half-cycle sample shifting
- Port 1 low IO、Port 1 clock 与 NCS

初始化函数返回状态码，不调用 `Error_Handler()`，失败时允许上层降级运行。重复调用
必须幂等，不重复破坏已初始化的外设状态。

## W25Q64 API

保留并整理以下能力：

- reset enable / device reset
- JEDEC ID 读取和 `0xEF4017` 校验
- write enable 和 BUSY/WEL 自动轮询
- 4 KiB、32 KiB、64 KiB、整片擦除
- 256-byte page program
- 跨页 buffer write
- Quad IO buffer read
- memory-mapped read mode

所有地址和长度在发送 HAL 命令前校验，禁止越过 8 MiB 边界；页写禁止跨越单页。
输入指针为空且长度非零时返回参数错误。擦除地址按对应粒度对齐，否则返回参数错误。

轮询和 HAL 操作使用有界超时。整片擦除保留更长但有限的专用超时，不允许永久阻塞。

## 启动集成

`platformio.ini` 增加：

- `HAL_OSPI_MODULE_ENABLED`
- `ENABLE_W25Q64=1`

固件启动任务在其他业务任务创建前调用统一 `w25q64_init()`：初始化 OCTOSPI2、复位
Flash、等待就绪并检查 JEDEC ID。只执行非破坏性初始化和读 ID，不运行例程的擦除/
写入测试。

初始化结果保存在 GDB 可读诊断变量中。Flash 缺失或初始化失败不得阻止 control、
sensor 或 micro-ROS 任务启动；后续持久化功能通过该状态决定是否降级到 RAM/默认值。

`ENABLE_W25Q64=0` 时不初始化硬件，其他固件功能保持原状。

## 并发边界

本阶段只有启动任务访问 Flash，因此不引入锁。后续参数持久化加入运行期读写时，必须
由单一 storage task 拥有驱动，或在库外使用互斥量串行化；不得在 ROS executor 回调
或高优先级 control/sensor task 中执行擦除等待。

进入 memory-mapped 模式后不能直接执行间接写/擦除。后续业务若需要两种模式切换，
必须显式退出 memory-mapped 并重新初始化；本阶段启动流程不进入 memory-mapped。

## 测试与验证

### 主机测试

把地址、长度、页边界和擦除对齐校验提取为无 HAL 依赖的纯函数，覆盖：

- 首尾合法地址
- 越过 8 MiB 边界
- 零长度与空指针规则
- page program 跨页拒绝
- 4/32/64 KiB 擦除对齐

### 固件构建

- 新库被 PlatformIO 实际编译并链接
- `ENABLE_W25Q64=1` 默认构建成功
- `ENABLE_W25Q64=0` 构建成功
- 全部现有固件主机测试通过
- `git diff --check` 通过

### 真机边界

自动验证不执行擦除或写入。真机只允许检查初始化状态和 JEDEC ID；读写/掉电恢复测试
留待参数持久化格式设计完成后，在指定测试扇区执行。
