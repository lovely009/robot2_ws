# w25q64

## 审查结论

**硬件驱动分层符合；host 目前只覆盖 geometry。** `storage_task` 是唯一所有者，库不理解参数字段、保存命令或双槽选择策略。

## 内部结构

| 文件 | 作用 |
|---|---|
| `include/octospi.h` / `src/octospi.c` | OCTOSPI2 时钟、GPIO、句柄和间接传输控制层 |
| `include/w25q64.h` / `src/w25q64.c` | JEDEC、reset、读、页编程、跨页写和扇区擦除命令层 |
| `include/w25q64_geometry.h` / `src/w25q64_geometry.c` | Flash 范围、页边界和擦除地址纯逻辑 |

公共操作是 `w25q64_init/read/write/program_page/erase_sector()`；状态码区分参数、HAL、超时、ID 和状态错误。`octospi2_*` 只属于该库内部的控制器层。

## 依赖与所有权

- 依赖 STM32 HAL 和 OCTOSPI2；geometry 可独立 host 编译。
- 地址、长度单位为字节；页编程不得跨页，`w25q64_write()` 负责拆分。
- `storage_task` 在 boot 初始化并独占所有读写；其他任务不得直接访问。
- 库不决定 A/B 槽、CRC、seq 或保存触发条件，这些归 `param_store`/storage 任务。

## 验证与缺口

`test_w25q64_geometry.cpp` 覆盖范围、页和擦除边界；PlatformIO 固件构建通过。`octospi.c`、命令时序、JEDEC ID、擦写超时和编程后读回只能在目标板/Flash 上验证，当前没有 host HAL 仿真。

## 后续建议

台架验收应记录 JEDEC ID、读写回读、跨页写、擦除耗时和 HAL 超时映射；不要在 storage 任务之外增加第二个调用者。
