#ifndef PROJECT_STM32FREE_RTOS_CONFIG_H
#define PROJECT_STM32FREE_RTOS_CONFIG_H

// 项目级 FreeRTOS 配置入口：STM32FreeRTOS 的 FreeRTOSConfig.h 以 __has_include
// 优先包含本文件（完全替换库内 Default 路径）。先取库默认值，再按项目安全
// 要求覆盖；升级库时只需复查本文件的覆盖项。库内 Default.h 的部分宏无
// #ifndef 保护，且 extra 文件先于 Default 包含，故不能用 -D/extra 方式覆盖。
#include "FreeRTOSConfig_Default.h"

#ifdef configCHECK_FOR_STACK_OVERFLOW
#undef configCHECK_FOR_STACK_OVERFLOW
#endif
// 栈溢出检测 mode 2：每次上下文切换检查栈边界水印；命中时进应用侧
// vApplicationStackOverflowHook（main.cpp）。库默认 blink hook 因
// configCHECK_FOR_STACK_OVERFLOW_BLINK=0 不参与编译，无符号冲突。
#define configCHECK_FOR_STACK_OVERFLOW 2

#endif /* PROJECT_STM32FREE_RTOS_CONFIG_H */
