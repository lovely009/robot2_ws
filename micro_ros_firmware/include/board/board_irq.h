// 板级中断优先级共享常量（不依赖 FreeRTOS）：板级组件（lib/fdcan、src/dma.c）
// 不得反向 include RTOS 头，统一引用本常量做编译期合规断言。
// 必须与 FreeRTOS configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 一致；
// 反向断言在 main.cpp（同时可见两侧定义），漂移即编译失败。
#ifndef BOARD_IRQ_H
#define BOARD_IRQ_H

// 数值越大紧迫度越低；>= 此阈值的 ISR 不得调用 FreeRTOS FromISR API。
#define BOARD_ISR_SYSCALL_THRESHOLD 5U

#endif
