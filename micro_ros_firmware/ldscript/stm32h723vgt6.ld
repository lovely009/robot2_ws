/*
 * DM-MC-Board02 / STM32H723VGT6 链接脚本
 *
 * 本脚本采用的默认 SRAM 布局：
 *   ITCM     64 KiB  @ 0x00000000  （已声明，启动代码不会自动加载代码到此处）
 *   DTCM    128 KiB  @ 0x20000000  （栈/堆 + 显式 DTCM 对象）
 *   AXI SRAM 320 KiB @ 0x24000000  （.data/.bss/.noinit）
 *   SRAM1+2  32 KiB  @ 0x30000000  （DMA1/DMA2 缓冲区）
 *   SRAM4    16 KiB  @ 0x38000000  （BDMA 缓冲区）
 *
 * 4 KiB 备份 SRAM 有意不纳入常规分配器。
 *
 * 注意：
 * - 板级定义会开启 D-cache，因此 DMA/BDMA 缓冲区需要相应地
 *   做缓存维护，或放入 MPU 非缓存区域。
 * - MPU 非缓存区域在 src/board/system_clock.cpp 的 ConfigureMpuNonCacheRegions()
 *   中按本文件的 RAM_D2（SRAM1+2）/ RAM_D3（SRAM4）划分设置；
 *   改动下方 ORIGIN/LENGTH 必须同步改该函数的基址与大小。
 * - ITCM 代码放置需要启动侧的拷贝逻辑；本脚本不会静默把代码放进 ITCM。
 */

ENTRY(Reset_Handler)

_Min_Heap_Size  = 0x1000; /* 最小 4 KiB */
_Min_Stack_Size = 0x2000; /* 最小 8 KiB */

MEMORY
{
  ITCMRAM (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM_D1  (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
  RAM_D2  (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
  RAM_D3  (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
}

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
__StackTop = _estack;

/* 这些区域符号供 MPU/缓存配置与诊断使用。 */
__itcm_start__  = ORIGIN(ITCMRAM);
__itcm_end__    = ORIGIN(ITCMRAM) + LENGTH(ITCMRAM);
__dtcm_start__  = ORIGIN(DTCMRAM);
__dtcm_end__    = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
__ram_d1_start__ = ORIGIN(RAM_D1);
__ram_d1_end__   = ORIGIN(RAM_D1) + LENGTH(RAM_D1);
__ram_d2_start__ = ORIGIN(RAM_D2);
__ram_d2_end__   = ORIGIN(RAM_D2) + LENGTH(RAM_D2);
__ram_d3_start__ = ORIGIN(RAM_D3);
__ram_d3_end__   = ORIGIN(RAM_D3) + LENGTH(RAM_D3);

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab (READONLY) :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM (READONLY) :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH

  .preinit_array (READONLY) :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH

  .init_array (READONLY) :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH

  .fini_array (READONLY) :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } > RAM_D1 AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM_D1

  /* 软复位后内容保持不变的通用 RAM；需在代码中显式初始化。 */
  .noinit (NOLOAD) :
  {
    . = ALIGN(8);
    __noinit_start__ = .;
    *(.noinit)
    *(.noinit.*)
    . = ALIGN(8);
    __noinit_end__ = .;
  } > RAM_D1

  /* 适配 DMA1/DMA2 的缓冲池。32 字节对齐对应 Cortex-M7 的 D-cache 行大小。 */
  .dma_noinit (NOLOAD) :
  {
    . = ALIGN(32);
    __dma_noinit_start__ = .;
    *(.dma_noinit)
    *(.dma_noinit.*)
    . = ALIGN(32);
    __dma_noinit_end__ = .;
  } > RAM_D2

  /* SRAM4（D3）中可供 BDMA 访问的缓冲池。 */
  .bdma_noinit (NOLOAD) :
  {
    . = ALIGN(32);
    __bdma_noinit_start__ = .;
    *(.bdma_noinit)
    *(.bdma_noinit.*)
    . = ALIGN(32);
    __bdma_noinit_end__ = .;
  } > RAM_D3

  /* 可选的显式 DTCM 对象。不要把 DMA 缓冲区放在这里。 */
  .dtcm_noinit (NOLOAD) :
  {
    . = ALIGN(8);
    __dtcm_noinit_start__ = .;
    *(.dtcm_noinit)
    *(.dtcm_noinit.*)
    . = ALIGN(8);
    __dtcm_noinit_end__ = .;
  } > DTCMRAM

  /*
   * 将堆 + 主栈放在 DTCM，使常规的 _end/_estack 模型对
   * newlib/Arduino 保持自洽。运行时向上增长的堆与向下生长的
   * 主栈共享剩余的 DTCM 空间。
   */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    __HeapBase = .;
    . = . + _Min_Heap_Size;
    __HeapMinLimit = .;
    . = . + _Min_Stack_Size;
    __StackMinLimit = .;
    . = ALIGN(8);
  } > DTCMRAM

  /* 保留编译器属性段；无用库代码由 --gc-sections 剔除。 */
  .ARM.attributes 0 :
  {
    *(.ARM.attributes)
  }

  ASSERT(__StackMinLimit <= __StackTop, "DTCM overflow: static DTCM + minimum heap/stack exceed 128 KiB")
  ASSERT(SIZEOF(.dma_noinit) <= LENGTH(RAM_D2), "RAM_D2 DMA pool overflow")
  ASSERT(SIZEOF(.bdma_noinit) <= LENGTH(RAM_D3), "RAM_D3 BDMA pool overflow")
}
