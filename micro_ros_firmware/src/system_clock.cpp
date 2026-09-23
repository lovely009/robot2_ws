/*
 * DM-MC-Board02 / STM32H723VGT6 板级基础设施：时钟树 + 内存策略 + 时钟故障兜底。
 *
 *   HSE 24 MHz -> PLL1: M=2, N=40, P=1 => SYSCLK 480 MHz
 *                                  Q=2 => SPI1/2/3 内核时钟 240 MHz
 *   SYSCLK /2 => HCLK 240 MHz；APB1/2/3/4 /2 => 120 MHz
 *   HSE 24 MHz -> PLL2: M=2, N=40, Q=6 => FDCAN 内核时钟 80 MHz
 *   HSI48 48 MHz -> USB 内核时钟；CRS 以 USB1 SOF 同步 HSI48
 *   HSE 24 MHz -> SPI6 内核时钟（WS2812）
 *
 * 内存策略（MPU 非缓存区域）与故障兜底（CSS/NMI）都服务同一件事——
 * 「让时钟与 DMA 缓冲在受控前提下工作」，因此与时钟树同文件维护；
 * 区域与链接脚本的对应关系见下方 ConfigureMpuNonCacheRegions()。
 */

#include "pins_arduino.h"
#include "stm32h7xx_hal.h"

#ifndef DM_MC_EXPECTED_SYSCLK_HZ
#define DM_MC_EXPECTED_SYSCLK_HZ 480000000U
#endif

#ifndef DM_MC_EXPECTED_HSE_HZ
#define DM_MC_EXPECTED_HSE_HZ 24000000U
#endif

// stm32duino 将 F_CPU 定义为运行时变量 SystemCoreClock
// （SrcWrapper/inc/stm32_def.h），因此 SYSCLK 是否匹配无法用 static_assert 校验，
// 改为在 HAL 时钟配置完成后由下方的 VerifyCoreClocks() 强制检查。
// 晶振频率是编译期常量，在此处用静态断言钉死。
static_assert(HSE_VALUE == DM_MC_EXPECTED_HSE_HZ,
    "HSE_VALUE does not match the 24 MHz crystal on DM-MC-Board02");

namespace {

// 期望的派生时钟：与下方 PLL1 参数 / AHB / APB 分频一一对应，改动必须同步。
constexpr uint32_t kExpectedHclkHz = 240000000U;
constexpr uint32_t kExpectedPclk1Hz = 120000000U;
constexpr uint32_t kExpectedPclk2Hz = 120000000U;

[[noreturn]] void ClockFatal()
{
  Error_Handler();
  while (true) {
    __NOP();
  }
}

// MPU 非缓存区域：为 DMA/BDMA 缓冲池建立非缓存内存，并在此前后正确
// 保存/恢复 D-Cache 状态（Arduino 核心可能已开启 D-Cache）。
// 与链接脚本 stm32h723vgt6.ld 的对应关系（改动必须两处同步）：
//   D2 SRAM  0x30000000 / 32 KiB -> .dma_noinit （DMA1/DMA2 缓冲池）
//   D3 SRAM4 0x38000000 / 16 KiB -> .bdma_noinit（BDMA 缓冲池）
// 必须在任何 DMA 缓冲被 CPU 写入之前调用；缓冲地址与时钟无关，
// 由 SystemClock_Config() 在时钟树配置前调用即可。
void ConfigureMpuNonCacheRegions()
{
#if defined(__MPU_PRESENT) && (__MPU_PRESENT == 1U)
  const bool dcache_was_enabled = (SCB->CCR & SCB_CCR_DC_Msk) != 0U;

  if (dcache_was_enabled) {
    SCB_CleanInvalidateDCache();
    SCB_DisableDCache();
  }

  HAL_MPU_Disable();

  MPU_Region_InitTypeDef region = {};
  region.Enable = MPU_REGION_ENABLE;
  region.AccessPermission = MPU_REGION_FULL_ACCESS;
  region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable = MPU_ACCESS_SHAREABLE;
  region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  region.TypeExtField = MPU_TEX_LEVEL1;  // Normal 内存类型、非缓存。
  region.SubRegionDisable = 0x00U;

  // D2 SRAM1+2：DMA1/DMA2 缓冲池（后续 SPI/485 等新 DMA 消费者使用）。
  region.Number = MPU_REGION_NUMBER6;
  region.BaseAddress = 0x30000000U;
  region.Size = MPU_REGION_SIZE_32KB;
  HAL_MPU_ConfigRegion(&region);

  // D3 SRAM4：BDMA 缓冲池。
  region.Number = MPU_REGION_NUMBER7;
  region.BaseAddress = 0x38000000U;
  region.Size = MPU_REGION_SIZE_16KB;
  HAL_MPU_ConfigRegion(&region);

  // 显式区域之外，特权代码继续使用架构默认内存映射。
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

  if (dcache_was_enabled) {
    SCB_EnableDCache();
  }
#endif
}

void ConfigureUsbCrs()
{
#if defined(CRS) && defined(RCC_CRS_SYNC_SOURCE_USB1)
  __HAL_RCC_CRS_CLK_ENABLE();

  RCC_CRSInitTypeDef crs = {};
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB1;  // STM32H723 的 USB1 即 USB_OTG_HS。
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;  // 目标 48 MHz / 以 1 kHz 的 USB SOF 为参考。
  crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&crs);
#endif
}

void VerifyCoreClocks()
{
  SystemCoreClockUpdate();

  if (SystemCoreClock != DM_MC_EXPECTED_SYSCLK_HZ ||
      HAL_RCC_GetHCLKFreq() != kExpectedHclkHz ||
      HAL_RCC_GetPCLK1Freq() != kExpectedPclk1Hz ||
      HAL_RCC_GetPCLK2Freq() != kExpectedPclk2Hz) {
    ClockFatal();
  }
}

}  // namespace

extern "C" void SystemClock_Config(void)
{
  // DMA 缓冲区在被 CPU 写入之前必须已建立非缓存 MPU 区域；Arduino 核心可能
  // 已开启 D-cache，ConfigureMpuNonCacheRegions() 会保存并恢复该状态。
  ConfigureMpuNonCacheRegions();

  RCC_OscInitTypeDef osc = {};
  RCC_ClkInitTypeDef clk = {};
  RCC_PeriphCLKInitTypeDef periph = {};

  // 板卡使用 MCU 内部 LDO 供电路径。以 VOS0 运行 480 MHz。
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) {
    ClockFatal();
  }

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  // 若电源调压迟迟未就绪，避免陷入无声死循环。
  uint32_t vos_guard = 0x00FFFFFFU;
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    if (vos_guard-- == 0U) {
      ClockFatal();
    }
  }

  // HSE：24 MHz 晶振。HSI48：USB FS 内核时钟。
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  osc.HSEState = RCC_HSE_ON;
  osc.HSI48State = RCC_HSI48_ON;

  // PLL1：24 / 2 * 40 = 480 MHz VCO；P=1 => 480 MHz SYSCLK；Q=2 => 240 MHz。
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 2;
  osc.PLL.PLLN = 40;
  osc.PLL.PLLP = 1;
  osc.PLL.PLLQ = 2;
  osc.PLL.PLLR = 2;
  osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;  // 12 MHz 输入位于 8..16 MHz 区间。
  osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  osc.PLL.PLLFRACN = 0;

  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    ClockFatal();
  }

  // CPU 480 MHz；AHB/AXI 240 MHz；APB1/2/3/4 120 MHz。
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
      RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider = RCC_HCLK_DIV2;
  clk.APB3CLKDivider = RCC_APB3_DIV2;
  clk.APB1CLKDivider = RCC_APB1_DIV2;
  clk.APB2CLKDivider = RCC_APB2_DIV2;
  clk.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) {
    ClockFatal();
  }

  // 外设内核时钟：
  //   USB      <- HSI48（随后由 CRS 锁定到 USB SOF）
  //   SPI6     <- HSE 24 MHz（WS2812）
  //   SPI1/2/3 <- PLL1Q 240 MHz（BMI088 / 后续 SPI 扩展）
  //   FDCAN    <- PLL2Q 80 MHz
  periph.PeriphClockSelection =
      RCC_PERIPHCLK_USB | RCC_PERIPHCLK_SPI6 |
      RCC_PERIPHCLK_SPI123 | RCC_PERIPHCLK_FDCAN;

  periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  periph.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
  periph.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  periph.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;

  // PLL2：24 / 2 * 40 = 480 MHz VCO；Q=6 => 80 MHz FDCAN。
  periph.PLL2.PLL2M = 2;
  periph.PLL2.PLL2N = 40;
  periph.PLL2.PLL2P = 2;
  periph.PLL2.PLL2Q = 6;
  periph.PLL2.PLL2R = 2;
  periph.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  periph.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  periph.PLL2.PLL2FRACN = 0;

  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
    ClockFatal();
  }

  ConfigureUsbCrs();
  VerifyCoreClocks();

  // 使能 HSE 失效检测（CSS），其时钟安全中断路由到 NMI：
  // 处理与安全停机钩子见本文件末尾的故障兜底段。
  HAL_RCC_EnableCSS();
}

// ---------------------------------------------------------------------------
// 时钟故障兜底：时钟安全中断（CSS）与 NMI 的落地处理。
// ---------------------------------------------------------------------------

// 弱定义：电机控制等应用应提供**强定义**覆盖它，且只使用在 HSE/PLL 失效时
// 仍确认安全的操作（立即切断驱动输出/电源使能）。默认实现为空——即直接
// 复位，这是有意保守的做法；在安全停机路径完备之前不要假定有更聪明的兜底。
extern "C" __attribute__((weak)) void Board_ClockFaultSafeStop(void) {
}

extern "C" void HAL_RCC_CSSCallback(void) {
  // HSE 失效：先让应用切断危险输出，再复位。
  Board_ClockFaultSafeStop();
  NVIC_SystemReset();
}

extern "C" void NMI_Handler(void) {
  // CSS 触发的 NMI 交给 HAL 统一清标志并回调 HAL_RCC_CSSCallback。
  if (__HAL_RCC_GET_IT(RCC_IT_CSS)) {
    HAL_RCC_NMI_IRQHandler();
    return;
  }

  // 未知 NMI 来源：宁可故障关断，也不在状态不明时继续运行。
  Board_ClockFaultSafeStop();
  NVIC_SystemReset();
}
