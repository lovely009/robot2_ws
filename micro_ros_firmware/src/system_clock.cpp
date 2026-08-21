// 覆盖 STM32duino 变体的 SystemClock_Config（WEAK 符号，此处强定义覆盖）。
//
// 背景：已安装的 framework-arduinoststm32 4.21200.0 中，H723V 变体的
// generic_clock.c 只配置了 HSI/PLL1 主时钟，未使能 HSI48、也未选择 USB 时钟源，
// 导致 USB CDC（PA11=USB_DM / PA12=USB_DP）无法工作。
// 本文件提供与官方 CtrBoard-H7 例程一致的 H723V 时钟配置：
//   - 主时钟：HSE 24MHz（板载晶振）→ PLL1 → 480 MHz SYSCLK（比 HSI 精确稳定）
//   - 使能 HSI48（48 MHz RC）作为 USB 内核时钟源（RCC_USBCLKSOURCE_HSI48）
//   - SPI6 内核时钟 = HSE 24MHz（WS2812 彩灯，官方例程同源）
//
// 链接说明：SystemClock_Config 在 stm32_def.h 中以 C 链接声明，故此处用
// extern "C" 定义；调用链 main() → init() → hw_config_init() → SystemClock_Config()。

#include "pins_arduino.h"

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};

  // 电源配置：LDO 供电 + 电压档位 0
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  // 振荡器：HSE（主时钟源，板载 24MHz 晶振）+ HSI48（USB 时钟）
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // 总线时钟：SYSCLK = PLL1P = 480 MHz，AHB/APB 分频与官方一致
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }

  // 外设时钟：USB 用 HSI48（48 MHz）、SPI6 用 HSE（24 MHz，WS2812）、
  // SPI2 用 PLL（BMI088，官方 IMU 例程同源）
  PeriphClkInitStruct.PeriphClockSelection =
      RCC_PERIPHCLK_USB | RCC_PERIPHCLK_SPI6 | RCC_PERIPHCLK_SPI123;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}
