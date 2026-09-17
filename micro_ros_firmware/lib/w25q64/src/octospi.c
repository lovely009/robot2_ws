#include "octospi.h"

#include <stdbool.h>
#include <string.h>

static OSPI_HandleTypeDef g_hospi2;
static bool g_initialized = false;

#define OCTOSPI2_TIMEOUT_MS 1000U

static octospi_status_t ospi_status(const OSPI_HandleTypeDef *handle,
                                   HAL_StatusTypeDef status) {
  if (status == HAL_TIMEOUT ||
      (handle != NULL &&
       (HAL_OSPI_GetError(handle) & HAL_OSPI_ERROR_TIMEOUT) != 0U)) {
    return OCTOSPI_ERROR_TIMEOUT;
  }
  return OCTOSPI_ERROR_HAL;
}

static octospi_status_t octospi2_configure_clock(void) {
  RCC_PeriphCLKInitTypeDef periph_clock = {0};
  periph_clock.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
  periph_clock.OspiClockSelection = RCC_OSPICLKSOURCE_D1HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clock) != HAL_OK) {
    return OCTOSPI_ERROR_HAL;
  }
  return OCTOSPI_OK;
}

static void octospi2_gpio_init(void) {
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gpio.Pin = GPIO_PIN_1;
  gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_3;
  gpio.Alternate = GPIO_AF6_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_0;
  gpio.Alternate = GPIO_AF4_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_2;
  gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_11;
  gpio.Alternate = GPIO_AF11_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOE, &gpio);

  gpio.Pin = GPIO_PIN_11;
  gpio.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOD, &gpio);
}

static void octospi2_gpio_deinit(void) {
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_3);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_2);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11);
}

octospi_status_t octospi2_init(void) {
  if (g_initialized) {
    return OCTOSPI_OK;
  }

  octospi_status_t status = octospi2_configure_clock();
  if (status != OCTOSPI_OK) {
    return status;
  }

  __HAL_RCC_OCTOSPIM_CLK_ENABLE();
  __HAL_RCC_OSPI2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  octospi2_gpio_init();

  memset(&g_hospi2, 0, sizeof(g_hospi2));
  g_hospi2.Instance = OCTOSPI2;
  g_hospi2.Init.FifoThreshold = 8U;
  g_hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  g_hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  g_hospi2.Init.DeviceSize = 23U; // 2^23 = 8 MiB，对应 W25Q64_FLASH_SIZE
  g_hospi2.Init.ChipSelectHighTime = 1U;
  g_hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  g_hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
  g_hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  g_hospi2.Init.ClockPrescaler = 3U;
  g_hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
  g_hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  g_hospi2.Init.ChipSelectBoundary = 0U;
  g_hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  g_hospi2.Init.MaxTran = 0U;
  g_hospi2.Init.Refresh = 0U;

  HAL_StatusTypeDef hal_status = HAL_OSPI_Init(&g_hospi2);
  if (hal_status != HAL_OK) {
    octospi_status_t status = ospi_status(&g_hospi2, hal_status);
    octospi2_deinit();
    return status;
  }

  OSPIM_CfgTypeDef manager = {0};
  manager.ClkPort = 1U;
  manager.NCSPort = 1U;
  manager.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  manager.Req2AckTime = 1U;
  hal_status = HAL_OSPIM_Config(&g_hospi2, &manager, OCTOSPI2_TIMEOUT_MS);
  if (hal_status != HAL_OK) {
    octospi_status_t status = ospi_status(&g_hospi2, hal_status);
    octospi2_deinit();
    return status;
  }

  g_initialized = true;
  return OCTOSPI_OK;
}

void octospi2_deinit(void) {
  if (g_hospi2.Instance == OCTOSPI2) {
    (void)HAL_OSPI_DeInit(&g_hospi2);
  }
  octospi2_gpio_deinit();
  __HAL_RCC_OSPI2_CLK_DISABLE();
  __HAL_RCC_OCTOSPIM_CLK_DISABLE();
  memset(&g_hospi2, 0, sizeof(g_hospi2));
  g_initialized = false;
}

OSPI_HandleTypeDef *octospi2_handle(void) {
  return g_initialized ? &g_hospi2 : NULL;
}
