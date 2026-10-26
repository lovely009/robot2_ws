// W25Q64 命令层：JEDEC 识别、读、页编、扇区擦与编程后读回校验，经 octospi
// 控制器层下发。唯一调用者是 storage 任务；地址/长度单位均为字节。
#include "w25q64.h"
#include "octospi.h"

// octospi 层状态枚举与本层数值对齐：跨层返回值依赖等值，编译期锁死。
_Static_assert((int)W25Q64_ERROR_HAL == (int)OCTOSPI_ERROR_HAL,
    "W25Q64/octospi 错误码 HAL 必须等值");
_Static_assert((int)W25Q64_ERROR_TIMEOUT == (int)OCTOSPI_ERROR_TIMEOUT,
    "W25Q64/octospi 错误码 TIMEOUT 必须等值");
_Static_assert((int)W25Q64_ERROR_STATE == (int)OCTOSPI_ERROR_STATE,
    "W25Q64/octospi 错误码 STATE 必须等值");

#define W25Q64_CMD_ENABLE_RESET 0x66U
#define W25Q64_CMD_RESET_DEVICE 0x99U
#define W25Q64_CMD_READ_JEDEC_ID 0x9FU
#define W25Q64_CMD_WRITE_ENABLE 0x06U
#define W25Q64_CMD_SECTOR_ERASE 0x20U
#define W25Q64_CMD_QUAD_INPUT_PAGE_PROGRAM 0x32U
#define W25Q64_CMD_FAST_READ_QUAD_IO 0xEBU
#define W25Q64_CMD_READ_STATUS_REG1 0x05U

#define W25Q64_STATUS_BUSY 0x01U
#define W25Q64_STATUS_WEL 0x02U
#define W25Q64_HAL_TIMEOUT_MS 1000U
#define W25Q64_SECTOR_ERASE_TIMEOUT_MS 1000U

static w25q64_status_t w25q64_hal_status(const OSPI_HandleTypeDef *handle,
    HAL_StatusTypeDef status) {
  if (status == HAL_TIMEOUT ||
      (handle != NULL &&
      (HAL_OSPI_GetError(handle) & HAL_OSPI_ERROR_TIMEOUT) != 0U)) {
    return W25Q64_ERROR_TIMEOUT;
  }
  return W25Q64_ERROR_HAL;
}

// HAL 状态 → 本层状态：超时单独成一类（上层据此区分可重试与否）。
static w25q64_status_t w25q64_map_status(OSPI_HandleTypeDef *handle,
    HAL_StatusTypeDef hal_status) {
  return hal_status == HAL_OK ? W25Q64_OK : w25q64_hal_status(handle, hal_status);
}

// 下发一条常规命令（超时统一 W25Q64_HAL_TIMEOUT_MS），返回值已是本层状态。
static w25q64_status_t w25q64_send_command(OSPI_HandleTypeDef *handle,
    OSPI_RegularCmdTypeDef *command) {
  return w25q64_map_status(
      handle, HAL_OSPI_Command(handle, command, W25Q64_HAL_TIMEOUT_MS));
}

static OSPI_HandleTypeDef *w25q64_handle(void) {
  return octospi2_handle();
}

static void w25q64_command_init(OSPI_RegularCmdTypeDef *command,
    uint32_t instruction) {
  *command = (OSPI_RegularCmdTypeDef){0};
  command->OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  command->FlashId = HAL_OSPI_FLASH_ID_1;
  command->Instruction = instruction;
  command->InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
  command->InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  command->InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  command->AddressMode = HAL_OSPI_ADDRESS_NONE;
  command->AddressSize = HAL_OSPI_ADDRESS_24_BITS;
  command->AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  command->AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  command->AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  command->DataMode = HAL_OSPI_DATA_NONE;
  command->DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
  command->DQSMode = HAL_OSPI_DQS_DISABLE;
  command->SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
}

static w25q64_status_t w25q64_wait_ready(uint32_t timeout_ms) {
  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return W25Q64_ERROR_STATE;
  }

  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, W25Q64_CMD_READ_STATUS_REG1);
  command.DataMode = HAL_OSPI_DATA_1_LINE;
  command.NbData = 1U;
  w25q64_status_t status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }

  OSPI_AutoPollingTypeDef polling = {0};
  polling.Match = 0U;
  polling.Mask = W25Q64_STATUS_BUSY;
  polling.MatchMode = HAL_OSPI_MATCH_MODE_AND;
  polling.Interval = 0x10U;
  polling.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
  return w25q64_map_status(handle,
      HAL_OSPI_AutoPolling(handle, &polling, timeout_ms));
}

static w25q64_status_t w25q64_write_enable(void) {
  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return W25Q64_ERROR_STATE;
  }

  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, W25Q64_CMD_WRITE_ENABLE);
  w25q64_status_t status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }

  w25q64_command_init(&command, W25Q64_CMD_READ_STATUS_REG1);
  command.DataMode = HAL_OSPI_DATA_1_LINE;
  command.NbData = 1U;
  status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }

  OSPI_AutoPollingTypeDef polling = {0};
  polling.Match = W25Q64_STATUS_WEL;
  polling.Mask = W25Q64_STATUS_WEL;
  polling.MatchMode = HAL_OSPI_MATCH_MODE_AND;
  polling.Interval = 0x10U;
  polling.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;
  return w25q64_map_status(
      handle, HAL_OSPI_AutoPolling(handle, &polling, W25Q64_HAL_TIMEOUT_MS));
}

static w25q64_status_t w25q64_erase(uint32_t address, uint32_t erase_size,
    uint32_t instruction,
    uint32_t timeout_ms) {
  if (!w25q64_erase_address_valid(address, erase_size)) {
    return W25Q64_ERROR_ARGUMENT;
  }

  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return W25Q64_ERROR_STATE;
  }
  w25q64_status_t status = w25q64_write_enable();
  if (status != W25Q64_OK) {
    return status;
  }

  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, instruction);
  command.Address = address;
  command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }
  return w25q64_wait_ready(timeout_ms);
}

w25q64_status_t w25q64_init(void) {
  w25q64_status_t status = octospi2_init();
  if (status != W25Q64_OK) {
    return status;
  }
  status = w25q64_reset();
  if (status != W25Q64_OK) {
    return status;
  }
  status = w25q64_wait_ready(W25Q64_HAL_TIMEOUT_MS);
  if (status != W25Q64_OK) {
    return status;
  }
  return w25q64_read_id() == W25Q64_JEDEC_ID ? W25Q64_OK : W25Q64_ERROR_ID;
}

w25q64_status_t w25q64_reset(void) {
  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return W25Q64_ERROR_STATE;
  }

  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, W25Q64_CMD_ENABLE_RESET);
  w25q64_status_t status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }
  w25q64_command_init(&command, W25Q64_CMD_RESET_DEVICE);
  return w25q64_send_command(handle, &command);
}

uint32_t w25q64_read_id(void) {
  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return 0U;
  }

  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, W25Q64_CMD_READ_JEDEC_ID);
  command.DataMode = HAL_OSPI_DATA_1_LINE;
  command.NbData = 3U;
  if (HAL_OSPI_Command(handle, &command, W25Q64_HAL_TIMEOUT_MS) != HAL_OK) {
    return 0U;
  }
  uint8_t id[3] = {0};
  if (HAL_OSPI_Receive(handle, id, W25Q64_HAL_TIMEOUT_MS) != HAL_OK) {
    return 0U;
  }
  return ((uint32_t)id[0] << 16U) | ((uint32_t)id[1] << 8U) | id[2];
}

w25q64_status_t w25q64_read(uint32_t address, uint8_t *data, uint32_t size) {
  if (data == NULL && size != 0U) {
    return W25Q64_ERROR_ARGUMENT;
  }
  if (!w25q64_range_valid(address, size)) {
    return W25Q64_ERROR_ARGUMENT;
  }
  if (size == 0U) {
    return W25Q64_OK;
  }

  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return W25Q64_ERROR_STATE;
  }
  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, W25Q64_CMD_FAST_READ_QUAD_IO);
  command.Address = address;
  command.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
  command.DataMode = HAL_OSPI_DATA_4_LINES;
  command.NbData = size;
  command.DummyCycles = 6U;
  w25q64_status_t status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }
  return w25q64_map_status(
      handle, HAL_OSPI_Receive(handle, data, W25Q64_HAL_TIMEOUT_MS));
}

w25q64_status_t w25q64_program_page(uint32_t address, const uint8_t *data,
    uint32_t size) {
  if (data == NULL && size != 0U) {
    return W25Q64_ERROR_ARGUMENT;
  }
  if (!w25q64_page_program_valid(address, size)) {
    return W25Q64_ERROR_ARGUMENT;
  }

  OSPI_HandleTypeDef *handle = w25q64_handle();
  if (handle == NULL) {
    return W25Q64_ERROR_STATE;
  }
  w25q64_status_t status = w25q64_write_enable();
  if (status != W25Q64_OK) {
    return status;
  }

  OSPI_RegularCmdTypeDef command;
  w25q64_command_init(&command, W25Q64_CMD_QUAD_INPUT_PAGE_PROGRAM);
  command.Address = address;
  command.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  command.DataMode = HAL_OSPI_DATA_4_LINES;
  command.NbData = size;
  status = w25q64_send_command(handle, &command);
  if (status != W25Q64_OK) {
    return status;
  }
  status = w25q64_map_status(
      handle, HAL_OSPI_Transmit(handle, (uint8_t *)data, W25Q64_HAL_TIMEOUT_MS));
  if (status != W25Q64_OK) {
    return status;
  }
  return w25q64_wait_ready(W25Q64_HAL_TIMEOUT_MS);
}

w25q64_status_t w25q64_write(uint32_t address, const uint8_t *data,
    uint32_t size) {
  if (data == NULL && size != 0U) {
    return W25Q64_ERROR_ARGUMENT;
  }
  if (!w25q64_range_valid(address, size)) {
    return W25Q64_ERROR_ARGUMENT;
  }
  if (size == 0U) {
    return W25Q64_OK;
  }

  uint32_t current_address = address;
  const uint8_t *current_data = data;
  uint32_t remaining = size;
  // 页编程不可跨页（跨页地址回卷到页首）：逐页拆分写入。
  while (remaining != 0U) {
    uint32_t page_remaining =
        W25Q64_PAGE_SIZE - (current_address % W25Q64_PAGE_SIZE);
    uint32_t chunk = remaining < page_remaining ? remaining : page_remaining;
    w25q64_status_t status =
        w25q64_program_page(current_address, current_data, chunk);
    if (status != W25Q64_OK) {
      return status;
    }
    current_address += chunk;
    current_data += chunk;
    remaining -= chunk;
  }
  return W25Q64_OK;
}

w25q64_status_t w25q64_erase_sector(uint32_t address) {
  return w25q64_erase(address, W25Q64_SECTOR_SIZE, W25Q64_CMD_SECTOR_ERASE,
      W25Q64_SECTOR_ERASE_TIMEOUT_MS);
}
