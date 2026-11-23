// SPI 对端替身只模拟芯片寄存器和 FIFO。初始化、帧编解码及错误处理均跑实际驱动。
#include "can4_spi.h"
#include "mcp2518fd.h"
#include <assert.h>
#include <string.h>

static uint8_t chip[4096];
static bool connected, corrupt_once, int_connected = true, int_stuck_low;
static bool unplug_on_ram_write;
static unsigned resets, normal_tx, rx_pending;

static void put32(unsigned reg, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) chip[reg + i] = value >> (8 * i);
}

static unsigned fifo_ua(unsigned reg) {
  return chip[reg] | ((unsigned)chip[reg + 1] << 8);
}

static void reset_fifos(void) {
  put32(0x64, 0);
  put32(0x70, 8U * 72U);
  rx_pending = 0;
  chip[0x60] = 1;
  chip[0x6c] = 0;
}

// 微芯 CRC-16 的逐位参考算法（多项式 x^16+x^15+x^2+1）。
static uint16_t crc(const uint8_t *bytes, unsigned size) {
  unsigned value = 0xffff;
  for (unsigned i = 0; i < size; ++i) {
    for (unsigned mask = 128; mask; mask >>= 1) {
      bool feedback = !!(value & 0x8000) != !!(bytes[i] & mask);
      value <<= 1;
      if (feedback) value ^= 0x8005;
    }
  }
  return value;
}

static void reset_registers(void) {
  memset(chip, 0, sizeof(chip));
  chip[2] = 4U << 5;
  chip[3] = 4U;
  put32(0xe00, 0x460); // OSCRDY、CLKODIV
  chip[0xe14] = 0x01;
  reset_fifos();
}

bool can4_spi_init(void) { return true; }
void can4_init_delay_ms(uint32_t ms) { (void)ms; }
bool can4_irq_asserted(void) {
  return int_stuck_low || (int_connected && (chip[0x6c] & 9U) &&
      (chip[0x68] & 1U) && (chip[0x1e] & 2U));
}

bool can4_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t length, bool may_block) {
  unsigned addr = ((tx[0] & 15U) << 8) | tx[1];
  assert(length <= CAN4_SPI_MAX_BYTES);
  assert(may_block == (tx[0] >> 4 == 11 && addr >= 0x400 && addr < 0xc00));
  memset(rx, 0xff, length);
  if (!connected) return true; // 拔掉模块时 H7 传输本身仍可成功
  switch (tx[0] >> 4) {
  case 0:
    assert(length == 2 && addr == 0 && (chip[2] >> 5) == 4);
    ++resets;
    reset_registers();
    return true;
  case 2:
    assert(addr + length - 2 <= sizeof(chip));
    if (addr >= 0x400 && addr < 0xc00 && unplug_on_ram_write) {
      connected = false;
      return true;
    }
    memcpy(chip + addr, tx + 2, length - 2);
    if (addr <= 3 && addr + length - 2 > 3) {
      chip[2] = (chip[2] & 31U) | ((chip[3] & 7U) << 5);
      if ((chip[3] & 7U) == 4) reset_fifos();
    }
    if (addr == 0x5d && (tx[2] & 3U) == 3U) {
      unsigned head = fifo_ua(0x64);
      if ((chip[2] >> 5) == 2) {
        assert(chip[0x1d0] == 0x82);
        assert(chip[0x404 + head] == 0xcf); // 64 字节 FD+BRS；启动回环必须覆盖长写和长读
        memcpy(chip + 0x400 + fifo_ua(0x70), chip + 0x400 + head, 72);
        rx_pending = 1;
        chip[0x6c] |= 1;
      } else {
        assert((chip[2] >> 5) == 0);
        ++normal_tx;
      }
      put32(0x64, (head + 72U) % (8U * 72U));
    }
    if (addr == 0x6c && rx_pending) chip[0x6c] |= 1;
    if (addr == 0x69 && (tx[2] & 1U)) {
      assert(rx_pending);
      put32(0x70, 8U * 72U + (fifo_ua(0x70) - 8U * 72U + 72U) % (16U * 72U));
      if (--rx_pending == 0) chip[0x6c] &= ~1U;
    }
    return true;
  case 11: {
    unsigned size = (addr >= 0x400 && addr < 0xc00) ? 4U * tx[2] : tx[2];
    assert(length == size + 5 && addr + size <= sizeof(chip));
    memcpy(rx, tx, 3);
    memcpy(rx + 3, chip + addr, size);
    uint16_t expected = crc(rx, size + 3);
    rx[size + 3] = expected >> 8;
    rx[size + 4] = expected ^ corrupt_once;
    corrupt_once = false;
    return true;
  }
  default: assert(false); return false;
  }
}

int main(void) {
  reset_registers();
  assert(!mcp2518fd_init() && !mcp2518fd_ready());
  connected = true;
  chip[2] = 0; // 只复位了 MCU，外部控制器仍处在正常模式
  corrupt_once = true;
  assert(mcp2518fd_init() && resets == 1 && normal_tx == 0);
  assert(mcp2518fd_ready() && (chip[2] >> 5) == 0);
  assert(chip[6] == 30 && chip[5] == 7 && chip[10] == 4 && chip[9] == 1);
  can_tx_frame_t frame = {.can_id = 0x1abcdef1, .is_extended = true,
                          .is_fd = true, .is_brs = true, .size = 64};
  for (unsigned i = 0; i < sizeof(frame.data); ++i) frame.data[i] = 0xf0 ^ i;
  assert(mcp2518fd_transmit(&frame, 15) && normal_tx == 1);
  uint32_t id = (frame.can_id >> 18) | ((frame.can_id & 0x3ffffU) << 11);
  for (unsigned i = 0; i < 4; ++i) assert(chip[0x400 + i] == (uint8_t)(id >> (8 * i)));
  assert(chip[0x404] == 0xdf && !memcmp(chip + 0x408, frame.data, 64));
  for (unsigned n = 1; n < 8; ++n) {
    frame.data[0] = n;
    assert(mcp2518fd_transmit(&frame, 15) && chip[0x408 + n * 72] == n);
  }
  assert(fifo_ua(0x64) == 0);
  chip[0x60] = 0;
  assert(!mcp2518fd_transmit(&frame, 15) && normal_tx == 8 && mcp2518fd_ready());

  can_rx_ring_t ring;
  can_rx_ring_init(&ring);
  const unsigned slot = 0x400 + 8 * 72;
  put32(slot, 0x456);
  put32(slot + 4, 0xc9); // 标准 CAN FD + BRS、12 字节
  memcpy(chip + slot + 8, frame.data, 12);
  rx_pending = 1;
  chip[0x6c] = 9; // 非空且曾溢出
  mcp2518fd_drain(&ring, 123);
  can_rx_frame_t received;
  assert(can_rx_ring_pop(&ring, &received, 1) == 1);
  assert(received.bus_index == 3 && received.can_id == 0x456 && received.rx_tick_ms == 123);
  assert(received.dlc == 12 && received.is_fd && received.is_brs && !received.is_extended);
  assert(!memcmp(received.data, frame.data, 12) && mcp2518fd_rx_overflows() == 1);
  // 末槽 RTR 后紧跟首槽的有效帧，两者同拍消费。
  put32(0x70, (8 + 15) * 72);
  chip[slot + 15 * 72 + 4] = 0x28;
  rx_pending = 2;
  chip[0x6c] = 1;
  mcp2518fd_drain(&ring, 124);
  assert((chip[0x6c] & 1U) == 0 && can_rx_ring_pop(&ring, &received, 1) == 1);
  assert(received.can_id == 0x456 && received.rx_tick_ms == 124);
  chip[0x60] = 1;
  unplug_on_ram_write = true;
  assert(!mcp2518fd_transmit(&frame, 15) && !mcp2518fd_ready() && normal_tx == 8);
  unplug_on_ram_write = false;
  connected = true;
  int_connected = false;
  assert(!mcp2518fd_init() && !mcp2518fd_ready() && normal_tx == 8);
  int_stuck_low = true;
  assert(!mcp2518fd_init() && !mcp2518fd_ready());
}
