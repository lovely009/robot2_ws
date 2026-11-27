// MCP2518FD，40 MHz 晶振，FIFO1 发送、FIFO2 接收。仅从一个任务调用。
#include "mcp2518fd.h"
#include "can4_spi.h"
#include <string.h>

enum {
  CON = 0x000, NBTCFG = 0x004, DBTCFG = 0x008, TDC = 0x00c, INT_ENABLE = 0x01e,
  TX_CTRL = 0x05c, TX_STA = 0x060, RX_CTRL = 0x068, RX_STA = 0x06c,
  FILTER_CTRL = 0x1d0, FILTER_OBJ = 0x1f0, FILTER_MASK = 0x1f4,
  RAM = 0x400, RAM_SIZE = 2048, OSC = 0xe00, DEVID = 0xe14,
  NORMAL = 0, INTERNAL_LOOPBACK = 2, CONFIG = 4,
  TX_SLOTS = 8, RX_SLOTS = 16, OBJECT_BYTES = 8 + 64, RX_BUDGET = 4,
  FIFO_AVAILABLE = 1, RX_OVERFLOW = 8, UINC = 1, TXREQ = 2,
  IDE = 0x10, RTR = 0x20, BRS = 0x40, FDF = 0x80
};
_Static_assert((TX_SLOTS + RX_SLOTS) * OBJECT_BYTES <= RAM_SIZE, "FIFO 超出消息 RAM");
_Static_assert(OBJECT_BYTES + 5 <= CAN4_SPI_MAX_BYTES, "SPI 缓冲须容纳完整 READ_CRC");

// 调试器可以在 CPU 运行时读取 DTCM，观察到哪一步以及实际收发计数。
enum { INIT_SPI = 1, INIT_RESET, INIT_TIMING, INIT_LOOPBACK, INIT_NORMAL, ONLINE, IO_FAULT };
static volatile struct can4_diagnostics {
  uint32_t stage, device_id, crc_errors, spi_errors, last_error_address;
  uint32_t rx_overflows, rx_frames, tx_submitted, last_rx_id;
  uint32_t crc_expected, crc_received;
  uint32_t reinit_attempts, reinit_successes, power_cycles, tx_stalls, mode_faults;
} g_can4_diag __attribute__((section(".dtcm_noinit")));

enum {
  REINIT_BACKOFF_MS = 1000U,
  POWER_CYCLE_FAIL_THRESHOLD = 2U,
  POWER_OFF_MS = 60U,
  POWER_ON_MS = 30U,
  HEALTH_CHECK_MS = 500U,
  TX_STALL_MS = 500U,
  TX_STALL_MIN_REJECTS = 8U,
  RESET_SETTLE_MS = 4U
};

enum reinit_state {
  R_IDLE = 0,
  R_POWER_OFF,
  R_POWER_ON,
  R_REQ_CONFIG,
  R_WAIT_CONFIG,
  R_WAIT_RESET,
  R_WAIT_LOOPBACK_MODE,
  R_WAIT_LOOPBACK_RX,
  R_WAIT_LOOPBACK_CLR,
  R_WAIT_NORMAL_CFG,
  R_WAIT_NORMAL
};

static enum reinit_state g_reinit_state;
static bool g_backoff_armed;
static bool g_tx_stall_active;
static uint32_t g_fail_streak;
static uint32_t g_tx_full_streak;
static uint32_t g_tx_stall_since_ms;
static uint32_t g_last_health_ms;
static uint32_t g_last_attempt_ms;
static uint32_t g_step_start_ms;
static unsigned g_poll_count;

static const uint8_t dlc_bytes[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
static uint32_t get_le32(const uint8_t *b) {
  return b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static void put_le32(uint8_t *b, uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) b[i] = (uint8_t)(v >> (8U * i));
}

static uint16_t crc16(const uint8_t *p, size_t n) {
  static const uint16_t table[16] = {
    0x0000,0x8005,0x800f,0x000a,0x801b,0x001e,0x0014,0x8011,
    0x8033,0x0036,0x003c,0x8039,0x0028,0x802d,0x8027,0x0022
  };
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)p[i] << 8;
    crc = (uint16_t)(crc << 4) ^ table[crc >> 12];
    crc = (uint16_t)(crc << 4) ^ table[crc >> 12];
  }
  return crc;
}

static bool io_fault(unsigned addr) {
  g_can4_diag.last_error_address = addr;
  if (g_can4_diag.stage == ONLINE) g_can4_diag.stage = IO_FAULT;
  return false;
}

// DS80000789F 第 1 项：仅使用 READ_CRC，遇到 SDO 瞬态错误有界重读。
static bool read_bytes(unsigned addr, uint8_t *out, unsigned n) {
  uint8_t tx[OBJECT_BYTES + 5] = {0}, rx[sizeof(tx)];
  tx[0] = 0xb0 | (addr >> 8);
  tx[1] = (uint8_t)addr;
  bool message_ram = addr >= RAM && addr < RAM + RAM_SIZE;
  tx[2] = message_ram ? n / 4U : n;
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    if (!can4_spi_transfer(tx, rx, n + 5, message_ram)) {
      ++g_can4_diag.spi_errors;
      break;
    }
    memcpy(rx, tx, 3);
    uint16_t expected = crc16(rx, n + 3);
    uint16_t actual = (uint16_t)rx[n + 3] << 8 | rx[n + 4];
    if (expected == actual) {
      memcpy(out, rx + 3, n);
      return true;
    }
    ++g_can4_diag.crc_errors;
    g_can4_diag.crc_expected = expected;
    g_can4_diag.crc_received = actual;
  }
  return io_fault(addr);
}

static bool write_bytes(unsigned addr, const uint8_t *data, unsigned n) {
  uint8_t tx[OBJECT_BYTES + 2], rx[sizeof(tx)];
  tx[0] = 0x20 | (addr >> 8);
  tx[1] = (uint8_t)addr;
  memcpy(tx + 2, data, n);
  if (can4_spi_transfer(tx, rx, n + 2, false)) return true;
  ++g_can4_diag.spi_errors;
  return io_fault(addr);
}

static bool write_byte(unsigned addr, uint8_t value) {
  return write_bytes(addr, &value, 1);
}

static bool write_word(unsigned addr, uint32_t value) {
  uint8_t data[4];
  put_le32(data, value);
  return write_bytes(addr, data, sizeof(data));
}

static bool set_mode(uint8_t mode) {
  // MCU 单独复位时可能有待 ACK 的旧帧，先中止，再进入配置模式。
  uint8_t request = mode == CONFIG ? mode | 8U : mode;
  if (!write_byte(CON + 3, request)) return false;
  for (unsigned i = 0; i < 12; ++i) {
    uint8_t status;
    if (!read_bytes(CON + 2, &status, 1)) return false;
    if ((status >> 5) == mode) return true;
    can4_init_delay_ms(1);
  }
  return false;
}

static bool reset_controller(void) {
  uint8_t reset[2] = {0}, reply[2], osc[4], device[4];
  if (!set_mode(CONFIG) || !can4_spi_transfer(reset, reply, 2, false)) return false;
  can4_init_delay_ms(3);
  if (!read_bytes(OSC, osc, 4) || !read_bytes(DEVID, device, 4)) return false;
  g_can4_diag.device_id = get_le32(device);
  // OSCRDY=1，SCLKDIV=PLL倍频=0，使用模块的 40 MHz 原始时钟。
  return (get_le32(osc) & 0x411U) == 0x400U && (get_le32(device) & ~0xffU) == 0;
}

static bool configure(void) {
  // 仲裁 40 tq (1+31+8)、数据 8 tq (1+5+2)，BRP=1；寄存器编码减一。
  // 使用自动 TDC，按微芯 40 MHz / 5 Mbps 参考配置选 TDCO=5。
  const uint32_t payload64 = 7UL << 29;
  return write_word(CON, ((uint32_t)CONFIG << 24) | (1U << 16) | (1U << 5)) &&
      write_word(NBTCFG, (30U << 16) | (7U << 8) | 7U) &&
      write_word(DBTCFG, (4U << 16) | (1U << 8) | 1U) &&
      write_word(TDC, (2U << 16) | (5U << 8)) &&
      // 禁用 TEF/TXQ。短时干扰最多重试三次，断线时不无限积压旧帧。
      write_word(TX_CTRL, payload64 | ((TX_SLOTS - 1U) << 24) | (1U << 21) | 0x80U) &&
      write_word(RX_CTRL, payload64 | ((RX_SLOTS - 1U) << 24) | RX_OVERFLOW | FIFO_AVAILABLE) &&
      write_word(FILTER_OBJ, 0) && write_word(FILTER_MASK, 1UL << 30) &&
      write_byte(FILTER_CTRL, 0x82) && // 匹配全部标准 ID → FIFO2
      write_byte(INT_ENABLE, 2);       // RXIE → H1.9；不启用 TX 非满中断
}

// 状态和对应 UA 连读。UA 是以字节计的 RAM 偏移，配置模式下不可用。
static bool fifo_object(unsigned sta, unsigned *addr) {
  uint8_t pair[8];
  if (!read_bytes(sta, pair, sizeof(pair))) return false;
  if (sta == RX_STA && (pair[0] & RX_OVERFLOW)) {
    ++g_can4_diag.rx_overflows;
    if (!write_byte(RX_STA, 0)) return false;
  }
  if (!(pair[0] & FIFO_AVAILABLE)) return false;
  uint32_t ua = get_le32(pair + 4);
  unsigned first = sta == TX_STA ? 0 : TX_SLOTS * OBJECT_BYTES;
  unsigned end = sta == TX_STA ? TX_SLOTS * OBJECT_BYTES : (TX_SLOTS + RX_SLOTS) * OBJECT_BYTES;
  if (ua < first || ua >= end || ua % OBJECT_BYTES != 0) return io_fault(sta + 4);
  *addr = RAM + ua;
  return true;
}

static bool send_object(const can_tx_frame_t *f, uint8_t dlc) {
  unsigned addr;
  if (!fifo_object(TX_STA, &addr)) return false;
  uint8_t object[OBJECT_BYTES] = {0};
  uint32_t id = f->can_id;
  if (f->is_extended) id = (id >> 18) | ((id & 0x3ffffU) << 11);
  put_le32(object, id);
  object[4] = dlc | (f->is_extended ? IDE : 0) | (f->is_fd ? FDF : 0) | (f->is_brs ? BRS : 0);
  memcpy(object + 8, f->data, f->size);
  unsigned bytes = 8U + ((f->size + 3U) & ~3U);
  if (!write_bytes(addr, object, bytes) || !write_byte(TX_CTRL + 1, UINC | TXREQ)) return false;
  // SPI 写指令没有从机应答。确认芯片真的推进写指针，断线时不会把 P0 记为成功。
  uint8_t ua[4];
  if (!read_bytes(TX_STA + 4, ua, 4)) return false;
  uint32_t expected = (addr - RAM + OBJECT_BYTES) % (TX_SLOTS * OBJECT_BYTES);
  return get_le32(ua) == expected || io_fault(TX_STA + 4);
}

// 消费远程帧，其数据不交给上层。只有 UINC 成功后才发布帧。
enum rx_result { RX_STOP, RX_IGNORED, RX_DATA };
static enum rx_result receive_object(can_rx_frame_t *f) {
  unsigned addr;
  uint8_t object[OBJECT_BYTES];
  if (!fifo_object(RX_STA, &addr) || !read_bytes(addr, object, sizeof(object)) ||
      !write_byte(RX_CTRL + 1, UINC)) return RX_STOP;
  uint8_t flags = object[4];
  if (flags & (IDE | RTR)) return RX_IGNORED;
  f->can_id = get_le32(object) & 0x7ff;
  f->is_fd = (flags & FDF) != 0;
  f->is_brs = f->is_fd && (flags & BRS);
  uint8_t dlc = flags & 15U;
  f->dlc = f->is_fd || dlc <= 8U ? dlc_bytes[dlc] : 8U;
  memcpy(f->data, object + 8, f->dlc);
  return RX_DATA;
}

static bool loopback(void) {
  can_tx_frame_t probe = {
    .can_id = 0x5a5, .is_fd = true, .is_brs = true, .size = 64
  };
  for (unsigned i = 0; i < probe.size; ++i) probe.data[i] = (uint8_t)(0xa5U ^ (29U * i));
  if (!set_mode(INTERNAL_LOOPBACK) || can4_irq_asserted() || !send_object(&probe, 15)) return false;
  for (unsigned i = 0; i < 8; ++i) {
    if (can4_irq_asserted()) {
      can_rx_frame_t f = {0};
      if (receive_object(&f) != RX_DATA || !f.is_fd || !f.is_brs || f.can_id != probe.can_id ||
          f.dlc != probe.size || memcmp(f.data, probe.data, probe.size) != 0) return false;
      for (unsigned wait = 0; can4_irq_asserted() && wait < 3; ++wait) can4_init_delay_ms(1);
      return !can4_irq_asserted();
    }
    can4_init_delay_ms(1);
  }
  return false;
}

static void reset_runtime_recovery(void) {
  g_reinit_state = R_IDLE;
  g_backoff_armed = false;
  g_tx_stall_active = false;
  g_fail_streak = 0U;
  g_tx_full_streak = 0U;
  g_tx_stall_since_ms = 0U;
  g_last_health_ms = 0U;
  g_last_attempt_ms = 0U;
  g_step_start_ms = 0U;
  g_poll_count = 0U;
}

static bool probe_reset_and_configure(void) {
  uint8_t osc[4], device[4];
  if (!read_bytes(OSC, osc, 4) || !read_bytes(DEVID, device, 4)) return false;
  g_can4_diag.device_id = get_le32(device);
  if ((get_le32(osc) & 0x411U) != 0x400U || (get_le32(device) & ~0xffU) != 0) return false;
  g_can4_diag.stage = INIT_TIMING;
  return configure();
}

static bool send_loopback_probe(void) {
  can_tx_frame_t probe = {
    .can_id = 0x5a5, .is_fd = true, .is_brs = true, .size = 64
  };
  for (unsigned i = 0; i < probe.size; ++i) probe.data[i] = (uint8_t)(0xa5U ^ (29U * i));
  return !can4_irq_asserted() && send_object(&probe, 15);
}

static bool verify_loopback_probe(void) {
  can_rx_frame_t f = {0};
  if (receive_object(&f) != RX_DATA || !f.is_fd || !f.is_brs ||
      f.can_id != 0x5a5U || f.dlc != 64U) {
    return false;
  }
  for (unsigned i = 0; i < 64U; ++i) {
    if (f.data[i] != (uint8_t)(0xa5U ^ (29U * i))) return false;
  }
  return true;
}

static void fail_reinit_attempt(uint32_t now_ms) {
  ++g_fail_streak;
  g_last_attempt_ms = now_ms;
  g_backoff_armed = true;
  g_reinit_state = R_IDLE;
}

static void step_reinit(uint32_t now_ms) {
  switch (g_reinit_state) {
  case R_IDLE:
    if (!g_backoff_armed) {
      g_last_attempt_ms = now_ms;
      g_backoff_armed = true;
      return;
    }
    if ((uint32_t)(now_ms - g_last_attempt_ms) < REINIT_BACKOFF_MS) return;
    g_last_attempt_ms = now_ms;
    ++g_can4_diag.reinit_attempts;
    if (g_fail_streak >= POWER_CYCLE_FAIL_THRESHOLD) {
      can4_spi_power_set(false);
      ++g_can4_diag.power_cycles;
      g_step_start_ms = now_ms;
      g_reinit_state = R_POWER_OFF;
      return;
    }
    g_can4_diag.stage = INIT_RESET;
    if (!write_byte(CON + 3, CONFIG | 8U)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    g_poll_count = 0U;
    g_reinit_state = R_WAIT_CONFIG;
    return;

  case R_POWER_OFF:
    if ((uint32_t)(now_ms - g_step_start_ms) < POWER_OFF_MS) return;
    can4_spi_power_set(true);
    g_step_start_ms = now_ms;
    g_reinit_state = R_POWER_ON;
    return;

  case R_POWER_ON:
    if ((uint32_t)(now_ms - g_step_start_ms) < POWER_ON_MS) return;
    if (!can4_spi_init()) {
      fail_reinit_attempt(now_ms);
      return;
    }
    g_fail_streak = 0U;
    g_can4_diag.stage = INIT_RESET;
    if (!write_byte(CON + 3, CONFIG | 8U)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    g_poll_count = 0U;
    g_reinit_state = R_WAIT_CONFIG;
    return;

  case R_WAIT_CONFIG: {
    uint8_t status;
    if (!read_bytes(CON + 2, &status, 1)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    if ((status >> 5) == CONFIG) {
      uint8_t reset[2] = {0}, reply[2];
      if (!can4_spi_transfer(reset, reply, 2, false)) {
        ++g_can4_diag.spi_errors;
        fail_reinit_attempt(now_ms);
        return;
      }
      g_step_start_ms = now_ms;
      g_reinit_state = R_WAIT_RESET;
      return;
    }
    if (++g_poll_count >= 12U) fail_reinit_attempt(now_ms);
    return;
  }

  case R_WAIT_RESET:
    if ((uint32_t)(now_ms - g_step_start_ms) < RESET_SETTLE_MS) return;
    if (!probe_reset_and_configure()) {
      fail_reinit_attempt(now_ms);
      return;
    }
    g_can4_diag.stage = INIT_LOOPBACK;
    if (!write_byte(CON + 3, INTERNAL_LOOPBACK)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    g_poll_count = 0U;
    g_reinit_state = R_WAIT_LOOPBACK_MODE;
    return;

  case R_WAIT_LOOPBACK_MODE: {
    uint8_t status;
    if (!read_bytes(CON + 2, &status, 1)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    if ((status >> 5) == INTERNAL_LOOPBACK) {
      if (!send_loopback_probe()) {
        fail_reinit_attempt(now_ms);
        return;
      }
      g_poll_count = 0U;
      g_reinit_state = R_WAIT_LOOPBACK_RX;
      return;
    }
    if (++g_poll_count >= 12U) fail_reinit_attempt(now_ms);
    return;
  }

  case R_WAIT_LOOPBACK_RX:
    if (can4_irq_asserted()) {
      if (!verify_loopback_probe()) {
        fail_reinit_attempt(now_ms);
        return;
      }
      g_poll_count = 0U;
      g_reinit_state = R_WAIT_LOOPBACK_CLR;
      return;
    }
    if (++g_poll_count >= 8U) fail_reinit_attempt(now_ms);
    return;

  case R_WAIT_LOOPBACK_CLR:
    if (!can4_irq_asserted()) {
      g_can4_diag.stage = INIT_NORMAL;
      if (!write_byte(CON + 3, CONFIG | 8U)) {
        fail_reinit_attempt(now_ms);
        return;
      }
      g_poll_count = 0U;
      g_reinit_state = R_WAIT_NORMAL_CFG;
      return;
    }
    if (++g_poll_count >= 3U) fail_reinit_attempt(now_ms);
    return;

  case R_WAIT_NORMAL_CFG: {
    uint8_t status;
    if (!read_bytes(CON + 2, &status, 1)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    if ((status >> 5) == CONFIG) {
      if (!write_byte(CON + 3, NORMAL)) {
        fail_reinit_attempt(now_ms);
        return;
      }
      g_poll_count = 0U;
      g_reinit_state = R_WAIT_NORMAL;
      return;
    }
    if (++g_poll_count >= 12U) fail_reinit_attempt(now_ms);
    return;
  }

  case R_WAIT_NORMAL: {
    uint8_t status;
    if (!read_bytes(CON + 2, &status, 1)) {
      fail_reinit_attempt(now_ms);
      return;
    }
    if ((status >> 5) == NORMAL) {
      g_can4_diag.stage = ONLINE;
      ++g_can4_diag.reinit_successes;
      g_fail_streak = 0U;
      g_tx_full_streak = 0U;
      g_tx_stall_active = false;
      g_last_health_ms = now_ms;
      g_reinit_state = R_IDLE;
      return;
    }
    if (++g_poll_count >= 12U) fail_reinit_attempt(now_ms);
    return;
  }

  case R_REQ_CONFIG:
  default:
    fail_reinit_attempt(now_ms);
    return;
  }
}

static void check_online_health(uint32_t now_ms) {
  if (g_tx_full_streak >= TX_STALL_MIN_REJECTS) {
    if (!g_tx_stall_active) {
      g_tx_stall_active = true;
      g_tx_stall_since_ms = now_ms;
    } else if ((uint32_t)(now_ms - g_tx_stall_since_ms) >= TX_STALL_MS) {
      ++g_can4_diag.tx_stalls;
      g_tx_full_streak = 0U;
      g_tx_stall_active = false;
      g_fail_streak = 0U;
      g_backoff_armed = false;
      (void)io_fault(TX_STA);
      return;
    }
  }
  if ((uint32_t)(now_ms - g_last_health_ms) >= HEALTH_CHECK_MS) {
    g_last_health_ms = now_ms;
    uint8_t status;
    if (!read_bytes(CON + 2, &status, 1)) return;
    if ((status >> 5) != NORMAL) {
      ++g_can4_diag.mode_faults;
      g_fail_streak = 0U;
      g_backoff_armed = false;
      (void)io_fault(CON + 2);
    }
  }
}

bool mcp2518fd_init(void) {
  g_can4_diag = (struct can4_diagnostics){.stage = INIT_SPI};
  reset_runtime_recovery();
  if (!can4_spi_init()) return false;
  g_can4_diag.stage = INIT_RESET;
  if (!reset_controller()) return false;
  g_can4_diag.stage = INIT_TIMING;
  if (!configure()) return false;
  g_can4_diag.stage = INIT_LOOPBACK;
  if (!loopback()) return false;
  g_can4_diag.stage = INIT_NORMAL;
  if (!set_mode(CONFIG) || !set_mode(NORMAL)) return false;
  g_can4_diag.stage = ONLINE;
  return true;
}

bool mcp2518fd_ready(void) { return g_can4_diag.stage == ONLINE; }
uint32_t mcp2518fd_rx_overflows(void) { return g_can4_diag.rx_overflows; }

bool mcp2518fd_transmit(const can_tx_frame_t *frame, uint8_t dlc) {
  if (!mcp2518fd_ready() || !frame || dlc > 15U || dlc_bytes[dlc] != frame->size ||
      (!frame->is_fd && (frame->size > 8U || frame->is_brs)) ||
      frame->can_id > (frame->is_extended ? 0x1fffffffU : 0x7ffU)) return false;
  if (!send_object(frame, dlc)) {
    if (g_can4_diag.stage == ONLINE && g_tx_full_streak < UINT32_MAX) ++g_tx_full_streak;
    return false;
  }
  g_tx_full_streak = 0U;
  g_tx_stall_active = false;
  ++g_can4_diag.tx_submitted;
  return true;
}

void mcp2518fd_drain(can_rx_ring_t *ring, uint32_t now_ms) {
  if (!mcp2518fd_ready()) {
    step_reinit(now_ms);
    return;
  }
  check_online_health(now_ms);
  if (!mcp2518fd_ready()) return;
  for (unsigned i = 0; i < RX_BUDGET && can4_irq_asserted(); ++i) {
    can_rx_frame_t f = {.bus_index = FIRMWARE_CAN4_INDEX, .rx_tick_ms = now_ms};
    enum rx_result result = receive_object(&f);
    if (result == RX_STOP) break;
    if (result == RX_IGNORED) continue;
    ++g_can4_diag.rx_frames;
    g_can4_diag.last_rx_id = f.can_id;
    (void)can_rx_ring_push_from_isr(ring, &f);
  }
}
