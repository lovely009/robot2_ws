#include "motor_drive/slcan.hpp"

#include <cctype>
#include <cstdio>

namespace motor_drive {
namespace {

constexpr std::size_t kSffIdLen = 3U;
constexpr std::size_t kEffIdLen = 8U;
constexpr uint32_t kSffIdMask = 0x7FFU;
constexpr uint32_t kEffIdMask = 0x1FFFFFFFU;
constexpr std::size_t kClassicMaxDlc = 8U;

int hex_value(const char ch) {
  if (ch >= '0' && ch <= '9') {
    return ch - '0';
  }
  if (ch >= 'A' && ch <= 'F') {
    return ch - 'A' + 10;
  }
  if (ch >= 'a' && ch <= 'f') {
    return ch - 'a' + 10;
  }
  return -1;
}

// CAN-FD 有效负载字节数 -> slcan 长度码字符；非法长度返回 0（0 字节也合法，用哨兵区分）。
char fd_len_code(const std::size_t nbytes) {
  if (nbytes <= kClassicMaxDlc) {
    return static_cast<char>('0' + nbytes);
  }
  switch (nbytes) {
    case 12U:
      return '9';
    case 16U:
      return 'A';
    case 20U:
      return 'B';
    case 24U:
      return 'C';
    case 32U:
      return 'D';
    case 48U:
      return 'E';
    case 64U:
      return 'F';
    default:
      return '\0';
  }
}

bool fd_bytes_for_code(const int code, std::size_t &nbytes) {
  if (code >= 0 && code <= 8) {
    nbytes = static_cast<std::size_t>(code);
    return true;
  }
  switch (code) {
    case 9:
      nbytes = 12U;
      return true;
    case 10:
      nbytes = 16U;
      return true;
    case 11:
      nbytes = 20U;
      return true;
    case 12:
      nbytes = 24U;
      return true;
    case 13:
      nbytes = 32U;
      return true;
    case 14:
      nbytes = 48U;
      return true;
    case 15:
      nbytes = 64U;
      return true;
    default:
      return false;
  }
}

} // namespace

std::string encode_slcan_frame(const CanFrame &frame) {
  if (frame.data.size() < frame.size) {
    return {};
  }

  char command = '\0';
  if (!frame.is_fd) {
    if (frame.size > kClassicMaxDlc || frame.is_extended) {
      // 当前协议族只需要经典标准帧；扩展经典帧暂不编码。
      return {};
    }
    command = 't';
  } else {
    const char length = fd_len_code(frame.size);
    if (length == '\0') {
      return {};
    }
    if (frame.bitrate_switch) {
      command = frame.is_extended ? 'B' : 'b';
    } else {
      command = frame.is_extended ? 'D' : 'd';
    }
  }

  if (frame.is_extended) {
    if (frame.id > kEffIdMask) {
      return {};
    }
  } else if (frame.id > kSffIdMask) {
    return {};
  }

  std::string line;
  line.reserve(1U + kEffIdLen + 1U + frame.size * 2U);
  line.push_back(command);
  char id_buffer[16]{};
  if (frame.is_extended) {
    std::snprintf(id_buffer, sizeof(id_buffer), "%08X", frame.id);
  } else {
    std::snprintf(id_buffer, sizeof(id_buffer), "%03X", frame.id);
  }
  line += id_buffer;
  if (frame.is_fd) {
    line.push_back(fd_len_code(frame.size));
  } else {
    line.push_back(static_cast<char>('0' + frame.size));
  }
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (std::size_t i = 0U; i < frame.size; ++i) {
    line.push_back(kHex[(frame.data[i] >> 4U) & 0x0FU]);
    line.push_back(kHex[frame.data[i] & 0x0FU]);
  }
  return line;
}

bool decode_slcan_line(const std::string &line, CanFrame &frame) {
  if (line.empty()) {
    return false;
  }

  bool is_fd = false;
  bool bitrate_switch = false;
  bool is_extended = false;
  switch (line[0]) {
    case 't':
      break;
    case 'T':
      is_extended = true;
      break;
    case 'd':
      is_fd = true;
      break;
    case 'D':
      is_fd = true;
      is_extended = true;
      break;
    case 'b':
      is_fd = true;
      bitrate_switch = true;
      break;
    case 'B':
      is_fd = true;
      bitrate_switch = true;
      is_extended = true;
      break;
    default:
      // 状态帧（a/w/p/b...）、错误帧与乱码一律忽略。
      return false;
  }

  const std::size_t id_len = is_extended ? kEffIdLen : kSffIdLen;
  if (line.size() < 1U + id_len + 1U) {
    return false;
  }

  uint32_t id = 0U;
  for (std::size_t i = 0U; i < id_len; ++i) {
    const int digit = hex_value(line[1U + i]);
    if (digit < 0) {
      return false;
    }
    id = (id << 4U) | static_cast<uint32_t>(digit);
  }

  const int length_code = hex_value(line[1U + id_len]);
  if (length_code < 0) {
    return false;
  }
  std::size_t nbytes = 0U;
  if (is_fd) {
    if (!fd_bytes_for_code(length_code, nbytes)) {
      return false;
    }
  } else if (length_code > static_cast<int>(kClassicMaxDlc)) {
    return false;
  } else {
    nbytes = static_cast<std::size_t>(length_code);
  }

  const std::size_t hex_start = 1U + id_len + 1U;
  if (line.size() < hex_start + nbytes * 2U) {
    return false;
  }

  CanFrame decoded{};
  decoded.id = id;
  decoded.is_fd = is_fd;
  decoded.bitrate_switch = bitrate_switch;
  decoded.is_extended = is_extended;
  decoded.size = nbytes;
  for (std::size_t i = 0U; i < nbytes; ++i) {
    const int high = hex_value(line[hex_start + i * 2U]);
    const int low = hex_value(line[hex_start + i * 2U + 1U]);
    if (high < 0 || low < 0) {
      return false;
    }
    decoded.data[i] = static_cast<uint8_t>((high << 4) | low);
  }

  frame = decoded;
  return true;
}

} // namespace motor_drive
