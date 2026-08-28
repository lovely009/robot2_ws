#include "motor_sim/transport/frame_record.hpp"

#include <cstdio>
#include <optional>
#include <stdexcept>

namespace motor_sim::transport {
namespace {

constexpr std::size_t kStandardIdWidth = 3U;
constexpr std::size_t kExtendedIdWidth = 8U;
constexpr uint32_t kStandardIdMaximum = 0x7ffU;
constexpr uint32_t kExtendedIdMaximum = 0x1fffffffU;

int hex_value(const char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  return -1;
}

char fd_length_code(const std::size_t size) {
  if (size <= 8U) {
    return static_cast<char>('0' + size);
  }
  switch (size) {
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
      throw std::invalid_argument("invalid CAN-FD payload length");
  }
}

std::optional<std::size_t> payload_size(const bool fd, const char code) {
  const int value = hex_value(code);
  if (value < 0) {
    return std::nullopt;
  }
  if (!fd) {
    return value <= 8 ? std::optional<std::size_t>(value) : std::nullopt;
  }
  if (value <= 8) {
    return static_cast<std::size_t>(value);
  }
  static constexpr std::size_t sizes[]{12U, 16U, 20U, 24U,
                                       32U, 48U, 64U};
  return sizes[value - 9];
}

std::optional<protocol::CanFrame> decode_record(const std::string_view record) {
  if (record.empty()) {
    return std::nullopt;
  }
  protocol::CanFrame frame;
  switch (record.front()) {
    case 't':
      break;
    case 'T':
      frame.extended = true;
      break;
    case 'd':
      frame.fd = true;
      break;
    case 'D':
      frame.fd = true;
      frame.extended = true;
      break;
    case 'b':
      frame.fd = true;
      frame.brs = true;
      break;
    case 'B':
      frame.extended = true;
      frame.fd = true;
      frame.brs = true;
      break;
    default:
      return std::nullopt;
  }

  const auto id_width = frame.extended ? kExtendedIdWidth : kStandardIdWidth;
  if (record.size() < 1U + id_width + 1U) {
    return std::nullopt;
  }
  for (std::size_t index = 0U; index < id_width; ++index) {
    const int digit = hex_value(record[1U + index]);
    if (digit < 0) {
      return std::nullopt;
    }
    frame.id = (frame.id << 4U) | static_cast<uint32_t>(digit);
  }
  if ((!frame.extended && frame.id > kStandardIdMaximum) ||
      (frame.extended && frame.id > kExtendedIdMaximum)) {
    return std::nullopt;
  }
  const auto size = payload_size(frame.fd, record[1U + id_width]);
  if (!size.has_value()) {
    return std::nullopt;
  }
  frame.size = *size;
  const auto payload_offset = 1U + id_width + 1U;
  if (record.size() != payload_offset + frame.size * 2U) {
    return std::nullopt;
  }
  for (std::size_t index = 0U; index < frame.size; ++index) {
    const int high = hex_value(record[payload_offset + index * 2U]);
    const int low = hex_value(record[payload_offset + index * 2U + 1U]);
    if (high < 0 || low < 0) {
      return std::nullopt;
    }
    frame.data[index] = static_cast<uint8_t>((high << 4U) | low);
  }
  return frame;
}

}  // namespace

std::string encode_frame_record(const protocol::CanFrame &frame) {
  if (frame.size > frame.data.size()) {
    throw std::invalid_argument("CAN payload exceeds frame storage");
  }
  if ((!frame.extended && frame.id > kStandardIdMaximum) ||
      (frame.extended && frame.id > kExtendedIdMaximum)) {
    throw std::invalid_argument("CAN identifier is out of range");
  }
  char prefix = 't';
  if (frame.fd) {
    prefix = frame.brs ? 'b' : 'd';
  } else if (frame.brs || frame.size > 8U) {
    throw std::invalid_argument("invalid classic CAN frame flags or length");
  }
  if (frame.extended) {
    prefix = static_cast<char>(prefix - ('a' - 'A'));
  }
  const char length = frame.fd ? fd_length_code(frame.size)
                               : static_cast<char>('0' + frame.size);

  std::string record;
  record.reserve(1U + kExtendedIdWidth + 1U + frame.size * 2U + 1U);
  record.push_back(prefix);
  char id_buffer[16]{};
  std::snprintf(id_buffer, sizeof(id_buffer), frame.extended ? "%08X" : "%03X",
                frame.id);
  record += id_buffer;
  record.push_back(length);
  static constexpr char kHex[] = "0123456789ABCDEF";
  for (std::size_t index = 0U; index < frame.size; ++index) {
    record.push_back(kHex[(frame.data[index] >> 4U) & 0x0fU]);
    record.push_back(kHex[frame.data[index] & 0x0fU]);
  }
  record.push_back('\r');
  return record;
}

std::vector<protocol::CanFrame>
FrameRecordParser::push(const std::string_view bytes) {
  std::vector<protocol::CanFrame> frames;
  for (const char byte : bytes) {
    if (byte == '\r') {
      if (dropping_overlong_record_) {
        ++rejected_records_;
        dropping_overlong_record_ = false;
      } else if (!pending_.empty()) {
        const auto frame = decode_record(pending_);
        if (frame.has_value()) {
          frames.push_back(*frame);
        } else {
          ++rejected_records_;
        }
      }
      pending_.clear();
      continue;
    }
    if (dropping_overlong_record_) {
      continue;
    }
    pending_.push_back(byte);
    if (pending_.size() > kMaximumRecordSize) {
      pending_.clear();
      dropping_overlong_record_ = true;
    }
  }
  return frames;
}

}  // namespace motor_sim::transport
