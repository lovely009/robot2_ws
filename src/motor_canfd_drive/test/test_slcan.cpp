#include "motor_canfd_drive/slcan.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

namespace drive = motor_canfd_drive;

namespace {

drive::CanFrame make_frame(const uint32_t id, const std::string &hex,
                           const bool is_fd, const bool brs,
                           const bool is_extended) {
  drive::CanFrame frame;
  frame.id = id;
  frame.is_fd = is_fd;
  frame.bitrate_switch = brs;
  frame.is_extended = is_extended;
  frame.size = hex.size() / 2U;
  for (std::size_t i = 0U; i < frame.size; ++i) {
    frame.data[i] = static_cast<uint8_t>(std::stoi(hex.substr(i * 2U, 2U), nullptr, 16));
  }
  return frame;
}

} // namespace

TEST(SlcanCodec, EncodesClassicStandardFrame) {
  // 2530 使能帧：ID 0x506，经典 CAN，8 字节 {1,0,...}。
  const auto frame = make_frame(0x506U, "0100000000000000", false, false, false);
  EXPECT_EQ(drive::encode_slcan_frame(frame), "t50680100000000000000");
}

TEST(SlcanCodec, EncodesFdBrsFrame) {
  // 2530 MIT 命令帧：ID 0x505，FD+BRS，12 字节。
  const auto frame =
      make_frame(0x505U, "010001800080200364000080", true, true, false);
  EXPECT_EQ(drive::encode_slcan_frame(frame), "b5059010001800080200364000080");
}

TEST(SlcanCodec, EncodesFdNoBrsExtendedAndMaxLength) {
  const auto no_brs =
      make_frame(0x1234U, "AABBCCDD", true, false, true);
  EXPECT_EQ(drive::encode_slcan_frame(no_brs), "D000012344AABBCCDD");

  drive::CanFrame max_frame;
  max_frame.id = 0x80U;
  max_frame.is_fd = true;
  max_frame.bitrate_switch = true;
  max_frame.size = 64U;
  for (std::size_t i = 0U; i < 64U; ++i) {
    max_frame.data[i] = static_cast<uint8_t>(i);
  }
  const auto line = drive::encode_slcan_frame(max_frame);
  EXPECT_EQ(line.size(), 1U + 3U + 1U + 128U);
  EXPECT_EQ(line.substr(0U, 5U), "b080F");
}

TEST(SlcanCodec, RejectsInvalidFrames) {
  drive::CanFrame classic_too_long;
  classic_too_long.size = 9U;
  EXPECT_TRUE(drive::encode_slcan_frame(classic_too_long).empty());

  drive::CanFrame bad_fd_length;
  bad_fd_length.is_fd = true;
  bad_fd_length.size = 10U;  // CAN-FD 不允许 10 字节
  EXPECT_TRUE(drive::encode_slcan_frame(bad_fd_length).empty());

  drive::CanFrame bad_extended_id;
  bad_extended_id.is_fd = true;
  bad_extended_id.is_extended = true;
  bad_extended_id.id = 0x20000000U;  // 超 EFF mask
  bad_extended_id.size = 1U;
  EXPECT_TRUE(drive::encode_slcan_frame(bad_extended_id).empty());
}

TEST(SlcanCodec, DecodesStatusFrame) {
  // 实测状态帧：b706B + 20 字节（Data0-1=05 05，pos 码 0x...）。
  const std::string line =
      "b706B0505EB532E800080008077000000000004000102";
  drive::CanFrame frame;
  ASSERT_TRUE(drive::decode_slcan_line(line, frame));
  EXPECT_EQ(frame.id, 0x706U);
  EXPECT_TRUE(frame.is_fd);
  EXPECT_TRUE(frame.bitrate_switch);
  EXPECT_FALSE(frame.is_extended);
  EXPECT_EQ(frame.size, 20U);
  EXPECT_EQ(frame.data[0], 0x05U);
  EXPECT_EQ(frame.data[1], 0x05U);
  EXPECT_EQ(frame.data[18], 0x01U);
  EXPECT_EQ(frame.data[19], 0x02U);
}

TEST(SlcanCodec, DecodesClassicAndReplyFrames) {
  drive::CanFrame frame;
  ASSERT_TRUE(drive::decode_slcan_line("t50680100000000000000", frame));
  EXPECT_EQ(frame.id, 0x506U);
  EXPECT_FALSE(frame.is_fd);
  EXPECT_EQ(frame.size, 8U);

  ASSERT_TRUE(drive::decode_slcan_line(
      "b705905055A542E800080FC7F0000", frame));
  EXPECT_EQ(frame.id, 0x705U);
  EXPECT_TRUE(frame.is_fd);
  EXPECT_EQ(frame.size, 12U);
}

TEST(SlcanCodec, IgnoresStateAndGarbageLines) {
  drive::CanFrame frame;
  EXPECT_FALSE(drive::decode_slcan_line("a000000", frame));
  EXPECT_FALSE(drive::decode_slcan_line("w001002", frame));
  EXPECT_FALSE(drive::decode_slcan_line("p000003", frame));
  EXPECT_FALSE(drive::decode_slcan_line("e0000", frame));
  EXPECT_FALSE(drive::decode_slcan_line("", frame));
  EXPECT_FALSE(drive::decode_slcan_line("t7", frame));
  EXPECT_FALSE(drive::decode_slcan_line("b706G0505", frame));
}

TEST(SlcanCodec, RoundTripPreservesFrame) {
  const auto original =
      make_frame(0x80U, "0102030405060708090A0B0C0D0E0F10", true, true, false);
  const auto line = drive::encode_slcan_frame(original);
  drive::CanFrame decoded;
  ASSERT_TRUE(drive::decode_slcan_line(line, decoded));
  EXPECT_EQ(decoded.id, original.id);
  EXPECT_EQ(decoded.is_fd, original.is_fd);
  EXPECT_EQ(decoded.bitrate_switch, original.bitrate_switch);
  EXPECT_EQ(decoded.size, original.size);
  EXPECT_EQ(decoded.data, original.data);
}
