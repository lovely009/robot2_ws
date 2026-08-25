#include "humanoid_gazebo/transport/frame_record.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace protocol = humanoid_gazebo::protocol;
namespace transport = humanoid_gazebo::transport;

namespace {

protocol::CanFrame make_frame(const uint32_t id, const std::string &hex,
                              const bool fd, const bool brs,
                              const bool extended) {
  protocol::CanFrame frame;
  frame.id = id;
  frame.fd = fd;
  frame.brs = brs;
  frame.extended = extended;
  frame.size = hex.size() / 2U;
  for (std::size_t index = 0U; index < frame.size; ++index) {
    frame.data[index] = static_cast<uint8_t>(
        std::stoi(hex.substr(index * 2U, 2U), nullptr, 16));
  }
  return frame;
}

}  // namespace

TEST(FrameRecord, EncodesClassicAndFdBrsGoldenVectors) {
  EXPECT_EQ(transport::encode_frame_record(
                make_frame(0x506U, "0100000000000000", false, false, false)),
            "t50680100000000000000\r");
  EXPECT_EQ(transport::encode_frame_record(
                make_frame(0x505U, "010001800080200364000080",
                           true, true, false)),
            "b5059010001800080200364000080\r");
  EXPECT_EQ(transport::encode_frame_record(
                make_frame(0x1234U, "AABBCCDD", true, false, true)),
            "D000012344AABBCCDD\r");
}

TEST(FrameRecord, RejectsInvalidPayloadLengthsAndIdentifiers) {
  auto frame = make_frame(0x80U, "00112233445566778899", false, false, false);
  EXPECT_THROW(transport::encode_frame_record(frame), std::invalid_argument);
  frame.fd = true;
  frame.size = 10U;
  EXPECT_THROW(transport::encode_frame_record(frame), std::invalid_argument);
  frame.size = 1U;
  frame.extended = true;
  frame.id = 0x20000000U;
  EXPECT_THROW(transport::encode_frame_record(frame), std::invalid_argument);
}

TEST(FrameRecordParser, ReassemblesFragmentedFdBrsRecord) {
  transport::FrameRecordParser parser;
  EXPECT_TRUE(parser.push("b1233AABB").empty());
  const auto frames = parser.push("CC\r");
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].id, 0x123U);
  EXPECT_TRUE(frames[0].fd);
  EXPECT_TRUE(frames[0].brs);
  EXPECT_EQ(frames[0].size, 3U);
  EXPECT_EQ(frames[0].data[2], 0xCCU);
}

TEST(FrameRecordParser, RejectsMalformedRecordWithoutLosingNextRecord) {
  transport::FrameRecordParser parser;
  const auto frames = parser.push("d1231GG\rd1231AA\r");
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].data[0], 0xAAU);
  EXPECT_EQ(parser.rejected_records(), 1U);
}

TEST(FrameRecordParser, DropsOverlongRecordAndRecoversAtTerminator) {
  transport::FrameRecordParser parser;
  EXPECT_TRUE(parser.push(std::string(257U, 'A')).empty());
  const auto frames = parser.push("\rt1231FF\r");
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].id, 0x123U);
  EXPECT_EQ(frames[0].data[0], 0xFFU);
  EXPECT_EQ(parser.rejected_records(), 1U);
}
