#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <ublox_gps/rtcm_frame.hpp>

namespace {

std::vector<uint8_t> makeFrame(const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame{
      0xD3,
      static_cast<uint8_t>((payload.size() >> 8) & 0x03U),
      static_cast<uint8_t>(payload.size() & 0xFFU)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint32_t crc = ublox_gps::crc24q(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  return frame;
}

TEST(RtcmFrame, Crc24QMatchesPublishedCheckValue) {
  const std::vector<uint8_t> input{
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(ublox_gps::crc24q(input.data(), input.size()), 0xCDE703U);
}

TEST(RtcmFrame, AcceptsOneCompleteFrame) {
  const auto frame = makeFrame({0x3E, 0xD0, 0x00, 0x01});
  const auto result = ublox_gps::validateRtcm3Batch(frame, 8192);
  EXPECT_TRUE(result);
  EXPECT_EQ(result.frame_count, 1U);
}

TEST(RtcmFrame, AcceptsMultipleConcatenatedFrames) {
  auto batch = makeFrame({0x3E, 0xD0, 0x00, 0x01});
  const auto second = makeFrame({0x43, 0x50, 0x12, 0x34, 0x56});
  batch.insert(batch.end(), second.begin(), second.end());
  const auto result = ublox_gps::validateRtcm3Batch(batch, 8192);
  EXPECT_TRUE(result);
  EXPECT_EQ(result.frame_count, 2U);
}

TEST(RtcmFrame, RejectsArbitraryOrTruncatedBytes) {
  EXPECT_EQ(
      ublox_gps::validateRtcm3Batch({0xB5, 0x62, 0x06, 0x04}, 8192).error,
      ublox_gps::Rtcm3BatchError::BAD_PREAMBLE);

  auto frame = makeFrame({0x3E, 0xD0, 0x00, 0x01});
  frame.pop_back();
  EXPECT_EQ(
      ublox_gps::validateRtcm3Batch(frame, 8192).error,
      ublox_gps::Rtcm3BatchError::TRUNCATED_FRAME);
}

TEST(RtcmFrame, RejectsReservedBitsBadCrcAndOversizeBatch) {
  auto reserved = makeFrame({0x3E, 0xD0});
  reserved[1] |= 0x04;
  EXPECT_EQ(
      ublox_gps::validateRtcm3Batch(reserved, 8192).error,
      ublox_gps::Rtcm3BatchError::RESERVED_BITS_SET);

  auto corrupt = makeFrame({0x3E, 0xD0});
  corrupt[3] ^= 0x01;
  EXPECT_EQ(
      ublox_gps::validateRtcm3Batch(corrupt, 8192).error,
      ublox_gps::Rtcm3BatchError::CRC_MISMATCH);

  const auto valid = makeFrame({0x3E, 0xD0});
  EXPECT_EQ(
      ublox_gps::validateRtcm3Batch(valid, valid.size() - 1).error,
      ublox_gps::Rtcm3BatchError::BATCH_TOO_LARGE);
}

}  // namespace
