#ifndef UBLOX_GPS_RTCM_FRAME_HPP
#define UBLOX_GPS_RTCM_FRAME_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ublox_gps {

constexpr std::size_t kRtcm3HeaderSize = 3;
constexpr std::size_t kRtcm3CrcSize = 3;
constexpr std::size_t kRtcm3MaximumPayloadSize = 1023;
constexpr std::size_t kRtcm3MaximumFrameSize =
    kRtcm3HeaderSize + kRtcm3MaximumPayloadSize + kRtcm3CrcSize;

enum class Rtcm3BatchError {
  NONE,
  EMPTY,
  BATCH_TOO_LARGE,
  TRUNCATED_HEADER,
  BAD_PREAMBLE,
  RESERVED_BITS_SET,
  TRUNCATED_FRAME,
  CRC_MISMATCH,
};

struct Rtcm3BatchValidation {
  Rtcm3BatchError error{Rtcm3BatchError::NONE};
  std::size_t frame_count{0};
  std::size_t error_offset{0};

  explicit operator bool() const {
    return error == Rtcm3BatchError::NONE;
  }
};

inline uint32_t crc24q(const uint8_t* data, std::size_t size) {
  constexpr uint32_t polynomial = 0x1864CFB;
  uint32_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1;
      if ((crc & 0x1000000U) != 0U) {
        crc ^= polynomial;
      }
    }
  }
  return crc & 0xFFFFFFU;
}

inline Rtcm3BatchValidation validateRtcm3Batch(
    const std::vector<uint8_t>& data,
    std::size_t maximum_batch_size) {
  if (data.empty()) {
    return {Rtcm3BatchError::EMPTY, 0, 0};
  }
  if (data.size() > maximum_batch_size) {
    return {Rtcm3BatchError::BATCH_TOO_LARGE, 0, 0};
  }

  std::size_t offset = 0;
  std::size_t frame_count = 0;
  while (offset < data.size()) {
    const std::size_t remaining = data.size() - offset;
    if (remaining < kRtcm3HeaderSize) {
      return {Rtcm3BatchError::TRUNCATED_HEADER, frame_count, offset};
    }
    if (data[offset] != 0xD3U) {
      return {Rtcm3BatchError::BAD_PREAMBLE, frame_count, offset};
    }
    if ((data[offset + 1] & 0xFCU) != 0U) {
      return {Rtcm3BatchError::RESERVED_BITS_SET, frame_count, offset};
    }

    const std::size_t payload_size =
        (static_cast<std::size_t>(data[offset + 1] & 0x03U) << 8) |
        static_cast<std::size_t>(data[offset + 2]);
    const std::size_t frame_size =
        kRtcm3HeaderSize + payload_size + kRtcm3CrcSize;
    if (remaining < frame_size) {
      return {Rtcm3BatchError::TRUNCATED_FRAME, frame_count, offset};
    }

    const std::size_t crc_offset = offset + frame_size - kRtcm3CrcSize;
    const uint32_t expected_crc =
        (static_cast<uint32_t>(data[crc_offset]) << 16) |
        (static_cast<uint32_t>(data[crc_offset + 1]) << 8) |
        static_cast<uint32_t>(data[crc_offset + 2]);
    const uint32_t actual_crc =
        crc24q(data.data() + offset, frame_size - kRtcm3CrcSize);
    if (actual_crc != expected_crc) {
      return {Rtcm3BatchError::CRC_MISMATCH, frame_count, offset};
    }

    offset += frame_size;
    ++frame_count;
  }

  return {Rtcm3BatchError::NONE, frame_count, offset};
}

inline const char* rtcm3BatchErrorString(Rtcm3BatchError error) {
  switch (error) {
    case Rtcm3BatchError::NONE:
      return "valid RTCM3 frame batch";
    case Rtcm3BatchError::EMPTY:
      return "empty RTCM3 batch";
    case Rtcm3BatchError::BATCH_TOO_LARGE:
      return "RTCM3 batch exceeds configured byte limit";
    case Rtcm3BatchError::TRUNCATED_HEADER:
      return "truncated RTCM3 header";
    case Rtcm3BatchError::BAD_PREAMBLE:
      return "RTCM3 preamble is not 0xD3";
    case Rtcm3BatchError::RESERVED_BITS_SET:
      return "RTCM3 reserved header bits are non-zero";
    case Rtcm3BatchError::TRUNCATED_FRAME:
      return "RTCM3 payload length exceeds remaining batch bytes";
    case Rtcm3BatchError::CRC_MISMATCH:
      return "RTCM3 CRC24Q mismatch";
  }
  return "unknown RTCM3 validation error";
}

}  // namespace ublox_gps

#endif  // UBLOX_GPS_RTCM_FRAME_HPP
