#include "panda_tracker/task_pose_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace panda_tracker {
namespace {

constexpr std::array<std::uint8_t, 4> kMagic{{'P', 'T', 'P', '2'}};
constexpr std::size_t kMagicOffset = 0;
constexpr std::size_t kVersionOffset = 4;
constexpr std::size_t kValidOffset = 5;
constexpr std::size_t kReservedOffset = 6;
constexpr std::size_t kSequenceOffset = 8;
constexpr std::size_t kConfidenceOffset = 16;
constexpr std::size_t kTransformOffset = 20;

template <typename T>
T load_value(const std::uint8_t* data, std::size_t offset) {
  T value{};
  std::memcpy(&value, data + offset, sizeof(T));
  return value;
}

template <typename T>
void store_value(std::uint8_t* data, std::size_t offset, const T& value) {
  std::memcpy(data + offset, &value, sizeof(T));
}

}  // namespace

bool host_is_little_endian() {
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 1;
}

DecodeStatus decode_task_pose(
    const std::uint8_t* data,
    std::size_t size,
    TaskPosePacket& packet) {
  if (!host_is_little_endian()) {
    return DecodeStatus::kUnsupportedHostEndianness;
  }
  if (data == nullptr || size != kTaskPosePacketSize) {
    return DecodeStatus::kWrongSize;
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), data + kMagicOffset)) {
    return DecodeStatus::kWrongMagic;
  }
  if (data[kVersionOffset] != kTaskPoseVersion) {
    return DecodeStatus::kWrongVersion;
  }

  const std::uint8_t valid_value = data[kValidOffset];
  if (valid_value > 1) {
    return DecodeStatus::kInvalidValidFlag;
  }
  if (load_value<std::uint16_t>(data, kReservedOffset) != 0) {
    return DecodeStatus::kReservedFieldNonZero;
  }

  TaskPosePacket decoded{};
  decoded.sequence_id = load_value<std::uint64_t>(data, kSequenceOffset);
  decoded.confidence = load_value<float>(data, kConfidenceOffset);
  decoded.valid = valid_value == 1;

  if (!std::isfinite(decoded.confidence) ||
      decoded.confidence < 0.0F ||
      decoded.confidence > 1.0F) {
    return DecodeStatus::kInvalidConfidence;
  }

  for (std::size_t i = 0; i < decoded.T_CT.size(); ++i) {
    decoded.T_CT[i] = load_value<double>(
        data, kTransformOffset + i * sizeof(double));
  }

  if (!finite_rigid_transform(decoded.T_CT)) {
    return DecodeStatus::kInvalidTransform;
  }

  packet = decoded;
  return DecodeStatus::kOk;
}

std::array<std::uint8_t, kTaskPosePacketSize> encode_task_pose(
    const TaskPosePacket& packet) {
  if (!host_is_little_endian()) {
    throw std::runtime_error(
        "PTP2 encoding currently requires a little-endian host.");
  }
  if (!std::isfinite(packet.confidence) ||
      packet.confidence < 0.0F ||
      packet.confidence > 1.0F) {
    throw std::invalid_argument(
        "Task-pose confidence must be in [0,1].");
  }
  if (!finite_rigid_transform(packet.T_CT)) {
    throw std::invalid_argument(
        "T_CT must be a finite rigid transform.");
  }

  std::array<std::uint8_t, kTaskPosePacketSize> data{};
  std::copy(kMagic.begin(), kMagic.end(), data.begin() + kMagicOffset);
  data[kVersionOffset] = kTaskPoseVersion;
  data[kValidOffset] = packet.valid ? 1 : 0;

  store_value<std::uint16_t>(data.data(), kReservedOffset, 0);
  store_value<std::uint64_t>(
      data.data(), kSequenceOffset, packet.sequence_id);
  store_value<float>(
      data.data(), kConfidenceOffset, packet.confidence);

  for (std::size_t i = 0; i < packet.T_CT.size(); ++i) {
    store_value<double>(
        data.data(),
        kTransformOffset + i * sizeof(double),
        packet.T_CT[i]);
  }

  return data;
}

const char* decode_status_message(DecodeStatus status) {
  switch (status) {
    case DecodeStatus::kOk: return "ok";
    case DecodeStatus::kUnsupportedHostEndianness:
      return "unsupported_host_endianness";
    case DecodeStatus::kWrongSize: return "wrong_size";
    case DecodeStatus::kWrongMagic: return "wrong_magic";
    case DecodeStatus::kWrongVersion: return "wrong_version";
    case DecodeStatus::kInvalidValidFlag: return "invalid_valid_flag";
    case DecodeStatus::kReservedFieldNonZero:
      return "reserved_field_nonzero";
    case DecodeStatus::kInvalidConfidence: return "invalid_confidence";
    case DecodeStatus::kInvalidTransform: return "invalid_transform";
  }
  return "unknown";
}

}  // namespace panda_tracker
