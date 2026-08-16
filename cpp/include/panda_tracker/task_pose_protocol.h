#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace panda_tracker {

constexpr std::size_t kTaskPosePacketSize = 148;
constexpr std::uint8_t kTaskPoseVersion = 2;

struct TaskPosePacket {
  // Row-major T_TS: stick-tip frame S expressed in target frame T.
  std::array<double, 16> T_TS{};
  std::uint64_t sequence_id{0};
  float confidence{0.0F};
  bool valid{false};
};

enum class DecodeStatus {
  kOk,
  kUnsupportedHostEndianness,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kInvalidValidFlag,
  kReservedFieldNonZero,
  kInvalidConfidence,
  kInvalidTransform,
};

bool host_is_little_endian();

bool is_finite_rigid_transform(
    const std::array<double, 16>& transform,
    double tolerance = 1e-6);

DecodeStatus decode_task_pose(
    const std::uint8_t* data,
    std::size_t size,
    TaskPosePacket& packet);

std::array<std::uint8_t, kTaskPosePacketSize> encode_task_pose(
    const TaskPosePacket& packet);

const char* decode_status_message(DecodeStatus status);

}  // namespace panda_tracker
