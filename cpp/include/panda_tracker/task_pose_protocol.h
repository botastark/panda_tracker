#pragma once

#include "panda_tracker/geometry.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace panda_tracker {

constexpr std::size_t kTaskPosePacketSize = 148;
constexpr std::uint8_t kTaskPoseVersion = 2;

// PTP2 wire packet.
//
// IMPORTANT: The 16-double matrix field is T_CT:
// target frame T expressed in camera frame C.
//
// The binary layout is unchanged from the existing PTP2 packet:
//   magic/version/valid/reserved/sequence/confidence/16 doubles.
//
// Renaming the C++ field to T_CT removes the old misleading robot-side T_TS
// name without changing the UDP protocol.
struct TaskPosePacket {
  Transform T_CT{};
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

DecodeStatus decode_task_pose(
    const std::uint8_t* data,
    std::size_t size,
    TaskPosePacket& packet);

std::array<std::uint8_t, kTaskPosePacketSize> encode_task_pose(
    const TaskPosePacket& packet);

const char* decode_status_message(DecodeStatus status);

}  // namespace panda_tracker
