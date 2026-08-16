#include "panda_tracker/task_pose_protocol.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using panda_tracker::DecodeStatus;
using panda_tracker::TaskPosePacket;

TaskPosePacket identity_packet() {
  TaskPosePacket packet{};
  packet.T_TS = {{
      1.0, 0.0, 0.0, 0.1,
      0.0, 1.0, 0.0, -0.2,
      0.0, 0.0, 1.0, 0.3,
      0.0, 0.0, 0.0, 1.0,
  }};
  packet.sequence_id = 42;
  packet.confidence = 0.75F;
  packet.valid = true;
  return packet;
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace

int main() {
  require(
      panda_tracker::host_is_little_endian(),
      "the current protocol implementation requires little endian");

  const TaskPosePacket original = identity_packet();
  auto bytes = panda_tracker::encode_task_pose(original);

  require(
      bytes.size() == panda_tracker::kTaskPosePacketSize,
      "encoded packet size");
  require(
      bytes[0] == 'P' && bytes[1] == 'T' &&
          bytes[2] == 'P' && bytes[3] == '2',
      "encoded magic");
  require(bytes[4] == panda_tracker::kTaskPoseVersion, "encoded version");
  require(bytes[5] == 1, "encoded valid flag");
  require(bytes[6] == 0 && bytes[7] == 0, "encoded reserved field");

  TaskPosePacket decoded{};
  require(
      panda_tracker::decode_task_pose(bytes.data(), bytes.size(), decoded) ==
          DecodeStatus::kOk,
      "round-trip decode");
  require(decoded.sequence_id == original.sequence_id, "sequence round trip");
  require(decoded.confidence == original.confidence, "confidence round trip");
  require(decoded.valid == original.valid, "valid flag round trip");
  require(decoded.T_TS == original.T_TS, "transform round trip");

  require(
      panda_tracker::decode_task_pose(
          bytes.data(), bytes.size() - 1, decoded) == DecodeStatus::kWrongSize,
      "wrong size rejection");

  auto invalid_magic = bytes;
  invalid_magic[0] = 'X';
  require(
      panda_tracker::decode_task_pose(
          invalid_magic.data(), invalid_magic.size(), decoded) ==
          DecodeStatus::kWrongMagic,
      "wrong magic rejection");

  auto invalid_version = bytes;
  invalid_version[4] = 99;
  require(
      panda_tracker::decode_task_pose(
          invalid_version.data(), invalid_version.size(), decoded) ==
          DecodeStatus::kWrongVersion,
      "wrong version rejection");

  auto invalid_reserved = bytes;
  invalid_reserved[6] = 1;
  require(
      panda_tracker::decode_task_pose(
          invalid_reserved.data(), invalid_reserved.size(), decoded) ==
          DecodeStatus::kReservedFieldNonZero,
      "reserved field rejection");

  auto invalid_transform = bytes;
  const double bad_bottom_row = 2.0;
  std::memcpy(
      invalid_transform.data() + 20 + 15 * sizeof(double),
      &bad_bottom_row,
      sizeof(double));
  require(
      panda_tracker::decode_task_pose(
          invalid_transform.data(), invalid_transform.size(), decoded) ==
          DecodeStatus::kInvalidTransform,
      "invalid transform rejection");

  TaskPosePacket invalid_confidence = identity_packet();
  invalid_confidence.confidence =
      std::numeric_limits<float>::quiet_NaN();
  bool threw = false;
  try {
    (void)panda_tracker::encode_task_pose(invalid_confidence);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "non-finite confidence rejection");

  std::cout << "PASS: PTP2 protocol tests\n";
  return EXIT_SUCCESS;
}
