#pragma once

#include "panda_tracker/task_pose_protocol.h"

#include <chrono>
#include <cstdint>
#include <string>

namespace panda_tracker {

struct TrackerSnapshot {
  TaskPosePacket packet{};
  std::chrono::steady_clock::time_point arrival{};
  std::string source_ip{};
  std::uint16_t source_port{0};
  bool available{false};
};

class TrackerReceiver {
 public:
  TrackerReceiver(
      const std::string& bind_ip,
      const std::string& expected_source_ip,
      std::uint16_t port);

  TrackerReceiver(const TrackerReceiver&) = delete;
  TrackerReceiver& operator=(const TrackerReceiver&) = delete;
  TrackerReceiver(TrackerReceiver&&) = delete;
  TrackerReceiver& operator=(TrackerReceiver&&) = delete;

  ~TrackerReceiver();

  // Drain all currently available UDP datagrams without blocking.
  void poll();

  TrackerSnapshot latest() const;

  std::uint64_t accepted_packets() const;
  std::uint64_t rejected_packets() const;
  std::uint64_t duplicate_or_old_packets() const;
  std::uint64_t wrong_source_packets() const;
  DecodeStatus last_decode_error() const;

 private:
  int socket_fd_{-1};
  std::string expected_source_ip_;
  TrackerSnapshot snapshot_{};
  std::uint64_t accepted_packets_{0};
  std::uint64_t rejected_packets_{0};
  std::uint64_t duplicate_or_old_packets_{0};
  std::uint64_t wrong_source_packets_{0};
  DecodeStatus last_decode_error_{DecodeStatus::kOk};
};

}  // namespace panda_tracker
