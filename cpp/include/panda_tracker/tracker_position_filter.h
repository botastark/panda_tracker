#pragma once

#include "panda_tracker/position_tracking_config.h"
#include "panda_tracker/tracker_receiver.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace panda_tracker {

enum class FilterUpdate {
  kNoNewPacket,
  kInvalidPacket,
  kWarming,
  kAccepted,
  kRejectedJump,
};

struct FilteredCameraPose {
  Transform T_CT{};
  std::chrono::steady_clock::time_point arrival{};
  std::uint64_t sequence{0};
  bool available{false};
};

class TrackerPositionFilter {
 public:
  TrackerPositionFilter(
      TrackerFilterConfig config,
      Matrix3 fixed_R_CT);

  FilterUpdate process(
      const TrackerSnapshot& snapshot);

  const FilteredCameraPose& output() const {
    return output_;
  }

  std::size_t raw_window_size() const {
    return raw_positions_.size();
  }

  std::size_t accepted_packets() const {
    return accepted_packets_;
  }

  std::size_t rejected_jumps() const {
    return rejected_jumps_;
  }

  std::size_t rejected_invalid() const {
    return rejected_invalid_;
  }

 private:
  Vector3 coordinate_median() const;

  TrackerFilterConfig config_{};
  Matrix3 fixed_R_CT_{};

  std::deque<Vector3> raw_positions_{};
  std::optional<Vector3> filtered_position_{};

  bool have_seen_sequence_{false};
  std::uint64_t seen_sequence_{0};

  FilteredCameraPose output_{};

  std::size_t accepted_packets_{0};
  std::size_t rejected_jumps_{0};
  std::size_t rejected_invalid_{0};
};

}  // namespace panda_tracker
