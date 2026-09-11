#include "panda_tracker/tracker_position_filter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace panda_tracker {
namespace {

Transform make_transform(const Matrix3& R, const Vector3& p) {
  Transform T = identity_transform();
  T[0] = R[0]; T[1] = R[1]; T[2] = R[2];
  T[4] = R[3]; T[5] = R[4]; T[6] = R[5];
  T[8] = R[6]; T[9] = R[7]; T[10] = R[8];
  T[3] = p[0]; T[7] = p[1]; T[11] = p[2];
  return T;
}

bool finite_vector(const Vector3& p) {
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

Vector3 subtract(const Vector3& a, const Vector3& b) {
  return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}

}  // namespace

TrackerPositionFilter::TrackerPositionFilter(
    TrackerFilterConfig config,
    Matrix3 fixed_R_CT)
    : config_(config), fixed_R_CT_(fixed_R_CT) {
  if (config_.median_window_samples < 3 ||
      config_.median_window_samples % 2 == 0) {
    throw std::invalid_argument("median window must be odd and >= 3");
  }
  if (!(config_.ema_alpha > 0.0 && config_.ema_alpha <= 1.0)) {
    throw std::invalid_argument("EMA alpha must be in (0, 1]");
  }
  if (!(config_.max_filtered_jump_m > 0.0)) {
    throw std::invalid_argument("filtered jump limit must be positive");
  }
}

Vector3 TrackerPositionFilter::coordinate_median() const {
  Vector3 median{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    values.reserve(raw_positions_.size());
    for (const auto& p : raw_positions_) values.push_back(p[axis]);
    std::sort(values.begin(), values.end());
    median[axis] = values[values.size() / 2];
  }
  return median;
}

FilterUpdate TrackerPositionFilter::process(const TrackerSnapshot& snapshot) {
  if (!snapshot.available) return FilterUpdate::kNoNewPacket;

  const std::uint64_t sequence = snapshot.packet.sequence_id;
  if (have_seen_sequence_ && sequence == seen_sequence_) {
    return FilterUpdate::kNoNewPacket;
  }
  have_seen_sequence_ = true;
  seen_sequence_ = sequence;

  if (!snapshot.packet.valid) {
    ++rejected_invalid_;
    return FilterUpdate::kInvalidPacket;
  }

  const Transform& raw_T_CT = snapshot.packet.T_CT;
  const Vector3 raw_p_CT = transform_translation(raw_T_CT);
  if (!finite_vector(raw_p_CT)) {
    ++rejected_invalid_;
    return FilterUpdate::kInvalidPacket;
  }

  raw_positions_.push_back(raw_p_CT);
  while (raw_positions_.size() > config_.median_window_samples) {
    raw_positions_.pop_front();
  }

  if (raw_positions_.size() < config_.median_window_samples) {
    return FilterUpdate::kWarming;
  }

  const Vector3 median_p = coordinate_median();

  Vector3 candidate = median_p;
  if (filtered_position_) {
    const double a = config_.ema_alpha;
    for (std::size_t i = 0; i < 3; ++i) {
      candidate[i] =
          (1.0 - a) * (*filtered_position_)[i] + a * median_p[i];
    }

    // Jump rejection is deliberately AFTER median + EMA filtering.
    const double jump = vector_norm(subtract(candidate, *filtered_position_));
    if (jump > config_.max_filtered_jump_m) {
      // Do not let a rejected raw point remain in the robust window.
      raw_positions_.pop_back();
      ++rejected_jumps_;
      return FilterUpdate::kRejectedJump;
    }
  }

  filtered_position_ = candidate;

  // Translation comes from the tracker after robust filtering.
  // Orientation is fixed because PnP orientation is intentionally ignored in
  // the current position-only experiment.
  output_.T_CT = make_transform(fixed_R_CT_, candidate);
  output_.arrival = snapshot.arrival;
  output_.sequence = sequence;
  output_.available = true;

  ++accepted_packets_;
  return FilterUpdate::kAccepted;
}

}  // namespace panda_tracker
