#pragma once

#include "panda_tracker/position_tracking_config.h"

namespace panda_tracker {

struct PositionServoResult {
  Transform T_TS{};
  Vector3 error_B_m{};
  Vector3 active_error_B_m{};
  Vector3 velocity_B_mps{};
  double active_error_norm_m{0.0};
  bool valid{false};
};

class PositionServo {
 public:
  explicit PositionServo(const PositionTrackingConfig& config);

  // Fixed camera-target orientation used by the translation filter. It is
  // derived from the desired target-stick relation and camera-stick geometry.
  Matrix3 reference_camera_rotation_R_CT() const;

  PositionServoResult compute(
      const Transform& T_BF,
      const Transform& filtered_T_CT) const;

 private:
  AxisMask axes_{};
  double kp_position_{1.0};
  double max_linear_speed_mps_{0.0005};
  Vector3 diagnostic_position_bias_B_m_{{0.0, 0.0, 0.0}};

  Transform T_FS_{};
  Transform T_CS_{};
  Transform T_TS_des_{};
};

}  // namespace panda_tracker
