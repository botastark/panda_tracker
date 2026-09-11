#include "panda_tracker/position_servo.h"

#include <cmath>

namespace panda_tracker {
namespace {

Transform translation_only(const Vector3& p) {
  Transform T = identity_transform();
  T[3] = p[0];
  T[7] = p[1];
  T[11] = p[2];
  return T;
}

Vector3 subtract(const Vector3& a, const Vector3& b) {
  return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}

Vector3 scale(const Vector3& a, double s) {
  return {{a[0] * s, a[1] * s, a[2] * s}};
}

bool finite_vector(const Vector3& a) {
  return std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}

}  // namespace

PositionServo::PositionServo(const PositionTrackingConfig& config)
    : axes_(config.control_axes),
      kp_position_(config.kp_position),
      max_linear_speed_mps_(config.max_linear_speed_mps),
      T_FS_(config.T_FS),
      T_CS_(config.T_CS),
      T_TS_des_(config.T_TS_des) {}

Matrix3 PositionServo::reference_camera_rotation_R_CT() const {
  // Current geometry: T_TS = T_TC * T_CS.
  // Desired camera pose relative to target:
  //   T_TC_des = T_TS_des * inverse(T_CS)
  // and therefore:
  //   T_CT_des = inverse(T_TC_des)
  const Transform T_TC_des =
      multiply_transform(T_TS_des_, invert_transform(T_CS_));
  const Transform T_CT_des = invert_transform(T_TC_des);
  return transform_rotation(T_CT_des);
}

PositionServoResult PositionServo::compute(
    const Transform& T_BF,
    const Transform& filtered_T_CT) const {
  PositionServoResult result{};

  if (!finite_rigid_transform(T_BF) ||
      !finite_rigid_transform(filtered_T_CT)) {
    return result;
  }

  // Tracker -> physical stick relation, entirely on the robot side.
  const Transform T_TC = invert_transform(filtered_T_CT);
  result.T_TS = multiply_transform(T_TC, T_CS_);

  // Translation-only PBVS error in the current stick frame.
  Transform delta_T_S =
      multiply_transform(invert_transform(result.T_TS), T_TS_des_);

  // Explicitly remove orientation before changing reference point. This avoids
  // any rotational lever-arm contribution to the translational command.
  delta_T_S = translation_only(transform_translation(delta_T_S));

  // Move the error from stick reference point S to physical flange F.
  const Transform delta_T_F =
      multiply_transform(
          multiply_transform(T_FS_, delta_T_S),
          invert_transform(T_FS_));

  const Transform T_BF_goal = multiply_transform(T_BF, delta_T_F);

  result.error_B_m = subtract(
      transform_translation(T_BF_goal),
      transform_translation(T_BF));

  result.active_error_B_m = result.error_B_m;
  if (!axes_.x) result.active_error_B_m[0] = 0.0;
  if (!axes_.y) result.active_error_B_m[1] = 0.0;
  if (!axes_.z) result.active_error_B_m[2] = 0.0;

  result.active_error_norm_m = vector_norm(result.active_error_B_m);

  result.velocity_B_mps =
      clamp_norm(scale(result.active_error_B_m, kp_position_),
                 max_linear_speed_mps_);

  result.valid =
      finite_vector(result.error_B_m) &&
      finite_vector(result.velocity_B_mps) &&
      std::isfinite(result.active_error_norm_m);

  return result;
}

}  // namespace panda_tracker
