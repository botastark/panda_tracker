#pragma once

#include <array>

namespace panda_tracker {

using Twist6 = std::array<double, 6>;
using Vector3Velocity = std::array<double, 3>;
using JointVector7 = std::array<double, 7>;
using Jacobian6x7 = std::array<double, 42>;

struct JointVelocityMappingResult {
  JointVector7 qdot_radps{};
  Twist6 achieved_twist{};
  bool valid{false};
};

// Position-only PBVS mapping.
//
// Only the translational rows of the 6x7 Franka zero Jacobian are inverted:
//
//   qdot = Jv^T (Jv Jv^T + lambda^2 I)^-1 v
//
// where Jv is 3x7. This intentionally does NOT constrain angular velocity,
// because target/orientation control is disabled in this experiment.
//
// desired_velocity_B_mps[2] may be zero to request constant base-frame Z
// velocity. The achieved full 6D twist is returned for diagnostics.
JointVelocityMappingResult map_translation_to_joint_velocity(
    const Jacobian6x7& jacobian_column_major,
    const Vector3Velocity& desired_velocity_B_mps,
    double damping);

Twist6 jacobian_times_joint_velocity(
    const Jacobian6x7& jacobian_column_major,
    const JointVector7& qdot_radps);

double max_abs_joint_value(const JointVector7& values);

}  // namespace panda_tracker
