#pragma once

#include "panda_tracker/position_tracking_config.h"

#include <franka/robot.h>

#include <array>
#include <cstddef>
#include <string>

namespace panda_tracker {

using Wrench6 = std::array<double, 6>;

enum class StopReason {
  kNone,
  kSignal,
  kArmTimeout,
  kRuntime,
  kTrackingLost,
  kCommandStale,
  kTravelEnvelope,
  kBrakingBoundary,
  kJointSpeed,
  kJointTorque,
  kExternalJointTorque,
  kJointTorqueRate,
  kExternalWrench,
  kRobotContact,
  kCommunication,
  kWorkerFailure,
};

const char* stop_reason_text(StopReason reason);

enum class DerateSource {
  kNone,
  kJointTorque,
  kExternalJointTorque,
  kJointTorqueRate,
  kExternalForce,
  kExternalTorque,
};

const char* derate_source_text(DerateSource source);

struct StopDiagnostics {
  Vector3 travel_B_m{};
  double rotation_rad{0.0};
  double external_force_n{0.0};
  double external_torque_nm{0.0};
  std::size_t joint_index{0};
  double joint_value{0.0};

  // Filled only if the controlled stop reaches max_stop_time_s.
  Vector3 timeout_limited_velocity_B_mps{};
  Vector3 timeout_last_commanded_velocity_B_mps{};
  Vector3 timeout_last_commanded_acceleration_B_mps2{};

  std::array<double, 7> timeout_limited_qdot_radps{};
  std::array<double, 7> timeout_dq_d_radps{};
  std::array<double, 7> timeout_ddq_d_radps2{};

  double timeout_elapsed_s{0.0};
  bool forced_finish_after_stop_timeout{false};
};

Transform physical_flange_pose(const franka::RobotState& state);

void apply_and_verify_load_model(
    franka::Robot& robot,
    const PositionTrackingConfig& config);

Wrench6 acquire_wrench_bias(
    franka::Robot& robot,
    const PositionTrackingConfig& config);

bool preflight_ok(
    const franka::RobotState& state,
    const PositionTrackingConfig& config,
    const Wrench6& wrench_bias,
    std::string& error);

void print_preflight_measurements(
    const franka::RobotState& state,
    const Wrench6& wrench_bias);

struct RuntimeSafetyResult {
  StopReason reason{StopReason::kNone};
  double speed_scale{1.0};

  // Diagnostic for whichever monitored quantity currently produces the
  // smallest speed scale. No safety threshold is changed by these fields.
  DerateSource derate_source{DerateSource::kNone};
  std::size_t derate_joint_index{0};
  double derate_value{0.0};
  double derate_limit{0.0};
  double derate_ratio{0.0};

  StopDiagnostics diagnostics{};
};

class RuntimeSafetyMonitor {
 public:
  RuntimeSafetyMonitor(
      PositionTrackingConfig config,
      Wrench6 wrench_bias);

  RuntimeSafetyResult update(
      const franka::RobotState& state,
      const Transform& T_BF,
      double control_elapsed_s);

 private:
  PositionTrackingConfig config_{};
  Wrench6 wrench_bias_{};

  bool startup_pose_set_{false};
  Transform startup_T_BF_{};
  std::size_t torque_bad_cycles_{0};
};

bool moving_toward_soft_travel_limit(
    const Vector3& velocity_B_mps,
    const Vector3& travel_B_m,
    const PositionTrackingConfig& config);

}  // namespace panda_tracker
