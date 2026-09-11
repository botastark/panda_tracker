#include "panda_tracker/robot_safety.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace panda_tracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double value) {
  return value * kPi / 180.0;
}

double norm3(double a, double b, double c) {
  return std::sqrt(a * a + b * b + c * c);
}

template <std::size_t N>
bool any_positive(const std::array<double, N>& values) {
  return std::any_of(
      values.begin(), values.end(),
      [](double value) { return value > 0.0; });
}

Wrench6 wrench_delta(
    const franka::RobotState& state,
    const Wrench6& bias) {
  Wrench6 delta{};
  for (std::size_t i = 0; i < 6; ++i) {
    delta[i] = state.O_F_ext_hat_K[i] - bias[i];
  }
  return delta;
}

double angular_distance_rad(
    const Transform& a,
    const Transform& b) {
  const Transform delta =
      multiply_transform(invert_transform(a), b);
  return vector_norm(so3_log(transform_rotation(delta)));
}

}  // namespace

const char* stop_reason_text(StopReason reason) {
  switch (reason) {
    case StopReason::kNone: return "none";
    case StopReason::kSignal: return "signal requested";
    case StopReason::kArmTimeout: return "arming timeout";
    case StopReason::kRuntime: return "motion runtime reached";
    case StopReason::kTrackingLost: return "tracking lost after arming";
    case StopReason::kCommandStale: return "servo command stale";
    case StopReason::kTravelEnvelope: return "physical flange travel envelope exceeded";
    case StopReason::kBrakingBoundary: return "travel braking boundary reached";
    case StopReason::kJointSpeed: return "joint speed limit exceeded";
    case StopReason::kJointTorque: return "measured joint torque limit exceeded";
    case StopReason::kExternalJointTorque: return "external joint torque limit exceeded";
    case StopReason::kJointTorqueRate: return "joint torque-rate limit exceeded";
    case StopReason::kExternalWrench: return "external Cartesian wrench limit exceeded";
    case StopReason::kRobotContact: return "robot contact/collision signal";
    case StopReason::kCommunication: return "control command success rate too low";
    case StopReason::kWorkerFailure: return "worker failure";
  }
  return "unknown";
}

const char* derate_source_text(DerateSource source) {
  switch (source) {
    case DerateSource::kNone: return "none";
    case DerateSource::kJointTorque: return "joint_torque";
    case DerateSource::kExternalJointTorque: return "external_joint_torque";
    case DerateSource::kJointTorqueRate: return "joint_torque_rate";
    case DerateSource::kExternalForce: return "external_force";
    case DerateSource::kExternalTorque: return "external_torque";
  }
  return "unknown";
}

Transform physical_flange_pose(const franka::RobotState& state) {
  const Transform O_T_EE =
      franka_column_major_transform(state.O_T_EE);
  const Transform F_T_EE =
      franka_column_major_transform(state.F_T_EE);
  return multiply_transform(O_T_EE, invert_transform(F_T_EE));
}

void apply_and_verify_load_model(
    franka::Robot& robot,
    const PositionTrackingConfig& config) {
  if (!config.load.enabled) return;

  const franka::RobotState before = robot.readOnce();
  if (config.load.includes_end_effector &&
      before.m_ee > config.load.max_preexisting_ee_mass_kg) {
    throw std::runtime_error(
        "Refusing combined Hand+tool load because robot already reports m_ee=" +
        std::to_string(before.m_ee) + " kg");
  }

  std::cout << std::setprecision(9)
            << "Applying flange load: mass=" << config.load.mass_kg
            << " kg COM_F=["
            << config.load.com_F_m[0] << ", "
            << config.load.com_F_m[1] << ", "
            << config.load.com_F_m[2] << "] m\n";

  robot.setLoad(
      config.load.mass_kg,
      config.load.com_F_m,
      config.load.inertia_at_com_F_kg_m2);

  std::this_thread::sleep_for(
      std::chrono::duration<double>(config.load.settle_s));

  const franka::RobotState after = robot.readOnce();
  constexpr double kMassToleranceKg = 1e-5;
  constexpr double kComToleranceM = 1e-5;

  if (std::abs(after.m_load - config.load.mass_kg) > kMassToleranceKg) {
    throw std::runtime_error("Robot did not report requested load mass after setLoad");
  }
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::abs(after.F_x_Cload[i] - config.load.com_F_m[i]) > kComToleranceM) {
      throw std::runtime_error("Robot did not report requested load COM after setLoad");
    }
  }

  std::cout << "Configured load model accepted by robot.\n";
}

Wrench6 acquire_wrench_bias(
    franka::Robot& robot,
    const PositionTrackingConfig& config) {
  Wrench6 bias{};
  if (!config.wrench_bias_enabled) return bias;

  std::this_thread::sleep_for(
      std::chrono::duration<double>(config.wrench_bias_settle_s));

  for (std::size_t sample = 0;
       sample < config.wrench_bias_samples;
       ++sample) {
    const franka::RobotState state = robot.readOnce();

    for (std::size_t i = 0; i < 7; ++i) {
      if (std::abs(state.dq[i]) > config.max_start_joint_speed_radps) {
        throw std::runtime_error("Cannot learn wrench bias while robot is moving");
      }
      if (std::abs(state.tau_ext_hat_filtered[i]) >
          config.max_abs_external_joint_torque_nm[i]) {
        throw std::runtime_error(
            "Cannot learn wrench bias: external joint torque exceeds limit");
      }
    }

    if (any_positive(state.joint_contact) ||
        any_positive(state.cartesian_contact) ||
        any_positive(state.joint_collision) ||
        any_positive(state.cartesian_collision)) {
      throw std::runtime_error(
          "Cannot learn wrench bias while robot reports contact/collision");
    }

    const double raw_force =
        norm3(state.O_F_ext_hat_K[0],
              state.O_F_ext_hat_K[1],
              state.O_F_ext_hat_K[2]);
    const double raw_torque =
        norm3(state.O_F_ext_hat_K[3],
              state.O_F_ext_hat_K[4],
              state.O_F_ext_hat_K[5]);

    if (raw_force > config.max_raw_external_force_n ||
        raw_torque > config.max_raw_external_torque_nm) {
      throw std::runtime_error(
          "Cannot learn wrench bias: raw Cartesian wrench exceeds emergency ceiling");
    }

    for (std::size_t i = 0; i < 6; ++i) {
      bias[i] += state.O_F_ext_hat_K[i];
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (double& value : bias) {
    value /= static_cast<double>(config.wrench_bias_samples);
  }
  return bias;
}

bool preflight_ok(
    const franka::RobotState& state,
    const PositionTrackingConfig& config,
    const Wrench6& wrench_bias,
    std::string& error) {
  for (std::size_t i = 0; i < 7; ++i) {
    if (std::abs(state.dq[i]) > config.max_start_joint_speed_radps) {
      error = "joint " + std::to_string(i + 1) +
              " is moving too fast before control";
      return false;
    }
    if (std::abs(state.tau_J[i]) > config.max_abs_joint_torque_nm[i]) {
      error = "joint " + std::to_string(i + 1) +
              " measured torque exceeds configured limit";
      return false;
    }
    if (std::abs(state.tau_ext_hat_filtered[i]) >
        config.max_abs_external_joint_torque_nm[i]) {
      error = "joint " + std::to_string(i + 1) +
              " external torque exceeds configured limit";
      return false;
    }
  }

  if (any_positive(state.joint_contact) ||
      any_positive(state.cartesian_contact) ||
      any_positive(state.joint_collision) ||
      any_positive(state.cartesian_collision)) {
    error = "robot reports contact/collision before control start";
    return false;
  }

  const double raw_force =
      norm3(state.O_F_ext_hat_K[0],
            state.O_F_ext_hat_K[1],
            state.O_F_ext_hat_K[2]);
  const double raw_torque =
      norm3(state.O_F_ext_hat_K[3],
            state.O_F_ext_hat_K[4],
            state.O_F_ext_hat_K[5]);

  if (raw_force > config.max_raw_external_force_n ||
      raw_torque > config.max_raw_external_torque_nm) {
    error = "raw external Cartesian wrench exceeds emergency ceiling";
    return false;
  }

  const Wrench6 delta = wrench_delta(state, wrench_bias);
  const double force = norm3(delta[0], delta[1], delta[2]);
  const double torque = norm3(delta[3], delta[4], delta[5]);

  if (force > config.max_external_force_n ||
      torque > config.max_external_torque_nm) {
    error = "baseline-subtracted external Cartesian wrench exceeds configured limit";
    return false;
  }

  error.clear();
  return true;
}

void print_preflight_measurements(
    const franka::RobotState& state,
    const Wrench6& wrench_bias) {
  const Wrench6 delta = wrench_delta(state, wrench_bias);

  const double raw_force =
      norm3(state.O_F_ext_hat_K[0],
            state.O_F_ext_hat_K[1],
            state.O_F_ext_hat_K[2]);
  const double raw_torque =
      norm3(state.O_F_ext_hat_K[3],
            state.O_F_ext_hat_K[4],
            state.O_F_ext_hat_K[5]);
  const double delta_force = norm3(delta[0], delta[1], delta[2]);
  const double delta_torque = norm3(delta[3], delta[4], delta[5]);

  std::cout << std::fixed << std::setprecision(4)
            << "Preflight configured masses kg: m_ee=" << state.m_ee
            << " m_load=" << state.m_load
            << " m_total=" << state.m_total << '\n';

  std::cout << "Preflight tau_ext Nm: [";
  for (std::size_t i = 0; i < 7; ++i) {
    if (i) std::cout << ", ";
    std::cout << state.tau_ext_hat_filtered[i];
  }

  std::cout << "]\nPreflight O_F_ext_hat_K: [";
  for (std::size_t i = 0; i < 6; ++i) {
    if (i) std::cout << ", ";
    std::cout << state.O_F_ext_hat_K[i];
  }

  std::cout << "]\nPreflight wrench bias: [";
  for (std::size_t i = 0; i < 6; ++i) {
    if (i) std::cout << ", ";
    std::cout << wrench_bias[i];
  }

  std::cout << "]\nPreflight wrench delta: [";
  for (std::size_t i = 0; i < 6; ++i) {
    if (i) std::cout << ", ";
    std::cout << delta[i];
  }

  std::cout << "]\nPreflight raw wrench norms: "
            << raw_force << " N, " << raw_torque << " Nm\n"
            << "Preflight baseline-subtracted wrench norms: "
            << delta_force << " N, " << delta_torque << " Nm\n";
}

RuntimeSafetyMonitor::RuntimeSafetyMonitor(
    PositionTrackingConfig config,
    Wrench6 wrench_bias)
    : config_(std::move(config)),
      wrench_bias_(wrench_bias) {}

RuntimeSafetyResult RuntimeSafetyMonitor::update(
    const franka::RobotState& state,
    const Transform& T_BF,
    double control_elapsed_s) {
  RuntimeSafetyResult result{};

  if (!startup_pose_set_) {
    startup_T_BF_ = T_BF;
    startup_pose_set_ = true;
  }

  const Vector3 p = transform_translation(T_BF);
  const Vector3 p0 = transform_translation(startup_T_BF_);
  for (std::size_t i = 0; i < 3; ++i) {
    result.diagnostics.travel_B_m[i] = p[i] - p0[i];
  }

  result.diagnostics.rotation_rad =
      angular_distance_rad(startup_T_BF_, T_BF);

  if (std::abs(result.diagnostics.travel_B_m[0]) > config_.max_travel_m[0] ||
      std::abs(result.diagnostics.travel_B_m[1]) > config_.max_travel_m[1] ||
      std::abs(result.diagnostics.travel_B_m[2]) > config_.max_travel_m[2] ||
      result.diagnostics.rotation_rad >
          deg_to_rad(config_.max_rotation_travel_deg)) {
    result.reason = StopReason::kTravelEnvelope;
    return result;
  }

  if (control_elapsed_s >= config_.communication_grace_s &&
      state.control_command_success_rate <
          config_.min_control_command_success_rate) {
    result.reason = StopReason::kCommunication;
    return result;
  }

  if (config_.stop_on_robot_contact &&
      (any_positive(state.joint_contact) ||
       any_positive(state.cartesian_contact) ||
       any_positive(state.joint_collision) ||
       any_positive(state.cartesian_collision))) {
    result.reason = StopReason::kRobotContact;
    return result;
  }

  for (std::size_t i = 0; i < 7; ++i) {
    if (std::abs(state.dq[i]) > config_.max_running_joint_speed_radps) {
      result.diagnostics.joint_index = i;
      result.diagnostics.joint_value = state.dq[i];
      result.reason = StopReason::kJointSpeed;
      return result;
    }
  }

  if (control_elapsed_s < config_.torque_monitor_grace_s) {
    return result;
  }

  bool torque_bad = false;
  StopReason torque_reason = StopReason::kNone;

  auto apply_derating = [&](
      double magnitude,
      double hard_limit,
      DerateSource source,
      std::size_t joint_index) {
    if (!(hard_limit > 0.0)) return;

    const double ratio = magnitude / hard_limit;
    if (ratio <= config_.torque_speed_derate_ratio) return;

    const double scale = std::clamp(
        (1.0 - ratio) /
            (1.0 - config_.torque_speed_derate_ratio),
        0.0,
        1.0);

    // Preserve the signal producing the strictest derate this cycle.
    if (scale < result.speed_scale) {
      result.speed_scale = scale;
      result.derate_source = source;
      result.derate_joint_index = joint_index;
      result.derate_value = magnitude;
      result.derate_limit = hard_limit;
      result.derate_ratio = ratio;
    }
  };

  for (std::size_t i = 0; i < 7; ++i) {
    apply_derating(
        std::abs(state.tau_J[i]),
        config_.max_abs_joint_torque_nm[i],
        DerateSource::kJointTorque,
        i);
    apply_derating(
        std::abs(state.tau_ext_hat_filtered[i]),
        config_.max_abs_external_joint_torque_nm[i],
        DerateSource::kExternalJointTorque,
        i);
    apply_derating(
        std::abs(state.dtau_J[i]),
        config_.max_abs_joint_torque_rate_nmps[i],
        DerateSource::kJointTorqueRate,
        i);

    if (std::abs(state.tau_J[i]) > config_.max_abs_joint_torque_nm[i]) {
      torque_bad = true;
      torque_reason = StopReason::kJointTorque;
      result.diagnostics.joint_index = i;
      result.diagnostics.joint_value = state.tau_J[i];
      break;
    }

    if (std::abs(state.tau_ext_hat_filtered[i]) >
        config_.max_abs_external_joint_torque_nm[i]) {
      torque_bad = true;
      torque_reason = StopReason::kExternalJointTorque;
      result.diagnostics.joint_index = i;
      result.diagnostics.joint_value = state.tau_ext_hat_filtered[i];
      break;
    }

    if (std::abs(state.dtau_J[i]) >
        config_.max_abs_joint_torque_rate_nmps[i]) {
      torque_bad = true;
      torque_reason = StopReason::kJointTorqueRate;
      result.diagnostics.joint_index = i;
      result.diagnostics.joint_value = state.dtau_J[i];
      break;
    }
  }

  const double raw_force =
      norm3(state.O_F_ext_hat_K[0],
            state.O_F_ext_hat_K[1],
            state.O_F_ext_hat_K[2]);
  const double raw_torque =
      norm3(state.O_F_ext_hat_K[3],
            state.O_F_ext_hat_K[4],
            state.O_F_ext_hat_K[5]);

  // Raw wrench ceilings are emergency ceilings and are never bias-subtracted.
  if (raw_force > config_.max_raw_external_force_n ||
      raw_torque > config_.max_raw_external_torque_nm) {
    result.reason = StopReason::kExternalWrench;
    return result;
  }

  const Wrench6 delta = wrench_delta(state, wrench_bias_);
  result.diagnostics.external_force_n =
      norm3(delta[0], delta[1], delta[2]);
  result.diagnostics.external_torque_nm =
      norm3(delta[3], delta[4], delta[5]);

  apply_derating(
      result.diagnostics.external_force_n,
      config_.max_external_force_n,
      DerateSource::kExternalForce,
      0);
  apply_derating(
      result.diagnostics.external_torque_nm,
      config_.max_external_torque_nm,
      DerateSource::kExternalTorque,
      0);

  if (result.diagnostics.external_force_n > config_.max_external_force_n ||
      result.diagnostics.external_torque_nm > config_.max_external_torque_nm) {
    torque_bad = true;
    torque_reason = StopReason::kExternalWrench;
  }

  torque_bad_cycles_ = torque_bad ? torque_bad_cycles_ + 1 : 0;
  if (torque_bad_cycles_ >= config_.torque_violation_cycles) {
    result.reason = torque_reason;
  }

  return result;
}

bool moving_toward_soft_travel_limit(
    const Vector3& velocity_B_mps,
    const Vector3& travel_B_m,
    const PositionTrackingConfig& config) {
  for (std::size_t i = 0; i < 3; ++i) {
    const double soft_limit =
        config.max_travel_m[i] - config.braking_margin_m;

    if ((velocity_B_mps[i] > 0.0 && travel_B_m[i] >= soft_limit) ||
        (velocity_B_mps[i] < 0.0 && travel_B_m[i] <= -soft_limit)) {
      return true;
    }
  }
  return false;
}

}  // namespace panda_tracker
