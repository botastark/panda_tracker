#include "panda_tracker/position_tracking_config.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace panda_tracker {
namespace {

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool parse_bool(const std::string& text, const std::string& key) {
  if (text == "true" || text == "True" || text == "TRUE") return true;
  if (text == "false" || text == "False" || text == "FALSE") return false;
  throw std::runtime_error("config key '" + key + "' must be true/false");
}

double parse_double(const std::string& text, const std::string& key) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("config key '" + key + "' must be finite");
  }
  return value;
}

std::size_t parse_size(const std::string& text, const std::string& key) {
  const double value = parse_double(text, key);
  if (value < 1.0 || std::floor(value) != value) {
    throw std::runtime_error("config key '" + key + "' must be a positive integer");
  }
  return static_cast<std::size_t>(value);
}

template <std::size_t N>
std::array<double, N> parse_array(std::string text, const std::string& key) {
  text = trim(text);
  if (text.size() < 2 || text.front() != '[' || text.back() != ']') {
    throw std::runtime_error("config key '" + key + "' must be an array");
  }

  text = text.substr(1, text.size() - 2);
  std::array<double, N> result{};
  std::stringstream stream(text);
  std::string item;
  std::size_t index = 0;

  while (std::getline(stream, item, ',')) {
    if (index >= N) {
      throw std::runtime_error("config key '" + key + "' has too many values");
    }
    result[index++] = parse_double(trim(item), key);
  }

  if (index != N) {
    throw std::runtime_error(
        "config key '" + key + "' must contain " + std::to_string(N) + " values");
  }
  return result;
}

std::unordered_map<std::string, std::string> read_flat_yaml(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Unable to open config: " + path);

  std::unordered_map<std::string, std::string> values;
  std::string line;
  std::size_t line_number = 0;

  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);

    line = trim(line);
    if (line.empty()) continue;

    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error(
          "Invalid config line " + std::to_string(line_number) +
          ": expected key: value");
    }

    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (key.empty() || value.empty()) {
      throw std::runtime_error("Invalid config line " + std::to_string(line_number));
    }
    if (!values.emplace(key, value).second) {
      throw std::runtime_error("Duplicate config key: " + key);
    }
  }
  return values;
}

bool positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

void validate_inertia(const std::array<double, 9>& I) {
  for (double value : I) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("load inertia must contain only finite values");
    }
  }

  // Franka uses column-major storage. For a symmetric 3x3 matrix this check is
  // independent of whether we view the flat array as row- or column-major.
  constexpr double kSymmetryTolerance = 1e-9;
  if (std::abs(I[1] - I[3]) > kSymmetryTolerance ||
      std::abs(I[2] - I[6]) > kSymmetryTolerance ||
      std::abs(I[5] - I[7]) > kSymmetryTolerance) {
    throw std::runtime_error("load inertia matrix must be symmetric");
  }

  const double det2 = I[0] * I[4] - I[3] * I[1];
  const double det3 =
      I[0] * (I[4] * I[8] - I[7] * I[5]) -
      I[3] * (I[1] * I[8] - I[7] * I[2]) +
      I[6] * (I[1] * I[5] - I[4] * I[2]);

  if (!positive(I[0]) || !positive(det2) || !positive(det3) ||
      I[0] > I[4] + I[8] ||
      I[4] > I[0] + I[8] ||
      I[8] > I[0] + I[4]) {
    throw std::runtime_error("load inertia matrix is not physically plausible");
  }
}

void validate(const PositionTrackingConfig& c) {
  if (!(c.control_axes.x || c.control_axes.y || c.control_axes.z)) {
    throw std::runtime_error("at least one of control_x/control_y/control_z must be true");
  }

  if (!positive(c.worker_rate_hz) || c.worker_rate_hz > 1000.0 ||
      !positive(c.kp_position) ||
      !positive(c.max_linear_speed_mps) ||
      !positive(c.max_linear_acceleration_mps2) ||
      !positive(c.max_linear_jerk_mps3) ||
      !positive(c.max_position_error_m) ||
      !positive(c.robot_state_timeout_s) ||
      !positive(c.tracker_timeout_s) ||
      c.tracking_loss_grace_s < 0.0 ||
      !std::isfinite(c.tracking_loss_grace_s) ||
      !positive(c.command_timeout_s) ||
      !positive(c.max_arm_wait_s) ||
      !positive(c.max_stop_time_s) ||
      !positive(c.max_rotation_travel_deg) ||
      !positive(c.max_start_joint_speed_radps) ||
      !positive(c.max_running_joint_speed_radps) ||
      !positive(c.max_external_force_n) ||
      !positive(c.max_external_torque_nm) ||
      !positive(c.max_raw_external_force_n) ||
      !positive(c.max_raw_external_torque_nm) ||
      !positive(c.stop_velocity_epsilon_mps) ||
      !positive(c.stop_acceleration_epsilon_mps2)) {
    throw std::runtime_error("config contains a non-positive required numeric value");
  }

  if (!std::isfinite(c.max_motion_runtime_s)) {
    throw std::runtime_error(
        "max_motion_runtime_s must be finite; use 0 for unlimited runtime");
  }

  for (double value : c.diagnostic_position_bias_B_m) {
    if (!std::isfinite(value)) {
      throw std::runtime_error(
          "diagnostic_position_bias_B_m must contain finite values");
    }
  }
  if (vector_norm(c.diagnostic_position_bias_B_m) > 0.050) {
    throw std::runtime_error(
        "diagnostic_position_bias_B_m is capped at 0.050 m for testing");
  }

  if (c.filter.median_window_samples < 3 ||
      c.filter.median_window_samples % 2 == 0) {
    throw std::runtime_error("filter_median_window_samples must be odd and >= 3");
  }
  if (!(c.filter.ema_alpha > 0.0 && c.filter.ema_alpha <= 1.0)) {
    throw std::runtime_error("filter_ema_alpha must be in (0, 1]");
  }
  if (!positive(c.filter.max_filtered_jump_m)) {
    throw std::runtime_error("filter_max_filtered_jump_m must be positive");
  }

  for (double travel : c.max_travel_m) {
    if (!positive(travel)) throw std::runtime_error("max_travel_m must be positive");
    if (!(c.braking_margin_m > 0.0 && c.braking_margin_m < travel)) {
      throw std::runtime_error(
          "braking_margin_m must be positive and smaller than every max_travel axis");
    }
  }

  if (c.wrench_bias_settle_s < 0.0 ||
      !(c.torque_speed_derate_ratio > 0.0 && c.torque_speed_derate_ratio < 1.0) ||
      c.torque_monitor_grace_s < 0.0 ||
      c.communication_grace_s < 0.0 ||
      c.min_control_command_success_rate < 0.0 ||
      c.min_control_command_success_rate > 1.0) {
    throw std::runtime_error("config contains an out-of-range safety value");
  }

  for (std::size_t i = 0; i < 7; ++i) {
    if (!positive(c.max_abs_joint_torque_nm[i]) ||
        !positive(c.max_abs_external_joint_torque_nm[i]) ||
        !positive(c.max_abs_joint_torque_rate_nmps[i])) {
      throw std::runtime_error("all joint torque/rate limits must be positive");
    }
  }

  if (!finite_rigid_transform(c.T_FS) ||
      !finite_rigid_transform(c.T_CS) ||
      !finite_rigid_transform(c.T_TS_des)) {
    throw std::runtime_error("T_FS, T_CS and T_TS_des must be finite rigid transforms");
  }

  if (c.load.enabled) {
    if (!positive(c.load.mass_kg) ||
        !positive(c.load.settle_s) ||
        c.load.max_preexisting_ee_mass_kg < 0.0) {
      throw std::runtime_error("enabled load model has invalid mass/timing values");
    }
    for (double value : c.load.com_F_m) {
      if (!std::isfinite(value)) throw std::runtime_error("load COM must be finite");
    }
    validate_inertia(c.load.inertia_at_com_F_kg_m2);
  }
}

}  // namespace

bool load_position_tracking_config(
    const std::string& path,
    PositionTrackingConfig& config,
    std::string& error) {
  try {
    const auto values = read_flat_yaml(path);

    auto require = [&](const char* key) -> const std::string& {
      const auto it = values.find(key);
      if (it == values.end()) {
        throw std::runtime_error(std::string("Missing config key: ") + key);
      }
      return it->second;
    };

    PositionTrackingConfig c{};

    c.motion_enabled = parse_bool(require("motion_enabled"), "motion_enabled");
    c.realtime_enforced = parse_bool(require("realtime_enforced"), "realtime_enforced");
    c.require_tracker_source_filter =
        parse_bool(require("require_tracker_source_filter"), "require_tracker_source_filter");
    c.stop_on_tracking_loss =
        parse_bool(require("stop_on_tracking_loss"), "stop_on_tracking_loss");
    c.stop_on_robot_contact =
        parse_bool(require("stop_on_robot_contact"), "stop_on_robot_contact");
    c.worker_rate_hz = parse_double(require("worker_rate_hz"), "worker_rate_hz");

    c.control_axes.x = parse_bool(require("control_x"), "control_x");
    c.control_axes.y = parse_bool(require("control_y"), "control_y");
    c.control_axes.z = parse_bool(require("control_z"), "control_z");

    c.kp_position = parse_double(require("kp_position"), "kp_position");
    c.max_linear_speed_mps =
        parse_double(require("max_linear_speed_mps"), "max_linear_speed_mps");
    c.max_linear_acceleration_mps2 =
        parse_double(require("max_linear_acceleration_mps2"), "max_linear_acceleration_mps2");
    c.max_linear_jerk_mps3 =
        parse_double(require("max_linear_jerk_mps3"), "max_linear_jerk_mps3");
    c.max_position_error_m =
        parse_double(require("max_position_error_m"), "max_position_error_m");

    // Optional diagnostic field so existing configs remain compatible.
    const auto diagnostic_bias_it =
        values.find("diagnostic_position_bias_B_m");
    if (diagnostic_bias_it != values.end()) {
      c.diagnostic_position_bias_B_m =
          parse_array<3>(
              diagnostic_bias_it->second,
              "diagnostic_position_bias_B_m");
    }

    c.robot_state_timeout_s =
        parse_double(require("robot_state_timeout_s"), "robot_state_timeout_s");
    c.tracker_timeout_s =
        parse_double(require("tracker_timeout_s"), "tracker_timeout_s");

    // Optional so older configs remain source-compatible. If omitted, a
    // 0.5-second recovery window is used.
    const auto tracking_loss_grace_it =
        values.find("tracking_loss_grace_s");
    if (tracking_loss_grace_it != values.end()) {
      c.tracking_loss_grace_s =
          parse_double(
              tracking_loss_grace_it->second,
              "tracking_loss_grace_s");
    }

    c.command_timeout_s =
        parse_double(require("command_timeout_s"), "command_timeout_s");
    c.arm_valid_packets = parse_size(require("arm_valid_packets"), "arm_valid_packets");
    c.max_arm_wait_s = parse_double(require("max_arm_wait_s"), "max_arm_wait_s");
    c.max_motion_runtime_s =
        parse_double(require("max_motion_runtime_s"), "max_motion_runtime_s");
    c.max_stop_time_s = parse_double(require("max_stop_time_s"), "max_stop_time_s");

    c.filter.median_window_samples =
        parse_size(require("filter_median_window_samples"), "filter_median_window_samples");
    c.filter.ema_alpha = parse_double(require("filter_ema_alpha"), "filter_ema_alpha");
    c.filter.max_filtered_jump_m =
        parse_double(require("filter_max_filtered_jump_m"), "filter_max_filtered_jump_m");

    c.max_travel_m = parse_array<3>(require("max_travel_m"), "max_travel_m");
    c.braking_margin_m = parse_double(require("braking_margin_m"), "braking_margin_m");
    c.max_rotation_travel_deg =
        parse_double(require("max_rotation_travel_deg"), "max_rotation_travel_deg");

    c.max_start_joint_speed_radps =
        parse_double(require("max_start_joint_speed_radps"), "max_start_joint_speed_radps");
    c.max_running_joint_speed_radps =
        parse_double(require("max_running_joint_speed_radps"), "max_running_joint_speed_radps");
    c.max_abs_joint_torque_nm =
        parse_array<7>(require("max_abs_joint_torque_nm"), "max_abs_joint_torque_nm");
    c.max_abs_external_joint_torque_nm =
        parse_array<7>(require("max_abs_external_joint_torque_nm"),
                       "max_abs_external_joint_torque_nm");
    c.max_abs_joint_torque_rate_nmps =
        parse_array<7>(require("max_abs_joint_torque_rate_nmps"),
                       "max_abs_joint_torque_rate_nmps");

    c.wrench_bias_enabled =
        parse_bool(require("wrench_bias_enabled"), "wrench_bias_enabled");
    c.wrench_bias_settle_s =
        parse_double(require("wrench_bias_settle_s"), "wrench_bias_settle_s");
    c.wrench_bias_samples =
        parse_size(require("wrench_bias_samples"), "wrench_bias_samples");
    c.max_external_force_n =
        parse_double(require("max_external_force_n"), "max_external_force_n");
    c.max_external_torque_nm =
        parse_double(require("max_external_torque_nm"), "max_external_torque_nm");
    c.max_raw_external_force_n =
        parse_double(require("max_raw_external_force_n"), "max_raw_external_force_n");
    c.max_raw_external_torque_nm =
        parse_double(require("max_raw_external_torque_nm"), "max_raw_external_torque_nm");
    c.torque_speed_derate_ratio =
        parse_double(require("torque_speed_derate_ratio"), "torque_speed_derate_ratio");
    c.torque_monitor_grace_s =
        parse_double(require("torque_monitor_grace_s"), "torque_monitor_grace_s");
    c.torque_violation_cycles =
        parse_size(require("torque_violation_cycles"), "torque_violation_cycles");

    c.min_control_command_success_rate =
        parse_double(require("min_control_command_success_rate"),
                     "min_control_command_success_rate");
    c.communication_grace_s =
        parse_double(require("communication_grace_s"), "communication_grace_s");

    c.stop_velocity_epsilon_mps =
        parse_double(require("stop_velocity_epsilon_mps"), "stop_velocity_epsilon_mps");
    c.stop_acceleration_epsilon_mps2 =
        parse_double(require("stop_acceleration_epsilon_mps2"),
                     "stop_acceleration_epsilon_mps2");

    c.load.enabled = parse_bool(require("load_model_enabled"), "load_model_enabled");
    c.load.includes_end_effector =
        parse_bool(require("load_model_includes_end_effector"),
                   "load_model_includes_end_effector");
    c.load.mass_kg = parse_double(require("load_mass_kg"), "load_mass_kg");
    c.load.com_F_m = parse_array<3>(require("load_com_F_m"), "load_com_F_m");
    c.load.inertia_at_com_F_kg_m2 =
        parse_array<9>(require("load_inertia_at_com_F_kg_m2"),
                       "load_inertia_at_com_F_kg_m2");
    c.load.settle_s = parse_double(require("load_settle_s"), "load_settle_s");
    c.load.max_preexisting_ee_mass_kg =
        parse_double(require("max_preexisting_ee_mass_kg"),
                     "max_preexisting_ee_mass_kg");

    c.T_FS = parse_array<16>(require("T_FS_row_major"), "T_FS_row_major");
    c.T_CS = parse_array<16>(require("T_CS_row_major"), "T_CS_row_major");
    c.T_TS_des = parse_array<16>(require("T_TS_des_row_major"), "T_TS_des_row_major");

    validate(c);
    config = c;
    error.clear();
    return true;
  } catch (const std::exception& e) {
    error = e.what();
    return false;
  }
}

}  // namespace panda_tracker
