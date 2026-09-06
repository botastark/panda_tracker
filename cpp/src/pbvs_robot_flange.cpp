#include "panda_tracker/pbvs.h"
#include "panda_tracker/tracker_receiver.h"

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <deque>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

namespace {

using Clock = std::chrono::steady_clock;
using panda_tracker::PbvsConfig;
using panda_tracker::PbvsController;
using panda_tracker::PbvsResult;
using panda_tracker::PbvsState;
using panda_tracker::TaskPoseMeasurement;
using panda_tracker::TrackerReceiver;
using panda_tracker::TrackerSnapshot;
using panda_tracker::Transform;

constexpr double kPi = 3.14159265358979323846;
std::atomic_bool g_stop_requested{false};

void request_stop(int) {
  g_stop_requested.store(true, std::memory_order_relaxed);
}

double deg_to_rad(double value) { return value * kPi / 180.0; }
double rad_to_deg(double value) { return value * 180.0 / kPi; }

double seconds_between(Clock::time_point later, Clock::time_point earlier) {
  return std::chrono::duration<double>(later - earlier).count();
}

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string tracker_bind_ip{"0.0.0.0"};
  std::string tracker_source_ip{};
  std::uint16_t tracker_port{5000};
  std::string pbvs_config_path{};
  std::string safe_config_path{};
  bool enable_motion{false};
  bool recover{false};
  bool preflight_only{false};
};

struct SafetyConfig {
  bool motion_enabled{false};
  bool require_tracker_source_filter{true};
  bool stop_on_tracking_loss{true};
  bool stop_on_robot_contact{true};

  double max_arm_wait_s{15.0};
  double max_motion_runtime_s{5.0};
  double max_stop_time_s{1.0};

  double max_linear_speed_mps{0.002};
  double max_linear_acceleration_mps2{0.010};
  double max_linear_jerk_mps3{0.100};
  double max_command_lead_m{0.001};
  double max_active_x_error_m{0.080};
  double envelope_braking_margin_m{0.001};
  double stop_velocity_epsilon_mps{0.00002};
  double stop_acceleration_epsilon_mps2{0.0005};

  double hard_max_tracker_age_s{0.150};
  double max_command_age_s{0.050};

  std::size_t quality_window_samples{12};
  std::size_t quality_min_samples{10};
  double tracker_ema_alpha_position{0.70};
  double tracker_max_position_jump_m{0.050};
  double max_task_std_x_m{0.005};
  double max_task_std_y_m{0.010};
  double max_task_std_z_m{0.020};
  double max_task_std_roll_deg{10.0};
  double max_task_std_pitch_deg{10.0};
  double max_task_std_yaw_deg{5.0};

  double max_travel_x_m{0.010};
  double max_travel_y_m{0.002};
  double max_travel_z_m{0.002};
  double max_rotation_travel_deg{2.0};
  double max_start_joint_speed_radps{0.050};
  double max_running_joint_speed_radps{0.500};

  std::array<double, 7> max_abs_joint_torque_nm{{80, 80, 80, 80, 11, 11, 11}};
  std::array<double, 7> max_abs_external_joint_torque_nm{{8, 8, 8, 8, 5, 5, 5}};
  std::array<double, 7> max_abs_joint_torque_rate_nmps{{800, 800, 800, 800, 300, 300, 300}};
  double max_external_force_n{15.0};
  double max_external_torque_nm{3.0};
  double torque_speed_derate_ratio{0.80};
  double torque_monitor_grace_s{0.50};
  std::size_t torque_violation_cycles{5};

  double min_control_command_success_rate{0.95};
  double communication_grace_s{0.50};
  std::size_t arm_tracking_cycles{10};
};

struct RobotSnapshot {
  std::array<double, 16> O_T_F{};
  Clock::time_point arrival{};
  bool available{false};
};

struct CommandSnapshot {
  double target_x_m{0.0};
  double requested_vx_mps{0.0};
  double active_x_error_m{0.0};
  Clock::time_point generated{};
  PbvsState state{PbvsState::kWaitForRobot};
  std::uint64_t tracker_sequence{0};
  bool target_available{false};
  bool armed{false};
};

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool parse_bool(const std::string& text, const std::string& key) {
  if (text == "true" || text == "True" || text == "TRUE") return true;
  if (text == "false" || text == "False" || text == "FALSE") return false;
  throw std::runtime_error("safe config key '" + key + "' must be true/false");
}

double parse_double(const std::string& text, const std::string& key) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("safe config key '" + key + "' must be finite");
  }
  return value;
}

std::size_t parse_size(const std::string& text, const std::string& key) {
  const double value = parse_double(text, key);
  if (value < 1.0 || std::floor(value) != value) {
    throw std::runtime_error("safe config key '" + key + "' must be a positive integer");
  }
  return static_cast<std::size_t>(value);
}

std::array<double, 7> parse_array7(std::string text, const std::string& key) {
  text = trim(text);
  if (text.size() < 2 || text.front() != '[' || text.back() != ']') {
    throw std::runtime_error("safe config key '" + key + "' must be a [7-value] array");
  }
  text = text.substr(1, text.size() - 2);
  std::array<double, 7> result{};
  std::stringstream stream(text);
  std::string item;
  std::size_t index = 0;
  while (std::getline(stream, item, ',')) {
    if (index >= result.size()) {
      throw std::runtime_error("safe config key '" + key + "' must contain 7 values");
    }
    result[index++] = parse_double(trim(item), key);
  }
  if (index != result.size()) {
    throw std::runtime_error("safe config key '" + key + "' must contain 7 values");
  }
  return result;
}

SafetyConfig load_safety_config(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Unable to open safety config: " + path);

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
      throw std::runtime_error("Invalid safety config line " + std::to_string(line_number));
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (key.empty() || value.empty()) {
      throw std::runtime_error("Invalid safety config line " + std::to_string(line_number));
    }
    if (!values.emplace(key, value).second) {
      throw std::runtime_error("Duplicate safety config key: " + key);
    }
  }

  auto require = [&](const char* key) -> const std::string& {
    const auto it = values.find(key);
    if (it == values.end()) throw std::runtime_error(std::string("Missing safety config key: ") + key);
    return it->second;
  };

  SafetyConfig s{};
  s.motion_enabled = parse_bool(require("motion_enabled"), "motion_enabled");
  s.require_tracker_source_filter = parse_bool(require("require_tracker_source_filter"), "require_tracker_source_filter");
  s.stop_on_tracking_loss = parse_bool(require("stop_on_tracking_loss"), "stop_on_tracking_loss");
  s.stop_on_robot_contact = parse_bool(require("stop_on_robot_contact"), "stop_on_robot_contact");
  s.max_arm_wait_s = parse_double(require("max_arm_wait_s"), "max_arm_wait_s");
  s.max_motion_runtime_s = parse_double(require("max_motion_runtime_s"), "max_motion_runtime_s");
  s.max_stop_time_s = parse_double(require("max_stop_time_s"), "max_stop_time_s");
  s.max_linear_speed_mps = parse_double(require("max_linear_speed_mps"), "max_linear_speed_mps");
  s.max_linear_acceleration_mps2 = parse_double(require("max_linear_acceleration_mps2"), "max_linear_acceleration_mps2");
  s.max_linear_jerk_mps3 = parse_double(require("max_linear_jerk_mps3"), "max_linear_jerk_mps3");
  s.max_command_lead_m = parse_double(require("max_command_lead_m"), "max_command_lead_m");
  s.max_active_x_error_m = parse_double(require("max_active_x_error_m"), "max_active_x_error_m");
  s.envelope_braking_margin_m = parse_double(require("envelope_braking_margin_m"), "envelope_braking_margin_m");
  s.stop_velocity_epsilon_mps = parse_double(require("stop_velocity_epsilon_mps"), "stop_velocity_epsilon_mps");
  s.stop_acceleration_epsilon_mps2 = parse_double(require("stop_acceleration_epsilon_mps2"), "stop_acceleration_epsilon_mps2");
  s.hard_max_tracker_age_s = parse_double(require("hard_max_tracker_age_s"), "hard_max_tracker_age_s");
  s.max_command_age_s = parse_double(require("max_command_age_s"), "max_command_age_s");
  s.quality_window_samples = parse_size(require("quality_window_samples"), "quality_window_samples");
  s.quality_min_samples = parse_size(require("quality_min_samples"), "quality_min_samples");
  s.tracker_ema_alpha_position = parse_double(require("tracker_ema_alpha_position"), "tracker_ema_alpha_position");
  s.tracker_max_position_jump_m = parse_double(require("tracker_max_position_jump_m"), "tracker_max_position_jump_m");
  s.max_task_std_x_m = parse_double(require("max_task_std_x_m"), "max_task_std_x_m");
  s.max_task_std_y_m = parse_double(require("max_task_std_y_m"), "max_task_std_y_m");
  s.max_task_std_z_m = parse_double(require("max_task_std_z_m"), "max_task_std_z_m");
  s.max_task_std_roll_deg = parse_double(require("max_task_std_roll_deg"), "max_task_std_roll_deg");
  s.max_task_std_pitch_deg = parse_double(require("max_task_std_pitch_deg"), "max_task_std_pitch_deg");
  s.max_task_std_yaw_deg = parse_double(require("max_task_std_yaw_deg"), "max_task_std_yaw_deg");
  s.max_travel_x_m = parse_double(require("max_travel_x_m"), "max_travel_x_m");
  s.max_travel_y_m = parse_double(require("max_travel_y_m"), "max_travel_y_m");
  s.max_travel_z_m = parse_double(require("max_travel_z_m"), "max_travel_z_m");
  s.max_rotation_travel_deg = parse_double(require("max_rotation_travel_deg"), "max_rotation_travel_deg");
  s.max_start_joint_speed_radps = parse_double(require("max_start_joint_speed_radps"), "max_start_joint_speed_radps");
  s.max_running_joint_speed_radps = parse_double(require("max_running_joint_speed_radps"), "max_running_joint_speed_radps");
  s.max_abs_joint_torque_nm = parse_array7(require("max_abs_joint_torque_nm"), "max_abs_joint_torque_nm");
  s.max_abs_external_joint_torque_nm = parse_array7(require("max_abs_external_joint_torque_nm"), "max_abs_external_joint_torque_nm");
  s.max_abs_joint_torque_rate_nmps = parse_array7(require("max_abs_joint_torque_rate_nmps"), "max_abs_joint_torque_rate_nmps");
  s.max_external_force_n = parse_double(require("max_external_force_n"), "max_external_force_n");
  s.max_external_torque_nm = parse_double(require("max_external_torque_nm"), "max_external_torque_nm");
  s.torque_speed_derate_ratio = parse_double(require("torque_speed_derate_ratio"), "torque_speed_derate_ratio");
  s.torque_monitor_grace_s = parse_double(require("torque_monitor_grace_s"), "torque_monitor_grace_s");
  s.torque_violation_cycles = parse_size(require("torque_violation_cycles"), "torque_violation_cycles");
  s.min_control_command_success_rate = parse_double(require("min_control_command_success_rate"), "min_control_command_success_rate");
  s.communication_grace_s = parse_double(require("communication_grace_s"), "communication_grace_s");
  s.arm_tracking_cycles = parse_size(require("arm_tracking_cycles"), "arm_tracking_cycles");

  auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
  if (!positive(s.max_arm_wait_s) || !positive(s.max_motion_runtime_s) ||
      !positive(s.max_stop_time_s) || !positive(s.max_linear_speed_mps) ||
      !positive(s.max_linear_acceleration_mps2) || !positive(s.max_linear_jerk_mps3) ||
      !positive(s.max_command_lead_m) || !positive(s.max_active_x_error_m) ||
      !positive(s.envelope_braking_margin_m) || !positive(s.stop_velocity_epsilon_mps) ||
      !positive(s.stop_acceleration_epsilon_mps2) || !positive(s.hard_max_tracker_age_s) ||
      !positive(s.max_command_age_s) || !positive(s.tracker_max_position_jump_m) ||
      !positive(s.max_travel_x_m) || !positive(s.max_travel_y_m) ||
      !positive(s.max_travel_z_m) || !positive(s.max_rotation_travel_deg) ||
      !positive(s.max_start_joint_speed_radps) || !positive(s.max_running_joint_speed_radps) ||
      !positive(s.max_external_force_n) || !positive(s.max_external_torque_nm) ||
      s.quality_min_samples > s.quality_window_samples ||
      !(s.tracker_ema_alpha_position > 0.0 && s.tracker_ema_alpha_position <= 1.0) ||
      s.min_control_command_success_rate < 0.0 || s.min_control_command_success_rate > 1.0 ||
      !(s.torque_speed_derate_ratio > 0.0 && s.torque_speed_derate_ratio < 1.0) ||
      s.communication_grace_s < 0.0 || s.torque_monitor_grace_s < 0.0) {
    throw std::runtime_error("Safety config contains an out-of-range value");
  }
  for (std::size_t i = 0; i < 7; ++i) {
    if (!positive(s.max_abs_joint_torque_nm[i]) ||
        !positive(s.max_abs_external_joint_torque_nm[i]) ||
        !positive(s.max_abs_joint_torque_rate_nmps[i])) {
      throw std::runtime_error("All torque limits must be positive");
    }
  }
  // Conservative allowance for braking from maximum speed while acceleration
  // is initially pointed in the direction of travel and must reverse under the
  // jerk limit. This intentionally overestimates the nominal stopping distance.
  const double minimum_braking_margin =
      s.max_linear_speed_mps * s.max_linear_speed_mps /
          s.max_linear_acceleration_mps2 +
      2.0 * s.max_linear_speed_mps * s.max_linear_acceleration_mps2 /
          s.max_linear_jerk_mps3;
  if (s.envelope_braking_margin_m < minimum_braking_margin ||
      s.envelope_braking_margin_m >= s.max_travel_x_m) {
    throw std::runtime_error("envelope_braking_margin_m is inconsistent with speed/acceleration limits");
  }
  return s;
}

std::uint16_t parse_port(const std::string& text) {
  const long value = std::stol(text);
  if (value < 1 || value > 65535) throw std::invalid_argument("--tracker-port must be in [1,65535]");
  return static_cast<std::uint16_t>(value);
}

void print_help(const char* argv0) {
  std::cout
      << "Conservative X-only PBVS controller using jerk-limited Cartesian velocity.\n\n"
      << "Usage: " << argv0 << " [options]\n"
      << "  --robot-ip IP\n"
      << "  --tracker-bind-ip IP\n"
      << "  --tracker-source-ip IP\n"
      << "  --tracker-port PORT\n"
      << "  --pbvs-config PATH\n"
      << "  --safe-config PATH\n"
      << "  --enable-motion       Required runtime motion enable\n"
      << "  --recover             Explicitly run automaticErrorRecovery first\n"
      << "  --preflight-only      Check state/torques and exit without control\n"
      << "  --help\n";
}

Options parse_options(int argc, char** argv) {
  Options o{};
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) {
      if (++i >= argc) throw std::invalid_argument(std::string("Missing value for ") + name);
      return std::string(argv[i]);
    };
    if (arg == "--robot-ip") o.robot_ip = next("--robot-ip");
    else if (arg == "--tracker-bind-ip") o.tracker_bind_ip = next("--tracker-bind-ip");
    else if (arg == "--tracker-source-ip") o.tracker_source_ip = next("--tracker-source-ip");
    else if (arg == "--tracker-port") o.tracker_port = parse_port(next("--tracker-port"));
    else if (arg == "--pbvs-config") o.pbvs_config_path = next("--pbvs-config");
    else if (arg == "--safe-config") o.safe_config_path = next("--safe-config");
    else if (arg == "--enable-motion") o.enable_motion = true;
    else if (arg == "--recover") o.recover = true;
    else if (arg == "--preflight-only") o.preflight_only = true;
    else if (arg == "--help" || arg == "-h") { print_help(argv[0]); std::exit(EXIT_SUCCESS); }
    else throw std::invalid_argument("Unknown option: " + arg);
  }
  if (o.pbvs_config_path.empty()) throw std::invalid_argument("--pbvs-config is required");
  if (o.safe_config_path.empty()) throw std::invalid_argument("--safe-config is required");
  return o;
}

std::array<double, 16> row_major_to_franka(const Transform& transform) {
  std::array<double, 16> result{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) result[col * 4 + row] = transform[row * 4 + col];
  }
  return result;
}

Transform measured_T_CS() {
  return {{
      1.0, 0.0, 0.0, -0.040,
      0.0, 1.0, 0.0,  0.000,
      0.0, 0.0, 1.0,  0.183,
      0.0, 0.0, 0.0,  1.000,
  }};
}

Transform make_transform(const panda_tracker::Matrix3& rotation,
                         const panda_tracker::Vector3& translation) {
  Transform result = panda_tracker::identity_transform();
  result[0] = rotation[0]; result[1] = rotation[1]; result[2] = rotation[2];
  result[4] = rotation[3]; result[5] = rotation[4]; result[6] = rotation[5];
  result[8] = rotation[6]; result[9] = rotation[7]; result[10] = rotation[8];
  result[3] = translation[0]; result[7] = translation[1]; result[11] = translation[2];
  return result;
}

std::array<double, 3> rpy_deg(const Transform& T) {
  return {{
      rad_to_deg(std::atan2(T[9], T[10])),
      rad_to_deg(std::atan2(-T[8], std::hypot(T[0], T[4]))),
      rad_to_deg(std::atan2(T[4], T[0])),
  }};
}

double norm3(double a, double b, double c) { return std::sqrt(a * a + b * b + c * c); }

struct QualitySample {
  std::array<double, 3> p{};
  std::array<double, 3> rpy{};
};

struct TrackerGate {
  std::deque<QualitySample> window;
  std::optional<Transform> last_accepted_T_CT;
  std::optional<Transform> filtered_T_TS;
  std::uint64_t sequence{0};
  std::uint64_t seen_sequence{0};
  Clock::time_point arrival{};
  bool have_seen{false};
  bool have_accepted{false};
  bool quality_ok{false};
  std::size_t rejected_jumps{0};

  static double stddev(const std::deque<QualitySample>& samples, bool angles, std::size_t axis) {
    if (samples.size() < 2) return std::numeric_limits<double>::infinity();
    double mean = 0.0;
    for (const auto& sample : samples) mean += angles ? sample.rpy[axis] : sample.p[axis];
    mean /= static_cast<double>(samples.size());
    double sum = 0.0;
    for (const auto& sample : samples) {
      const double value = angles ? sample.rpy[axis] : sample.p[axis];
      sum += (value - mean) * (value - mean);
    }
    return std::sqrt(sum / static_cast<double>(samples.size() - 1));
  }

  bool process_new(const TrackerSnapshot& tracker, const SafetyConfig& s) {
    const Transform T_CT = tracker.packet.T_TS;
    if (last_accepted_T_CT) {
      const Transform jump = panda_tracker::multiply_transform(
          panda_tracker::invert_transform(*last_accepted_T_CT), T_CT);
      const double distance = panda_tracker::vector_norm(panda_tracker::transform_translation(jump));
      if (distance > s.tracker_max_position_jump_m) {
        ++rejected_jumps;
        return false;
      }
    }
    last_accepted_T_CT = T_CT;
    const Transform raw_T_TS = panda_tracker::multiply_transform(
        panda_tracker::invert_transform(T_CT), measured_T_CS());
    if (!filtered_T_TS) {
      filtered_T_TS = raw_T_TS;
    } else {
      const auto old_p = panda_tracker::transform_translation(*filtered_T_TS);
      const auto new_p = panda_tracker::transform_translation(raw_T_TS);
      const double a = s.tracker_ema_alpha_position;
      const std::array<double, 3> p{{
          (1.0 - a) * old_p[0] + a * new_p[0],
          (1.0 - a) * old_p[1] + a * new_p[1],
          (1.0 - a) * old_p[2] + a * new_p[2],
      }};
      filtered_T_TS = make_transform(panda_tracker::transform_rotation(raw_T_TS), p);
    }
    QualitySample sample{};
    sample.p = panda_tracker::transform_translation(raw_T_TS);
    sample.rpy = rpy_deg(raw_T_TS);
    window.push_back(sample);
    while (window.size() > s.quality_window_samples) window.pop_front();
    quality_ok = window.size() >= s.quality_min_samples;
    if (quality_ok) {
      quality_ok =
          stddev(window, false, 0) <= s.max_task_std_x_m &&
          stddev(window, false, 1) <= s.max_task_std_y_m &&
          stddev(window, false, 2) <= s.max_task_std_z_m &&
          stddev(window, true, 0) <= s.max_task_std_roll_deg &&
          stddev(window, true, 1) <= s.max_task_std_pitch_deg &&
          stddev(window, true, 2) <= s.max_task_std_yaw_deg;
    }
    sequence = tracker.packet.sequence_id;
    arrival = tracker.arrival;
    have_accepted = true;
    return true;
  }
};

Transform physical_flange_pose(const franka::RobotState& state) {
  const Transform O_T_EE = panda_tracker::franka_column_major_transform(state.O_T_EE);
  const Transform F_T_EE = panda_tracker::franka_column_major_transform(state.F_T_EE);
  return panda_tracker::multiply_transform(O_T_EE, panda_tracker::invert_transform(F_T_EE));
}

double angular_distance_rad(const Transform& a, const Transform& b) {
  const Transform delta = panda_tracker::multiply_transform(panda_tracker::invert_transform(a), b);
  return panda_tracker::vector_norm(panda_tracker::so3_log(panda_tracker::transform_rotation(delta)));
}

bool any_positive(const std::array<double, 7>& values) {
  return std::any_of(values.begin(), values.end(), [](double value) { return value > 0.0; });
}

bool any_positive(const std::array<double, 6>& values) {
  return std::any_of(values.begin(), values.end(), [](double value) { return value > 0.0; });
}

enum class StopReason {
  kNone,
  kSignal,
  kArmTimeout,
  kRuntime,
  kTrackingLost,
  kCommandStale,
  kCommandLead,
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

const char* stop_reason_text(StopReason reason) {
  switch (reason) {
    case StopReason::kNone: return "none";
    case StopReason::kSignal: return "signal requested";
    case StopReason::kArmTimeout: return "arming timeout";
    case StopReason::kRuntime: return "motion runtime reached";
    case StopReason::kTrackingLost: return "tracking lost after arming";
    case StopReason::kCommandStale: return "PBVS command stale";
    case StopReason::kCommandLead: return "X command lead exceeded";
    case StopReason::kTravelEnvelope: return "measured flange travel envelope exceeded";
    case StopReason::kBrakingBoundary: return "X braking boundary reached";
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

struct StopDiagnostics {
  double dx_m{0.0};
  double dy_m{0.0};
  double dz_m{0.0};
  double rotation_rad{0.0};
  double force_n{0.0};
  double torque_nm{0.0};
  std::size_t joint_index{0};
  double joint_value{0.0};
  bool forced_finish_after_stop_timeout{false};
};

bool preflight_ok(const franka::RobotState& state, const SafetyConfig& s, std::string& error) {
  for (std::size_t i = 0; i < 7; ++i) {
    if (std::abs(state.dq[i]) > s.max_start_joint_speed_radps) {
      error = "joint " + std::to_string(i + 1) + " is moving at " + std::to_string(state.dq[i]) + " rad/s";
      return false;
    }
    if (std::abs(state.tau_J[i]) > s.max_abs_joint_torque_nm[i]) {
      error = "joint " + std::to_string(i + 1) + " measured torque " +
              std::to_string(state.tau_J[i]) + " Nm exceeds " +
              std::to_string(s.max_abs_joint_torque_nm[i]) + " Nm";
      return false;
    }
    if (std::abs(state.tau_ext_hat_filtered[i]) > s.max_abs_external_joint_torque_nm[i]) {
      error = "joint " + std::to_string(i + 1) + " external torque " +
              std::to_string(state.tau_ext_hat_filtered[i]) + " Nm exceeds " +
              std::to_string(s.max_abs_external_joint_torque_nm[i]) + " Nm";
      return false;
    }
  }
  if (any_positive(state.joint_contact) || any_positive(state.cartesian_contact) ||
      any_positive(state.joint_collision) || any_positive(state.cartesian_collision)) {
    error = "robot reports contact or collision before control start";
    return false;
  }
  const double external_force =
      norm3(state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1], state.O_F_ext_hat_K[2]);
  const double external_torque =
      norm3(state.O_F_ext_hat_K[3], state.O_F_ext_hat_K[4], state.O_F_ext_hat_K[5]);
  if (external_force > s.max_external_force_n || external_torque > s.max_external_torque_nm) {
    error = "external Cartesian wrench is above configured limit: force=" +
            std::to_string(external_force) + " N (limit " +
            std::to_string(s.max_external_force_n) + " N), torque=" +
            std::to_string(external_torque) + " Nm (limit " +
            std::to_string(s.max_external_torque_nm) + " Nm)";
    return false;
  }
  return true;
}

void print_preflight_measurements(const franka::RobotState& state) {
  std::cout << "Preflight configured masses kg: m_ee=" << state.m_ee
            << " m_load=" << state.m_load
            << " m_total=" << state.m_total << '\n';
  std::cout << "Preflight F_x_Cee m: [" << state.F_x_Cee[0] << ", "
            << state.F_x_Cee[1] << ", " << state.F_x_Cee[2] << "]\n";
  std::cout << "Preflight F_x_Cload m: [" << state.F_x_Cload[0] << ", "
            << state.F_x_Cload[1] << ", " << state.F_x_Cload[2] << "]\n";
  std::cout << "Preflight F_x_Ctotal m: [" << state.F_x_Ctotal[0] << ", "
            << state.F_x_Ctotal[1] << ", " << state.F_x_Ctotal[2] << "]\n";
  std::cout << "Preflight tau_J Nm: [";
  for (std::size_t i = 0; i < 7; ++i) {
    if (i) std::cout << ", ";
    std::cout << state.tau_J[i];
  }
  std::cout << "]\nPreflight tau_ext Nm: [";
  for (std::size_t i = 0; i < 7; ++i) {
    if (i) std::cout << ", ";
    std::cout << state.tau_ext_hat_filtered[i];
  }
  const double force = norm3(state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1],
                             state.O_F_ext_hat_K[2]);
  const double torque = norm3(state.O_F_ext_hat_K[3], state.O_F_ext_hat_K[4],
                              state.O_F_ext_hat_K[5]);
  std::cout << "]\nPreflight O_F_ext_hat_K: [";
  for (std::size_t i = 0; i < 6; ++i) {
    if (i) std::cout << ", ";
    std::cout << state.O_F_ext_hat_K[i];
  }
  std::cout << "]\nPreflight external wrench norms: " << force << " N, "
            << torque << " Nm\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const SafetyConfig safety = load_safety_config(options.safe_config_path);

    PbvsConfig pbvs_config{};
    std::string config_error;
    if (!panda_tracker::load_pbvs_config(options.pbvs_config_path, pbvs_config, config_error)) {
      throw std::runtime_error("Unable to load PBVS config: " + config_error);
    }
    if (!options.preflight_only && (!safety.motion_enabled || !options.enable_motion)) {
      throw std::runtime_error("Motion requires motion_enabled:true and --enable-motion");
    }
    if (!options.preflight_only && safety.require_tracker_source_filter &&
        options.tracker_source_ip.empty()) {
      throw std::runtime_error("Active motion requires --tracker-source-ip");
    }
    if (pbvs_config.control_orientation) {
      throw std::runtime_error("This first-test controller is deliberately X-translation-only; set control_orientation:false");
    }
    if (pbvs_config.max_linear_speed > safety.max_linear_speed_mps + 1e-12 ||
        pbvs_config.max_command_lead > safety.max_command_lead_m + 1e-12 ||
        pbvs_config.tracker_timeout > safety.hard_max_tracker_age_s + 1e-12) {
      throw std::runtime_error("PBVS config exceeds the independent safety config");
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    if (options.preflight_only) {
      franka::Robot robot(options.robot_ip, franka::RealtimeConfig::kEnforce);
      if (options.recover) {
        robot.automaticErrorRecovery();
        std::cout << "Explicit robot error recovery complete.\n";
      }
      const franka::RobotState state = robot.readOnce();
      print_preflight_measurements(state);
      std::string preflight_error;
      if (!preflight_ok(state, safety, preflight_error)) {
        throw std::runtime_error("Preflight rejected: " + preflight_error);
      }
      std::cout << "PREFLIGHT PASSED: no control loop or motion command was started.\n";
      return EXIT_SUCCESS;
    }

    TrackerReceiver tracker_receiver(options.tracker_bind_ip,
                                     options.tracker_source_ip,
                                     options.tracker_port);
    PbvsController pbvs(pbvs_config);

    std::mutex tracker_mutex;
    TrackerSnapshot shared_tracker{};
    std::mutex robot_mutex;
    RobotSnapshot shared_robot{};
    std::mutex command_mutex;
    CommandSnapshot shared_command{};
    std::mutex log_mutex;
    std::mutex failure_mutex;
    std::exception_ptr worker_failure;
    std::atomic_bool worker_failed{false};
    std::atomic_bool workers_running{true};

    std::thread tracker_thread([&]() {
      try {
        while (workers_running.load(std::memory_order_relaxed) &&
               !g_stop_requested.load(std::memory_order_relaxed)) {
          tracker_receiver.poll();
          const TrackerSnapshot latest = tracker_receiver.latest();
          if (latest.available && tracker_mutex.try_lock()) {
            shared_tracker = latest;
            tracker_mutex.unlock();
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(failure_mutex);
        worker_failure = std::current_exception();
        worker_failed.store(true, std::memory_order_relaxed);
        g_stop_requested.store(true, std::memory_order_relaxed);
      }
    });

    TrackerGate tracker_gate{};
    std::thread pbvs_thread([&]() {
      try {
        const auto nominal_period = std::chrono::duration<double>(1.0 / pbvs_config.control_rate_hz);
        auto next = Clock::now();
        auto last = next - std::chrono::duration_cast<Clock::duration>(nominal_period);
        auto next_log = next;
        std::size_t safe_cycles = 0;
        while (workers_running.load(std::memory_order_relaxed) &&
               !g_stop_requested.load(std::memory_order_relaxed)) {
          const auto now = Clock::now();
          if (now < next) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
          }
          RobotSnapshot robot{};
          TrackerSnapshot tracker{};
          {
            std::lock_guard<std::mutex> lock(robot_mutex);
            robot = shared_robot;
          }
          {
            std::lock_guard<std::mutex> lock(tracker_mutex);
            tracker = shared_tracker;
          }

          std::optional<Transform> T_BF;
          double robot_age_s = std::numeric_limits<double>::infinity();
          if (robot.available) {
            T_BF = panda_tracker::franka_column_major_transform(robot.O_T_F);
            robot_age_s = seconds_between(now, robot.arrival);
          }
          if (tracker.available &&
              (!tracker_gate.have_seen || tracker.packet.sequence_id != tracker_gate.seen_sequence)) {
            tracker_gate.have_seen = true;
            tracker_gate.seen_sequence = tracker.packet.sequence_id;
            if (tracker.packet.valid) tracker_gate.process_new(tracker, safety);
          }

          double tracker_age_s = std::numeric_limits<double>::infinity();
          std::optional<TaskPoseMeasurement> measurement;
          if (tracker_gate.have_accepted && tracker_gate.filtered_T_TS) {
            tracker_age_s = seconds_between(now, tracker_gate.arrival);
            TaskPoseMeasurement m{};
            m.T_TS = *tracker_gate.filtered_T_TS;
            m.timestamp_s = std::chrono::duration<double>(tracker_gate.arrival.time_since_epoch()).count();
            m.valid = tracker_gate.quality_ok && tracker_age_s >= 0.0 &&
                      tracker_age_s <= safety.hard_max_tracker_age_s;
            m.sequence_id = tracker_gate.sequence;
            measurement = m;
          }

          double dt_s = seconds_between(now, last);
          if (!(dt_s > 0.0) || !std::isfinite(dt_s)) dt_s = 1.0 / pbvs_config.control_rate_hz;
          last = now;
          const double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
          const PbvsResult result = pbvs.step(T_BF, robot_age_s, measurement, now_s, dt_s);

          const bool tracker_fresh = tracker_gate.have_accepted && tracker_gate.quality_ok &&
                                     tracker_age_s >= 0.0 && tracker_age_s <= safety.hard_max_tracker_age_s;
          const double active_x_error = std::abs(result.position_error_base_m[0]);
          const double requested_vx = result.proposed_linear_velocity_base_mps[0];
          const bool safe_tracking =
              result.state == PbvsState::kTracking && result.has_proposed_pose && tracker_fresh && T_BF &&
              active_x_error <= safety.max_active_x_error_m &&
              std::abs(requested_vx) <= safety.max_linear_speed_mps + 1e-9;
          safe_cycles = safe_tracking ? safe_cycles + 1 : 0;

          CommandSnapshot command{};
          command.state = result.state;
          command.active_x_error_m = active_x_error;
          command.requested_vx_mps = requested_vx;
          command.tracker_sequence = tracker_gate.have_seen ? tracker_gate.seen_sequence : 0;
          command.generated = now;
          command.armed = safe_tracking && safe_cycles >= safety.arm_tracking_cycles;
          if (safe_tracking) {
            command.target_x_m = result.proposed_T_BE[3];
            command.target_available = true;
          }
          {
            std::lock_guard<std::mutex> lock(command_mutex);
            shared_command = command;
          }

          if (now >= next_log) {
            std::lock_guard<std::mutex> log_lock(log_mutex);
            std::cout << std::fixed << std::setprecision(4)
                      << "pbvs=" << panda_tracker::pbvs_state_text(result.state)
                      << " tracker_seq=" << command.tracker_sequence
                      << " tracker_age_ms=" << tracker_age_s * 1000.0
                      << " quality=" << (tracker_gate.quality_ok ? "PASS" : "hold")
                      << " rejected_jumps=" << tracker_gate.rejected_jumps
                      << " x_error_mm=" << active_x_error * 1000.0
                      << " requested_vx_mmps=" << requested_vx * 1000.0
                      << " arm_cycles=" << safe_cycles
                      << " armed=" << (command.armed ? "YES" : "no");
            if (!result.reason.empty()) std::cout << " reason=" << result.reason;
            std::cout << '\n';
            next_log = now + std::chrono::seconds(1);
          }
          next = now + std::chrono::duration_cast<Clock::duration>(nominal_period);
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(failure_mutex);
        worker_failure = std::current_exception();
        worker_failed.store(true, std::memory_order_relaxed);
        g_stop_requested.store(true, std::memory_order_relaxed);
      }
    });

    {
      std::lock_guard<std::mutex> log_lock(log_mutex);
      std::cout
          << "SAFE X-ONLY PBVS ROBOT CONTROLLER\n"
          << "Robot: " << options.robot_ip << '\n'
          << "Tracker: " << options.tracker_bind_ip << ':' << options.tracker_port
          << " source=" << options.tracker_source_ip << '\n'
          << "Command interface: Cartesian velocity, base-frame X only\n"
          << "Max speed/acceleration/jerk: "
          << safety.max_linear_speed_mps * 1000.0 << " mm/s, "
          << safety.max_linear_acceleration_mps2 * 1000.0 << " mm/s^2, "
          << safety.max_linear_jerk_mps3 * 1000.0 << " mm/s^3\n"
          << "Automatic recovery: " << (options.recover ? "explicitly requested" : "disabled") << '\n';
    }

    std::exception_ptr robot_failure;
    StopReason stop_reason = StopReason::kNone;
    StopDiagnostics stop_diagnostics{};

    try {
      // A real-time scheduling failure is a hard preflight failure for motion.
      franka::Robot robot(options.robot_ip, franka::RealtimeConfig::kEnforce);
      if (options.recover) {
        robot.automaticErrorRecovery();
        std::lock_guard<std::mutex> log_lock(log_mutex);
        std::cout << "Explicit robot error recovery complete.\n";
      }
      const franka::RobotState initial_state = robot.readOnce();
      {
        std::lock_guard<std::mutex> log_lock(log_mutex);
        print_preflight_measurements(initial_state);
      }
      std::string preflight_error;
      if (!preflight_ok(initial_state, safety, preflight_error)) {
        throw std::runtime_error("Preflight rejected: " + preflight_error);
      }
      {
        std::lock_guard<std::mutex> log_lock(log_mutex);
        std::cout << "Preflight passed; waiting for safe PBVS arming.\n";
      }

      bool ever_armed = false;
      bool stopping = false;
      std::size_t torque_bad_cycles = 0;
      double control_elapsed_s = 0.0;
      double motion_elapsed_s = 0.0;
      double stopping_elapsed_s = 0.0;
      Transform startup_O_T_F = panda_tracker::identity_transform();
      bool startup_pose_set = false;

      auto begin_stop = [&](StopReason reason) {
        if (!stopping) {
          stopping = true;
          stop_reason = reason;
          stopping_elapsed_s = 0.0;
        }
      };

      robot.control(
          [&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianVelocities {
            const double dt = std::max(0.0, period.toSec());
            control_elapsed_s += dt;
            if (ever_armed && !stopping) motion_elapsed_s += dt;
            if (stopping) stopping_elapsed_s += dt;

            const auto now = Clock::now();
            const Transform O_T_F = physical_flange_pose(state);
            const auto flange_p = panda_tracker::transform_translation(O_T_F);
            if (!startup_pose_set) {
              startup_O_T_F = O_T_F;
              startup_pose_set = true;
            }
            const auto startup_p = panda_tracker::transform_translation(startup_O_T_F);
            stop_diagnostics.dx_m = flange_p[0] - startup_p[0];
            stop_diagnostics.dy_m = flange_p[1] - startup_p[1];
            stop_diagnostics.dz_m = flange_p[2] - startup_p[2];
            stop_diagnostics.rotation_rad = angular_distance_rad(startup_O_T_F, O_T_F);

            if (robot_mutex.try_lock()) {
              shared_robot.O_T_F = row_major_to_franka(O_T_F);
              shared_robot.arrival = now;
              shared_robot.available = true;
              robot_mutex.unlock();
            }

            if (g_stop_requested.load(std::memory_order_relaxed)) {
              begin_stop(worker_failed.load(std::memory_order_relaxed)
                             ? StopReason::kWorkerFailure
                             : StopReason::kSignal);
            }
            if (!ever_armed && control_elapsed_s >= safety.max_arm_wait_s) begin_stop(StopReason::kArmTimeout);
            if (ever_armed && motion_elapsed_s >= safety.max_motion_runtime_s) begin_stop(StopReason::kRuntime);
            if (control_elapsed_s >= safety.communication_grace_s &&
                state.control_command_success_rate < safety.min_control_command_success_rate) {
              begin_stop(StopReason::kCommunication);
            }
            if (std::abs(stop_diagnostics.dx_m) > safety.max_travel_x_m ||
                std::abs(stop_diagnostics.dy_m) > safety.max_travel_y_m ||
                std::abs(stop_diagnostics.dz_m) > safety.max_travel_z_m ||
                stop_diagnostics.rotation_rad > deg_to_rad(safety.max_rotation_travel_deg)) {
              begin_stop(StopReason::kTravelEnvelope);
            }

            for (std::size_t i = 0; i < 7; ++i) {
              if (std::abs(state.dq[i]) > safety.max_running_joint_speed_radps) {
                stop_diagnostics.joint_index = i;
                stop_diagnostics.joint_value = state.dq[i];
                begin_stop(StopReason::kJointSpeed);
              }
            }

            bool torque_bad = false;
            StopReason torque_reason = StopReason::kNone;
            double torque_speed_scale = 1.0;
            auto apply_torque_derating = [&](double magnitude, double hard_limit) {
              const double ratio = magnitude / hard_limit;
              if (ratio > safety.torque_speed_derate_ratio) {
                const double scale = std::clamp(
                    (1.0 - ratio) / (1.0 - safety.torque_speed_derate_ratio),
                    0.0,
                    1.0);
                torque_speed_scale = std::min(torque_speed_scale, scale);
              }
            };
            if (control_elapsed_s >= safety.torque_monitor_grace_s) {
              for (std::size_t i = 0; i < 7; ++i) {
                apply_torque_derating(std::abs(state.tau_J[i]), safety.max_abs_joint_torque_nm[i]);
                apply_torque_derating(std::abs(state.tau_ext_hat_filtered[i]),
                                      safety.max_abs_external_joint_torque_nm[i]);
                apply_torque_derating(std::abs(state.dtau_J[i]),
                                      safety.max_abs_joint_torque_rate_nmps[i]);
                if (std::abs(state.tau_J[i]) > safety.max_abs_joint_torque_nm[i]) {
                  torque_bad = true; torque_reason = StopReason::kJointTorque;
                  stop_diagnostics.joint_index = i; stop_diagnostics.joint_value = state.tau_J[i]; break;
                }
                if (std::abs(state.tau_ext_hat_filtered[i]) > safety.max_abs_external_joint_torque_nm[i]) {
                  torque_bad = true; torque_reason = StopReason::kExternalJointTorque;
                  stop_diagnostics.joint_index = i; stop_diagnostics.joint_value = state.tau_ext_hat_filtered[i]; break;
                }
                if (std::abs(state.dtau_J[i]) > safety.max_abs_joint_torque_rate_nmps[i]) {
                  torque_bad = true; torque_reason = StopReason::kJointTorqueRate;
                  stop_diagnostics.joint_index = i; stop_diagnostics.joint_value = state.dtau_J[i]; break;
                }
              }
              stop_diagnostics.force_n = norm3(state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1], state.O_F_ext_hat_K[2]);
              stop_diagnostics.torque_nm = norm3(state.O_F_ext_hat_K[3], state.O_F_ext_hat_K[4], state.O_F_ext_hat_K[5]);
              apply_torque_derating(stop_diagnostics.force_n, safety.max_external_force_n);
              apply_torque_derating(stop_diagnostics.torque_nm, safety.max_external_torque_nm);
              if (stop_diagnostics.force_n > safety.max_external_force_n ||
                  stop_diagnostics.torque_nm > safety.max_external_torque_nm) {
                torque_bad = true;
                torque_reason = StopReason::kExternalWrench;
              }
            }
            torque_bad_cycles = torque_bad ? torque_bad_cycles + 1 : 0;
            if (torque_bad_cycles >= safety.torque_violation_cycles) begin_stop(torque_reason);

            if (safety.stop_on_robot_contact &&
                (any_positive(state.joint_contact) || any_positive(state.cartesian_contact) ||
                 any_positive(state.joint_collision) || any_positive(state.cartesian_collision))) {
              begin_stop(StopReason::kRobotContact);
            }

            CommandSnapshot command{};
            bool got_command = false;
            if (command_mutex.try_lock()) {
              command = shared_command;
              command_mutex.unlock();
              got_command = true;
            }

            double desired_vx = 0.0;
            if (!stopping && got_command && command.armed && command.target_available) {
              if (seconds_between(now, command.generated) > safety.max_command_age_s) {
                begin_stop(StopReason::kCommandStale);
              } else if (std::abs(command.target_x_m - flange_p[0]) > safety.max_command_lead_m + 1e-6) {
                begin_stop(StopReason::kCommandLead);
              } else {
                ever_armed = true;
                desired_vx = std::clamp(command.requested_vx_mps,
                                        -safety.max_linear_speed_mps,
                                        safety.max_linear_speed_mps) *
                             torque_speed_scale;
                const double soft_limit = safety.max_travel_x_m - safety.envelope_braking_margin_m;
                if ((desired_vx > 0.0 && stop_diagnostics.dx_m >= soft_limit) ||
                    (desired_vx < 0.0 && stop_diagnostics.dx_m <= -soft_limit)) {
                  desired_vx = 0.0;
                  begin_stop(StopReason::kBrakingBoundary);
                }
              }
            } else if (!stopping && ever_armed && safety.stop_on_tracking_loss && got_command) {
              begin_stop(StopReason::kTrackingLost);
            }

            std::array<double, 6> desired_twist{{desired_vx, 0.0, 0.0, 0.0, 0.0, 0.0}};
            if (stopping) desired_twist.fill(0.0);

            const std::array<double, 6> limited_twist = franka::limitRate(
                safety.max_linear_speed_mps,
                safety.max_linear_acceleration_mps2,
                safety.max_linear_jerk_mps3,
                deg_to_rad(1.0),
                deg_to_rad(5.0),
                deg_to_rad(50.0),
                desired_twist,
                state.O_dP_EE_c,
                state.O_ddP_EE_c);

            if (stopping) {
              bool velocity_zero = true;
              bool acceleration_zero = true;
              for (std::size_t i = 0; i < 6; ++i) {
                velocity_zero = velocity_zero && std::abs(limited_twist[i]) <= safety.stop_velocity_epsilon_mps;
                acceleration_zero = acceleration_zero &&
                    std::abs(state.O_ddP_EE_c[i]) <= safety.stop_acceleration_epsilon_mps2;
              }
              if (velocity_zero && acceleration_zero) {
                return franka::MotionFinished(franka::CartesianVelocities(limited_twist));
              }
              if (stopping_elapsed_s >= safety.max_stop_time_s) {
                stop_diagnostics.forced_finish_after_stop_timeout = true;
                std::array<double, 6> zero{};
                return franka::MotionFinished(franka::CartesianVelocities(zero));
              }
            }
            return franka::CartesianVelocities(limited_twist);
          },
          franka::ControllerMode::kCartesianImpedance,
          true);
    } catch (...) {
      robot_failure = std::current_exception();
    }

    workers_running.store(false, std::memory_order_relaxed);
    g_stop_requested.store(true, std::memory_order_relaxed);
    if (tracker_thread.joinable()) tracker_thread.join();
    if (pbvs_thread.joinable()) pbvs_thread.join();

    if (stop_reason != StopReason::kNone) {
      std::cerr << std::fixed << std::setprecision(4)
                << "STOP: " << stop_reason_text(stop_reason)
                << " travel_mm=[" << stop_diagnostics.dx_m * 1000.0 << ','
                << stop_diagnostics.dy_m * 1000.0 << ','
                << stop_diagnostics.dz_m * 1000.0 << ']'
                << " rotation_deg=" << rad_to_deg(stop_diagnostics.rotation_rad)
                << " external_force_N=" << stop_diagnostics.force_n
                << " external_torque_Nm=" << stop_diagnostics.torque_nm
                << " forced_finish="
                << (stop_diagnostics.forced_finish_after_stop_timeout ? "YES" : "no");
      if (stop_reason == StopReason::kJointSpeed || stop_reason == StopReason::kJointTorque ||
          stop_reason == StopReason::kExternalJointTorque || stop_reason == StopReason::kJointTorqueRate) {
        std::cerr << " joint=" << stop_diagnostics.joint_index + 1
                  << " value=" << stop_diagnostics.joint_value;
      }
      std::cerr << '\n';
    }
    if (robot_failure) std::rethrow_exception(robot_failure);
    {
      std::lock_guard<std::mutex> lock(failure_mutex);
      if (worker_failure) std::rethrow_exception(worker_failure);
    }
    std::cout << "Robot control loop ended after a rate-limited stop.\n";
    return EXIT_SUCCESS;
  } catch (const franka::Exception& error) {
    std::cerr << "Franka error: " << error.what() << '\n';
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
  }
  return EXIT_FAILURE;
}
