#include "panda_tracker/pbvs.h"
#include "panda_tracker/tracker_receiver.h"

#include <franka/control_types.h>
#include <franka/exception.h>
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

std::atomic_bool g_stop_requested{false};

void request_stop(int) {
  g_stop_requested.store(true, std::memory_order_relaxed);
}

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string tracker_bind_ip{"0.0.0.0"};
  std::string tracker_source_ip{};
  std::uint16_t tracker_port{5000};
  std::string pbvs_config_path{};
  std::string safe_config_path{};
  bool enable_motion{false};
};

struct SafetyConfig {
  bool motion_enabled{false};
  bool require_tracker_source_filter{true};
  bool stop_on_tracking_loss{true};

  // Command-axis mask. Translation axes are Panda base O/B axes.
  // Rotation axes are flange/body rotation-vector axes.
  bool control_x{true};
  bool control_y{false};
  bool control_z{false};
  bool control_rx{false};
  bool control_ry{false};
  bool control_rz{false};

  double max_arm_wait_s{15.0};
  double max_runtime_s{5.0};

  // Independent hard command caps.
  double hard_max_linear_speed_mps{0.002};
  double hard_max_angular_speed_degps{1.0};
  double hard_max_command_lead_m{0.001};
  double hard_max_angular_command_lead_deg{1.0};
  double hard_max_active_position_error_m{0.080};
  double hard_max_active_orientation_error_deg{10.0};
  double hard_max_tracker_age_s{0.100};
  double max_command_age_s{0.050};

  // Low-latency tracker gate/filter. Updated only on NEW tracker packets.
  std::size_t quality_window_samples{12};
  std::size_t quality_min_samples{10};
  double tracker_ema_alpha_position{0.70};
  double tracker_ema_alpha_orientation{0.70};
  double tracker_max_position_jump_m{0.050};
  double tracker_max_angle_jump_deg{45.0};

  // Raw T_TS stationary-spread gates. These are NOT averaging delays:
  // the current accepted packet is used immediately.
  double max_task_std_x_m{0.015};
  double max_task_std_y_m{0.015};
  double max_task_std_z_m{0.050};
  double max_task_std_roll_deg{35.0};
  double max_task_std_pitch_deg{35.0};
  double max_task_std_yaw_deg{8.0};

  // Independent flange travel envelope measured from control-loop startup.
  double max_travel_x_m{0.010};
  double max_travel_y_m{0.002};
  double max_travel_z_m{0.002};
  double max_rotation_travel_deg{2.0};

  double min_control_command_success_rate{0.95};
  double communication_grace_s{0.50};
  std::size_t arm_tracking_cycles{20};
};

struct RobotSnapshot {
  // Physical Panda flange pose in robot base/world O.
  std::array<double, 16> O_T_F{};
  Clock::time_point arrival{};
  bool available{false};
};

struct CommandSnapshot {
  // PBVS target is a physical flange target. It is converted back to the
  // libfranka EE command frame inside the 1 kHz callback using F_T_EE.
  std::array<double, 16> target_O_T_F{};
  Clock::time_point generated{};
  PbvsState state{PbvsState::kWaitForRobot};
  double position_error_m{0.0};
  double proposed_linear_speed_mps{0.0};
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

SafetyConfig load_safety_config(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open safety config: " + path);
  }

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
          "Invalid safety config line " + std::to_string(line_number) +
          ": expected key: value");
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (key.empty() || value.empty()) {
      throw std::runtime_error(
          "Invalid safety config line " + std::to_string(line_number));
    }
    values[key] = value;
  }

  auto require = [&](const char* key) -> const std::string& {
    const auto it = values.find(key);
    if (it == values.end()) {
      throw std::runtime_error(std::string("Missing safety config key: ") + key);
    }
    return it->second;
  };

  SafetyConfig cfg{};
  cfg.motion_enabled = parse_bool(require("motion_enabled"), "motion_enabled");
  cfg.require_tracker_source_filter = parse_bool(
      require("require_tracker_source_filter"), "require_tracker_source_filter");
  cfg.stop_on_tracking_loss = parse_bool(
      require("stop_on_tracking_loss"), "stop_on_tracking_loss");

  cfg.control_x = parse_bool(require("control_x"), "control_x");
  cfg.control_y = parse_bool(require("control_y"), "control_y");
  cfg.control_z = parse_bool(require("control_z"), "control_z");
  cfg.control_rx = parse_bool(require("control_rx"), "control_rx");
  cfg.control_ry = parse_bool(require("control_ry"), "control_ry");
  cfg.control_rz = parse_bool(require("control_rz"), "control_rz");

  cfg.max_arm_wait_s = parse_double(require("max_arm_wait_s"), "max_arm_wait_s");
  cfg.max_runtime_s = parse_double(require("max_runtime_s"), "max_runtime_s");
  cfg.hard_max_linear_speed_mps = parse_double(
      require("hard_max_linear_speed_mps"), "hard_max_linear_speed_mps");
  cfg.hard_max_angular_speed_degps = parse_double(
      require("hard_max_angular_speed_degps"), "hard_max_angular_speed_degps");
  cfg.hard_max_command_lead_m = parse_double(
      require("hard_max_command_lead_m"), "hard_max_command_lead_m");
  cfg.hard_max_angular_command_lead_deg = parse_double(
      require("hard_max_angular_command_lead_deg"),
      "hard_max_angular_command_lead_deg");
  cfg.hard_max_active_position_error_m = parse_double(
      require("hard_max_active_position_error_m"),
      "hard_max_active_position_error_m");
  cfg.hard_max_active_orientation_error_deg = parse_double(
      require("hard_max_active_orientation_error_deg"),
      "hard_max_active_orientation_error_deg");
  cfg.hard_max_tracker_age_s = parse_double(
      require("hard_max_tracker_age_s"), "hard_max_tracker_age_s");
  cfg.max_command_age_s = parse_double(
      require("max_command_age_s"), "max_command_age_s");

  cfg.quality_window_samples = parse_size(
      require("quality_window_samples"), "quality_window_samples");
  cfg.quality_min_samples = parse_size(
      require("quality_min_samples"), "quality_min_samples");
  cfg.tracker_ema_alpha_position = parse_double(
      require("tracker_ema_alpha_position"), "tracker_ema_alpha_position");
  cfg.tracker_ema_alpha_orientation = parse_double(
      require("tracker_ema_alpha_orientation"),
      "tracker_ema_alpha_orientation");
  cfg.tracker_max_position_jump_m = parse_double(
      require("tracker_max_position_jump_m"), "tracker_max_position_jump_m");
  cfg.tracker_max_angle_jump_deg = parse_double(
      require("tracker_max_angle_jump_deg"), "tracker_max_angle_jump_deg");

  cfg.max_task_std_x_m = parse_double(
      require("max_task_std_x_m"), "max_task_std_x_m");
  cfg.max_task_std_y_m = parse_double(
      require("max_task_std_y_m"), "max_task_std_y_m");
  cfg.max_task_std_z_m = parse_double(
      require("max_task_std_z_m"), "max_task_std_z_m");
  cfg.max_task_std_roll_deg = parse_double(
      require("max_task_std_roll_deg"), "max_task_std_roll_deg");
  cfg.max_task_std_pitch_deg = parse_double(
      require("max_task_std_pitch_deg"), "max_task_std_pitch_deg");
  cfg.max_task_std_yaw_deg = parse_double(
      require("max_task_std_yaw_deg"), "max_task_std_yaw_deg");

  cfg.max_travel_x_m = parse_double(
      require("max_travel_x_m"), "max_travel_x_m");
  cfg.max_travel_y_m = parse_double(
      require("max_travel_y_m"), "max_travel_y_m");
  cfg.max_travel_z_m = parse_double(
      require("max_travel_z_m"), "max_travel_z_m");
  cfg.max_rotation_travel_deg = parse_double(
      require("max_rotation_travel_deg"), "max_rotation_travel_deg");

  cfg.min_control_command_success_rate = parse_double(
      require("min_control_command_success_rate"),
      "min_control_command_success_rate");
  cfg.communication_grace_s = parse_double(
      require("communication_grace_s"), "communication_grace_s");
  cfg.arm_tracking_cycles = parse_size(
      require("arm_tracking_cycles"), "arm_tracking_cycles");

  const bool any_axis =
      cfg.control_x || cfg.control_y || cfg.control_z ||
      cfg.control_rx || cfg.control_ry || cfg.control_rz;
  if (!any_axis) {
    throw std::runtime_error("At least one control axis must be enabled");
  }

  if (!(cfg.max_arm_wait_s > 0.0) || !(cfg.max_runtime_s > 0.0) ||
      !(cfg.hard_max_linear_speed_mps > 0.0) ||
      !(cfg.hard_max_angular_speed_degps > 0.0) ||
      !(cfg.hard_max_command_lead_m > 0.0) ||
      !(cfg.hard_max_angular_command_lead_deg > 0.0) ||
      !(cfg.hard_max_active_position_error_m > 0.0) ||
      !(cfg.hard_max_active_orientation_error_deg > 0.0) ||
      !(cfg.hard_max_tracker_age_s > 0.0) ||
      !(cfg.max_command_age_s > 0.0) ||
      !(cfg.tracker_max_position_jump_m > 0.0) ||
      !(cfg.tracker_max_angle_jump_deg > 0.0) ||
      cfg.quality_min_samples > cfg.quality_window_samples ||
      !(cfg.tracker_ema_alpha_position > 0.0 &&
        cfg.tracker_ema_alpha_position <= 1.0) ||
      !(cfg.tracker_ema_alpha_orientation > 0.0 &&
        cfg.tracker_ema_alpha_orientation <= 1.0) ||
      cfg.max_task_std_x_m < 0.0 ||
      cfg.max_task_std_y_m < 0.0 ||
      cfg.max_task_std_z_m < 0.0 ||
      cfg.max_task_std_roll_deg < 0.0 ||
      cfg.max_task_std_pitch_deg < 0.0 ||
      cfg.max_task_std_yaw_deg < 0.0 ||
      cfg.max_travel_x_m < 0.0 ||
      cfg.max_travel_y_m < 0.0 ||
      cfg.max_travel_z_m < 0.0 ||
      cfg.max_rotation_travel_deg < 0.0 ||
      cfg.communication_grace_s < 0.0 ||
      cfg.min_control_command_success_rate < 0.0 ||
      cfg.min_control_command_success_rate > 1.0) {
    throw std::runtime_error("Safety config contains an out-of-range value");
  }
  return cfg;
}

std::uint16_t parse_port(const std::string& value) {
  const long parsed = std::stol(value);
  if (parsed < 1 || parsed > 65535) {
    throw std::invalid_argument("--tracker-port must be in [1, 65535]");
  }
  return static_cast<std::uint16_t>(parsed);
}

void print_help(const char* argv0) {
  std::cout
      << "Translation-first PBVS robot controller.\n\n"
      << "Usage: " << argv0 << " [options]\n"
      << "  --robot-ip IP             Panda FCI IP (default 172.16.0.2)\n"
      << "  --tracker-bind-ip IP      Local UDP bind IP (default 0.0.0.0)\n"
      << "  --tracker-source-ip IP    Required source filter for active motion\n"
      << "  --tracker-port PORT       PTP2 UDP port (default 5000)\n"
      << "  --pbvs-config PATH        PBVS JSON config\n"
      << "  --safe-config PATH        Flat safety YAML config\n"
      << "  --enable-motion           Required explicit runtime enable\n"
      << "  --help                    Show this message\n";
}

Options parse_options(int argc, char** argv) {
  Options options{};
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> std::string {
      if (++i >= argc) throw std::invalid_argument(std::string("Missing value for ") + name);
      return argv[i];
    };

    if (arg == "--robot-ip") options.robot_ip = next("--robot-ip");
    else if (arg == "--tracker-bind-ip") options.tracker_bind_ip = next("--tracker-bind-ip");
    else if (arg == "--tracker-source-ip") options.tracker_source_ip = next("--tracker-source-ip");
    else if (arg == "--tracker-port") options.tracker_port = parse_port(next("--tracker-port"));
    else if (arg == "--pbvs-config") options.pbvs_config_path = next("--pbvs-config");
    else if (arg == "--safe-config") options.safe_config_path = next("--safe-config");
    else if (arg == "--enable-motion") options.enable_motion = true;
    else if (arg == "--help" || arg == "-h") {
      print_help(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }

  if (options.pbvs_config_path.empty()) {
    throw std::invalid_argument("--pbvs-config is required");
  }
  if (options.safe_config_path.empty()) {
    throw std::invalid_argument("--safe-config is required");
  }
  return options;
}

std::array<double, 16> row_major_to_franka(const Transform& transform) {
  std::array<double, 16> column_major{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      column_major[col * 4 + row] = transform[row * 4 + col];
    }
  }
  return column_major;
}

double seconds_between(Clock::time_point later, Clock::time_point earlier) {
  return std::chrono::duration<double>(later - earlier).count();
}

double translation_distance(
    const std::array<double, 16>& a,
    const std::array<double, 16>& b) {
  const double dx = a[12] - b[12];
  const double dy = a[13] - b[13];
  const double dz = a[14] - b[14];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double angular_distance_rad(
    const std::array<double, 16>& a,
    const std::array<double, 16>& b) {
  const Transform T_a = panda_tracker::franka_column_major_transform(a);
  const Transform T_b = panda_tracker::franka_column_major_transform(b);
  const Transform delta = panda_tracker::multiply_transform(
      panda_tracker::invert_transform(T_a), T_b);
  return panda_tracker::vector_norm(
      panda_tracker::so3_log(panda_tracker::transform_rotation(delta)));
}

void move_translation_toward(
    std::array<double, 16>& command,
    const std::array<double, 16>& target,
    double maximum_step_m) {
  const double dx = target[12] - command[12];
  const double dy = target[13] - command[13];
  const double dz = target[14] - command[14];
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (distance < 1e-12) return;
  const double scale = std::min(1.0, maximum_step_m / distance);
  command[12] += dx * scale;
  command[13] += dy * scale;
  command[14] += dz * scale;
}

constexpr double kPi = 3.14159265358979323846;

double deg_to_rad(double deg) {
  return deg * kPi / 180.0;
}

double rad_to_deg(double rad) {
  return rad * 180.0 / kPi;
}

Transform measured_T_CS() {
  // Robot-side measured camera -> stick geometry.
  return {{
      1.0, 0.0, 0.0, -0.040,
      0.0, 1.0, 0.0,  0.000,
      0.0, 0.0, 1.0,  0.183,
      0.0, 0.0, 0.0,  1.000,
  }};
}

std::array<double, 3> rpy_deg(const Transform& T) {
  const double roll = std::atan2(T[9], T[10]);
  const double pitch = std::atan2(-T[8], std::hypot(T[0], T[4]));
  const double yaw = std::atan2(T[4], T[0]);
  return {{rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw)}};
}

double vector_norm3(const std::array<double, 3>& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Transform make_transform_local(
    const panda_tracker::Matrix3& rotation,
    const panda_tracker::Vector3& translation) {
  Transform result = panda_tracker::identity_transform();

  result[0] = rotation[0];
  result[1] = rotation[1];
  result[2] = rotation[2];

  result[4] = rotation[3];
  result[5] = rotation[4];
  result[6] = rotation[5];

  result[8] = rotation[6];
  result[9] = rotation[7];
  result[10] = rotation[8];

  result[3] = translation[0];
  result[7] = translation[1];
  result[11] = translation[2];

  return result;
}

std::array<double, 3> masked_linear(
    const std::array<double, 3>& v,
    const SafetyConfig& s) {
  return {{
      s.control_x ? v[0] : 0.0,
      s.control_y ? v[1] : 0.0,
      s.control_z ? v[2] : 0.0,
  }};
}

std::array<double, 3> masked_angular(
    const std::array<double, 3>& v,
    const SafetyConfig& s) {
  return {{
      s.control_rx ? v[0] : 0.0,
      s.control_ry ? v[1] : 0.0,
      s.control_rz ? v[2] : 0.0,
  }};
}

Transform mask_target_pose(
    const Transform& current,
    const Transform& proposed,
    const SafetyConfig& s) {
  Transform result = current;

  const auto p = panda_tracker::transform_translation(proposed);
  if (s.control_x) result[3] = p[0];
  if (s.control_y) result[7] = p[1];
  if (s.control_z) result[11] = p[2];

  if (s.control_rx || s.control_ry || s.control_rz) {
    const Transform delta = panda_tracker::multiply_transform(
        panda_tracker::invert_transform(current), proposed);
    auto w = panda_tracker::so3_log(
        panda_tracker::transform_rotation(delta));
    if (!s.control_rx) w[0] = 0.0;
    if (!s.control_ry) w[1] = 0.0;
    if (!s.control_rz) w[2] = 0.0;

    const Transform delta_masked = make_transform_local(
        panda_tracker::so3_exp(w), {{0.0, 0.0, 0.0}});
    const Transform rotated = panda_tracker::multiply_transform(
        make_transform_local(
            panda_tracker::transform_rotation(current), {{0.0, 0.0, 0.0}}),
        delta_masked);
    const auto R = panda_tracker::transform_rotation(rotated);
    result[0] = R[0]; result[1] = R[1]; result[2] = R[2];
    result[4] = R[3]; result[5] = R[4]; result[6] = R[5];
    result[8] = R[6]; result[9] = R[7]; result[10] = R[8];
  }
  return result;
}

struct QualitySample {
  std::array<double, 3> p{};
  std::array<double, 3> rpy{};
};

struct TrackerGate {
  std::deque<QualitySample> window;

  // PTP2 wire semantic is pure target pose in camera frame: T_CT.
  // Jump rejection is deliberately performed on this raw tracker pose before
  // applying any robot-side camera/stick geometry.
  std::optional<Transform> last_accepted_T_CT;
  std::optional<Transform> filtered_T_TS;

  std::uint64_t sequence{0};
  std::uint64_t seen_sequence{0};
  Clock::time_point arrival{};
  bool have_accepted{false};
  bool have_seen{false};
  bool latest_packet_accepted{false};
  bool quality_ok{false};
  std::size_t rejected_jumps{0};

  static double stddev(
      const std::deque<QualitySample>& samples,
      int kind,
      std::size_t axis) {
    if (samples.size() < 2) return std::numeric_limits<double>::infinity();
    double mean = 0.0;
    for (const auto& s : samples) {
      mean += kind == 0 ? s.p[axis] : s.rpy[axis];
    }
    mean /= static_cast<double>(samples.size());
    double accum = 0.0;
    for (const auto& s : samples) {
      const double x = kind == 0 ? s.p[axis] : s.rpy[axis];
      const double d = x - mean;
      accum += d * d;
    }
    return std::sqrt(accum / static_cast<double>(samples.size() - 1));
  }

  bool process_new(
      const TrackerSnapshot& tracker,
      const SafetyConfig& s) {
    latest_packet_accepted = false;

    // PTP2 payload semantic is PURE T_CT (target/triangle T expressed in
    // camera C). Do not apply stick geometry before this jump test.
    const Transform T_CT = tracker.packet.T_TS;  // legacy packet member name

    if (last_accepted_T_CT) {
      const Transform jump_CT = panda_tracker::multiply_transform(
          panda_tracker::invert_transform(*last_accepted_T_CT), T_CT);
      const double dp_CT = panda_tracker::vector_norm(
          panda_tracker::transform_translation(jump_CT));
      const double da_CT = panda_tracker::vector_norm(
          panda_tracker::so3_log(panda_tracker::transform_rotation(jump_CT)));

      const bool orientation_active =
          s.control_rx || s.control_ry || s.control_rz;
      if (dp_CT > s.tracker_max_position_jump_m ||
          (orientation_active &&
           da_CT > deg_to_rad(s.tracker_max_angle_jump_deg))) {
        ++rejected_jumps;

        std::cerr
            << "TRACKER RAW_CT REJECT"
            << " seq=" << tracker.packet.sequence_id
            << " dp_CT_mm=" << dp_CT * 1000.0
            << " da_CT_deg=" << rad_to_deg(da_CT)
            << " max_dp_mm=" << s.tracker_max_position_jump_m * 1000.0
            << " max_da_deg=" << s.tracker_max_angle_jump_deg
            << '\n';

        return false;
      }
    }

    // Raw camera/target pose passed the tracker-side geometry gate.
    last_accepted_T_CT = T_CT;
    latest_packet_accepted = true;

    // Robot-side geometry starts only here:
    //   T_TC = inverse(T_CT)
    //   T_TS = T_TC * T_CS
    const Transform T_TC = panda_tracker::invert_transform(T_CT);
    const Transform raw_T_TS = panda_tracker::multiply_transform(
        T_TC, measured_T_CS());

    if (!filtered_T_TS) {
      filtered_T_TS = raw_T_TS;
    } else {
      const auto p_old = panda_tracker::transform_translation(*filtered_T_TS);
      const auto p_new = panda_tracker::transform_translation(raw_T_TS);
      const double a = s.tracker_ema_alpha_position;
      std::array<double, 3> p_f{{
          (1.0 - a) * p_old[0] + a * p_new[0],
          (1.0 - a) * p_old[1] + a * p_new[1],
          (1.0 - a) * p_old[2] + a * p_new[2],
      }};

      auto R_f = panda_tracker::transform_rotation(*filtered_T_TS);
      if (s.control_rx || s.control_ry || s.control_rz) {
        const Transform delta = panda_tracker::multiply_transform(
            panda_tracker::invert_transform(*filtered_T_TS), raw_T_TS);
        auto w = panda_tracker::so3_log(
            panda_tracker::transform_rotation(delta));
        const double ar = s.tracker_ema_alpha_orientation;
        w[0] *= ar; w[1] *= ar; w[2] *= ar;
        const Transform step = make_transform_local(
            panda_tracker::so3_exp(w), {{0.0, 0.0, 0.0}});
        const Transform old_R_only = make_transform_local(
            R_f, {{0.0, 0.0, 0.0}});
        R_f = panda_tracker::transform_rotation(
            panda_tracker::multiply_transform(old_R_only, step));
      } else {
        // Orientation is intentionally not used by translation-only PBVS.
        R_f = panda_tracker::transform_rotation(raw_T_TS);
      }
      filtered_T_TS = make_transform_local(R_f, p_f);
    }

    QualitySample q{};
    q.p = panda_tracker::transform_translation(raw_T_TS);
    q.rpy = rpy_deg(raw_T_TS);
    window.push_back(q);
    while (window.size() > s.quality_window_samples) window.pop_front();

    const bool x_only_moving_target =
        s.control_x && !s.control_y && !s.control_z &&
        !s.control_rx && !s.control_ry && !s.control_rz;
    if (x_only_moving_target) {
      // A moving target is expected to have non-zero window spread. In this
      // deliberately X-only experiment, rely on validity, freshness, raw T_CT
      // jump rejection, active-axis error checks and the hard motion caps.
      quality_ok = true;
    } else {
      quality_ok = window.size() >= s.quality_min_samples;
      if (quality_ok) {
        quality_ok =
            stddev(window, 0, 0) <= s.max_task_std_x_m &&
            stddev(window, 0, 1) <= s.max_task_std_y_m &&
            stddev(window, 0, 2) <= s.max_task_std_z_m &&
            stddev(window, 1, 0) <= s.max_task_std_roll_deg &&
            stddev(window, 1, 1) <= s.max_task_std_pitch_deg &&
            stddev(window, 1, 2) <= s.max_task_std_yaw_deg;
      }
    }

    sequence = tracker.packet.sequence_id;
    arrival = tracker.arrival;
    have_accepted = true;
    return true;
  }
};

struct ConservativeXMotionLimiter {
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};

  static constexpr double kMaxSpeedMps = 0.001;       // 1 mm/s
  static constexpr double kMaxAccelMps2 = 0.002;      // 2 mm/s^2
  static constexpr double kMaxJerkMps3 = 0.020;       // 20 mm/s^3
  static constexpr double kMaxCommandLeadM = 0.0005;  // 0.5 mm
  static constexpr double kMaxActiveErrorM = 0.050;   // 50 mm
  static constexpr double kStoppedSpeedMps = 1e-6;
  static constexpr double kStoppedAccelMps2 = 1e-5;

  void step_toward(
      std::array<double, 16>& command,
      double target_x_m,
      double dt_s,
      double configured_speed_cap_mps) {
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) return;

    const double vmax = std::min(configured_speed_cap_mps, kMaxSpeedMps);
    const double error = target_x_m - command[12];
    if (std::abs(error) < 1e-9 &&
        std::abs(velocity_mps) < kStoppedSpeedMps) {
      velocity_mps = 0.0;
      acceleration_mps2 = 0.0;
      return;
    }

    // Braking-aware desired speed: never ask for a speed that cannot be
    // stopped within the remaining X error using our conservative accel cap.
    const double braking_speed =
        std::sqrt(std::max(0.0, 2.0 * kMaxAccelMps2 * std::abs(error)));
    double desired_velocity = std::min(vmax, braking_speed);
    if (error < 0.0) desired_velocity = -desired_velocity;

    const double desired_acceleration = std::clamp(
        (desired_velocity - velocity_mps) / dt_s,
        -kMaxAccelMps2,
        kMaxAccelMps2);

    const double max_da = kMaxJerkMps3 * dt_s;
    acceleration_mps2 += std::clamp(
        desired_acceleration - acceleration_mps2,
        -max_da,
        max_da);
    acceleration_mps2 = std::clamp(
        acceleration_mps2, -kMaxAccelMps2, kMaxAccelMps2);

    const double old_velocity = velocity_mps;
    double new_velocity = std::clamp(
        velocity_mps + acceleration_mps2 * dt_s, -vmax, vmax);
    double dx = 0.5 * (old_velocity + new_velocity) * dt_s;

    // Never cross the requested X target.
    if ((error > 0.0 && dx >= error) ||
        (error < 0.0 && dx <= error)) {
      command[12] = target_x_m;
      velocity_mps = 0.0;
      acceleration_mps2 = 0.0;
      return;
    }

    command[12] += dx;
    velocity_mps = new_velocity;
  }

  bool step_to_stop(std::array<double, 16>& command, double dt_s) {
    if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
      return std::abs(velocity_mps) <= kStoppedSpeedMps;
    }

    if (std::abs(velocity_mps) <= kStoppedSpeedMps &&
        std::abs(acceleration_mps2) <= kStoppedAccelMps2) {
      velocity_mps = 0.0;
      acceleration_mps2 = 0.0;
      return true;
    }

    const double desired_acceleration = std::clamp(
        -velocity_mps / dt_s, -kMaxAccelMps2, kMaxAccelMps2);
    const double max_da = kMaxJerkMps3 * dt_s;
    acceleration_mps2 += std::clamp(
        desired_acceleration - acceleration_mps2,
        -max_da,
        max_da);
    acceleration_mps2 = std::clamp(
        acceleration_mps2, -kMaxAccelMps2, kMaxAccelMps2);

    const double old_velocity = velocity_mps;
    double new_velocity = velocity_mps + acceleration_mps2 * dt_s;
    if ((old_velocity > 0.0 && new_velocity < 0.0) ||
        (old_velocity < 0.0 && new_velocity > 0.0)) {
      new_velocity = 0.0;
      acceleration_mps2 = 0.0;
    }

    command[12] += 0.5 * (old_velocity + new_velocity) * dt_s;
    velocity_mps = new_velocity;

    if (std::abs(velocity_mps) <= kStoppedSpeedMps &&
        std::abs(acceleration_mps2) <= kStoppedAccelMps2) {
      velocity_mps = 0.0;
      acceleration_mps2 = 0.0;
      return true;
    }
    return false;
  }
};

bool startup_envelope_ok(
    const std::array<double, 16>& current_O_T_F,
    const std::array<double, 16>& start_O_T_F,
    const SafetyConfig& s,
    double* dx_m_out = nullptr,
    double* dy_m_out = nullptr,
    double* dz_m_out = nullptr,
    double* angle_rad_out = nullptr) {
  // Application-level envelope is never allowed to be looser than these
  // conservative X-only experiment caps, even if YAML requests more.
  const double max_x = std::min(s.max_travel_x_m, 0.006);  // 6 mm
  // Y/Z are not commanded in this experiment. Allow 2 mm of measured
  // startup/impedance settling before stopping; this is still a tight envelope.
  const double max_y = std::min(s.max_travel_y_m, 0.002);  // 2 mm
  const double max_z = std::min(s.max_travel_z_m, 0.002);  // 2 mm

  const double dx_m = std::abs(current_O_T_F[12] - start_O_T_F[12]);
  const double dy_m = std::abs(current_O_T_F[13] - start_O_T_F[13]);
  const double dz_m = std::abs(current_O_T_F[14] - start_O_T_F[14]);

  const Transform current =
      panda_tracker::franka_column_major_transform(current_O_T_F);
  const Transform start =
      panda_tracker::franka_column_major_transform(start_O_T_F);
  const Transform delta = panda_tracker::multiply_transform(
      panda_tracker::invert_transform(start), current);
  const double angle = panda_tracker::vector_norm(
      panda_tracker::so3_log(panda_tracker::transform_rotation(delta)));
  const double max_angle = std::min(s.max_rotation_travel_deg, 1.0);

  if (dx_m_out) *dx_m_out = dx_m;
  if (dy_m_out) *dy_m_out = dy_m;
  if (dz_m_out) *dz_m_out = dz_m;
  if (angle_rad_out) *angle_rad_out = angle;

  return dx_m <= max_x &&
         dy_m <= max_y &&
         dz_m <= max_z &&
         angle <= deg_to_rad(max_angle);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const SafetyConfig safety = load_safety_config(options.safe_config_path);

    PbvsConfig pbvs_config{};
    std::string config_error;
    if (!panda_tracker::load_pbvs_config(
            options.pbvs_config_path, pbvs_config, config_error)) {
      throw std::runtime_error("Unable to load PBVS config: " + config_error);
    }

    // Double-enable: file + command line.
    if (!safety.motion_enabled || !options.enable_motion) {
      throw std::runtime_error(
          "Motion is disabled. Set motion_enabled: true in safe_config.yml "
          "AND pass --enable-motion.");
    }
    if (safety.require_tracker_source_filter && options.tracker_source_ip.empty()) {
      throw std::runtime_error(
          "Active motion requires --tracker-source-ip when "
          "require_tracker_source_filter is true.");
    }
    if (!(safety.control_x && !safety.control_y && !safety.control_z &&
          !safety.control_rx && !safety.control_ry && !safety.control_rz)) {
      throw std::runtime_error(
          "This conservative build is intentionally X-only: enable X and disable Y/Z/Rx/Ry/Rz.");
    }
    const bool orientation_axes_enabled =
        safety.control_rx || safety.control_ry || safety.control_rz;
    if (pbvs_config.control_orientation != orientation_axes_enabled) {
      throw std::runtime_error(
          "PBVS control_orientation must match the safety axis mask: set it "
          "true iff at least one of control_rx/control_ry/control_rz is true.");
    }
    if (pbvs_config.max_linear_speed > safety.hard_max_linear_speed_mps + 1e-12) {
      throw std::runtime_error(
          "PBVS max_linear_speed exceeds hard_max_linear_speed_mps in safe_config.yml.");
    }
    if (pbvs_config.max_command_lead > safety.hard_max_command_lead_m + 1e-12) {
      throw std::runtime_error(
          "PBVS max_command_lead exceeds hard_max_command_lead_m in safe_config.yml.");
    }
    // PBVS may use a larger whole-task enable threshold during one-axis
    // experiments. The wrapper independently checks ONLY the enabled command
    // axes against hard_max_active_position_error_m before arming.
    if (pbvs_config.tracker_timeout > safety.hard_max_tracker_age_s + 1e-12) {
      throw std::runtime_error(
          "PBVS tracker_timeout exceeds hard_max_tracker_age_s in safe_config.yml.");
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    TrackerReceiver tracker_receiver(
        options.tracker_bind_ip,
        options.tracker_source_ip,
        options.tracker_port);
    PbvsController pbvs(pbvs_config);

    std::mutex tracker_mutex;
    TrackerSnapshot shared_tracker{};
    std::mutex robot_mutex;
    RobotSnapshot shared_robot{};
    std::mutex command_mutex;
    CommandSnapshot shared_command{};

    std::atomic_bool workers_running{true};
    std::exception_ptr worker_failure;
    std::mutex failure_mutex;

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
        g_stop_requested.store(true, std::memory_order_relaxed);
      }
    });

    TrackerGate tracker_gate{};

    std::thread pbvs_thread([&]() {
      try {
        const auto period = std::chrono::duration<double>(1.0 / pbvs_config.control_rate_hz);
        auto next = Clock::now();
        auto last = next - std::chrono::duration_cast<Clock::duration>(period);
        std::size_t tracking_cycles = 0;
        auto next_log = next;

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

          // The PBVS core historically calls this T_BE. For the active
          // flange-based runtime, E is deliberately the physical flange F.
          std::optional<Transform> T_BE;
          double robot_age_s = std::numeric_limits<double>::infinity();
          if (robot.available) {
            T_BE = panda_tracker::franka_column_major_transform(robot.O_T_F);
            robot_age_s = seconds_between(now, robot.arrival);
          }

          std::optional<TaskPoseMeasurement> measurement;
          double tracker_age_s = std::numeric_limits<double>::infinity();

          if (tracker.available &&
              (!tracker_gate.have_seen ||
               tracker.packet.sequence_id != tracker_gate.seen_sequence)) {
            // Gate/filter work happens exactly once per NEW ~30 Hz packet,
            // including invalid/rejected packets.
            tracker_gate.have_seen = true;
            tracker_gate.seen_sequence = tracker.packet.sequence_id;
            if (tracker.packet.valid) {
              tracker_gate.process_new(tracker, safety);
            }
            // Invalid heartbeat: keep the last accepted measurement. The
            // hard tracker-age timeout below decides when it is unusable.
          }

          if (tracker_gate.have_accepted && tracker_gate.filtered_T_TS) {
            tracker_age_s = seconds_between(now, tracker_gate.arrival);
            TaskPoseMeasurement m{};
            m.T_TS = *tracker_gate.filtered_T_TS;
            m.timestamp_s = std::chrono::duration<double>(
                tracker_gate.arrival.time_since_epoch()).count();
            m.valid = tracker_gate.quality_ok &&
                      tracker_age_s >= 0.0 &&
                      tracker_age_s <= safety.hard_max_tracker_age_s;
            m.sequence_id = tracker_gate.sequence;
            measurement = m;
          }

          const double now_s = std::chrono::duration<double>(
              now.time_since_epoch()).count();
          double dt_s = seconds_between(now, last);
          if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
            dt_s = 1.0 / pbvs_config.control_rate_hz;
          }
          last = now;

          PbvsResult result = pbvs.step(T_BE, robot_age_s, measurement, now_s, dt_s);

          const bool tracker_fresh =
              tracker_gate.have_accepted &&
              tracker_gate.quality_ok &&
              tracker_age_s >= 0.0 &&
              tracker_age_s <= safety.hard_max_tracker_age_s;

          const auto active_p_error = masked_linear(
              result.position_error_base_m, safety);
          const auto active_r_error = masked_angular(
              result.orientation_error_body_rad, safety);
          const auto active_v = masked_linear(
              result.proposed_linear_velocity_base_mps, safety);
          const auto active_w = masked_angular(
              result.proposed_angular_velocity_body_radps, safety);

          const double active_p_error_norm = vector_norm3(active_p_error);
          const double active_r_error_norm = vector_norm3(active_r_error);
          const double active_v_norm = vector_norm3(active_v);
          const double active_w_norm = vector_norm3(active_w);

          const bool safe_tracking =
              result.state == PbvsState::kTracking &&
              result.has_proposed_pose &&
              tracker_fresh &&
              active_p_error_norm <=
                  std::min(safety.hard_max_active_position_error_m,
                           ConservativeXMotionLimiter::kMaxActiveErrorM) &&
              active_r_error_norm <=
                  deg_to_rad(safety.hard_max_active_orientation_error_deg) &&
              active_v_norm <= safety.hard_max_linear_speed_mps + 1e-9 &&
              active_w_norm <=
                  deg_to_rad(safety.hard_max_angular_speed_degps) + 1e-9;

          if (safe_tracking) ++tracking_cycles;
          else tracking_cycles = 0;

          CommandSnapshot command{};
          command.state = result.state;
          command.position_error_m = result.position_error_norm_m;
          command.proposed_linear_speed_mps = result.proposed_linear_speed_mps;
          command.tracker_sequence = tracker.available ? tracker.packet.sequence_id : 0;
          command.generated = now;
          command.armed = safe_tracking && tracking_cycles >= safety.arm_tracking_cycles;
          if (safe_tracking && T_BE) {
            // Enforce the config axis mask independently of PBVS.
            const Transform masked = mask_target_pose(
                *T_BE, result.proposed_T_BE, safety);
            command.target_O_T_F = row_major_to_franka(masked);
            command.target_available = true;
          }
          {
            std::lock_guard<std::mutex> lock(command_mutex);
            shared_command = command;
          }

          if (now >= next_log) {
            std::cout << std::fixed << std::setprecision(4)
                      << "pbvs=" << panda_tracker::pbvs_state_text(result.state)
                      << " tracker_seq=" << command.tracker_sequence
                      << " tracker_age_ms=" << tracker_age_s * 1000.0
                      << " quality=" << (tracker_gate.quality_ok ? "PASS" : "hold")
                      << " rejected_jumps=" << tracker_gate.rejected_jumps
                      << " active_p_err_mm=" << active_p_error_norm * 1000.0
                      << " active_r_err_deg=" << rad_to_deg(active_r_error_norm)
                      << " active_v_mmps=" << active_v_norm * 1000.0
                      << " active_w_degps=" << rad_to_deg(active_w_norm)
                      << " arm_cycles=" << tracking_cycles
                      << " armed=" << (command.armed ? "YES" : "no");
            if (!result.reason.empty()) std::cout << " reason=" << result.reason;
            std::cout << '\n';
            next_log = now + std::chrono::seconds(1);
          }

          next = now + std::chrono::duration_cast<Clock::duration>(period);
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(failure_mutex);
        worker_failure = std::current_exception();
        g_stop_requested.store(true, std::memory_order_relaxed);
      }
    });

    std::cout
        << "ACTIVE PBVS ROBOT CONTROLLER\n"
        << "Robot: " << options.robot_ip << '\n'
        << "Tracker: " << options.tracker_bind_ip << ':' << options.tracker_port
        << " source=" << options.tracker_source_ip << '\n'
        << "Robot PBVS frame: PHYSICAL PANDA FLANGE F\n"
        << "Axes: "
        << (safety.control_x ? "X " : "")
        << (safety.control_y ? "Y " : "")
        << (safety.control_z ? "Z " : "")
        << (safety.control_rx ? "Rx " : "")
        << (safety.control_ry ? "Ry " : "")
        << (safety.control_rz ? "Rz " : "") << '\n'
        << "Configured hard max linear speed: "
        << safety.hard_max_linear_speed_mps * 1000.0 << " mm/s\n"
        << "Effective conservative X speed cap: "
        << std::min(safety.hard_max_linear_speed_mps,
                    ConservativeXMotionLimiter::kMaxSpeedMps) * 1000.0
        << " mm/s\n"
        << "Conservative X accel cap: "
        << ConservativeXMotionLimiter::kMaxAccelMps2 * 1000.0 << " mm/s^2\n"
        << "Conservative X jerk cap: "
        << ConservativeXMotionLimiter::kMaxJerkMps3 * 1000.0 << " mm/s^3\n"
        << "Effective command lead cap: "
        << std::min(safety.hard_max_command_lead_m,
                    ConservativeXMotionLimiter::kMaxCommandLeadM) * 1000.0
        << " mm\n"
        << "Effective active X error arm cap: "
        << std::min(safety.hard_max_active_position_error_m,
                    ConservativeXMotionLimiter::kMaxActiveErrorM) * 1000.0
        << " mm\n"
        << "Effective startup travel envelope: X<=6 mm, Y/Z<=2 mm, R<=1 deg\n"
        << "Hard max angular speed: "
        << safety.hard_max_angular_speed_degps << " deg/s\n"
        << "Hard max lead: " << safety.hard_max_command_lead_m * 1000.0 << " mm\n"
        << "Max runtime: " << safety.max_runtime_s << " s\n"
        << "Factory Panda collision/reflex safety: NOT OVERRIDDEN\n"
        << "libfranka command rate limiting: ENABLED\n"
        << "Waiting for safe PBVS arming...\n";

    std::exception_ptr robot_failure;

    try {
      // Diagnostic mode: user explicitly requested ignoring realtime-kernel
      // capability for now. Restore kEnforce before real PBVS motion tests.
      franka::Robot robot(options.robot_ip, franka::RealtimeConfig::kIgnore);

      // Error recovery does not disable or raise Panda factory reflex/limit
      // thresholds. We intentionally do not call setCollisionBehavior(),
      // setJointImpedance(), setCartesianImpedance(), or any limit override.
      robot.automaticErrorRecovery();
      std::cout << "Robot error recovery complete; factory safety/reflex limits unchanged.\n";

      const auto control_start = Clock::now();
      bool ever_armed = false;
      std::array<double, 16> command_pose{};
      bool command_initialized = false;
      std::array<double, 16> startup_O_T_F{};
      bool startup_flange_initialized = false;
      ConservativeXMotionLimiter x_motion_limiter{};
      bool soft_stop_requested = false;
      std::string soft_stop_reason;
      bool soft_stop_logged = false;

      auto request_soft_stop = [&](const std::string& reason) {
        if (!soft_stop_requested) {
          soft_stop_requested = true;
          soft_stop_reason = reason;
        }
      };

      robot.control(
          [&](const franka::RobotState& state, franka::Duration period)
              -> franka::CartesianPose {
            const auto now = Clock::now();
            const double dt = std::max(0.0, period.toSec());

            // IMPORTANT: initialize from the MEASURED current EE pose, not
            // O_T_EE_c (the previous commanded pose). After a prior abort or
            // reflex, O_T_EE_c can differ from the actual robot pose and using
            // it here can create an immediate startup motion/discontinuity.
            if (!command_initialized) {
              command_pose = state.O_T_EE;
              command_initialized = true;
            }

            // Convert libfranka's configured EE pose to the physical flange:
            // O_T_F = O_T_EE * inverse(F_T_EE)
            const Transform O_T_EE_rm =
                panda_tracker::franka_column_major_transform(state.O_T_EE);
            const Transform F_T_EE_rm =
                panda_tracker::franka_column_major_transform(state.F_T_EE);
            const Transform O_T_F_rm = panda_tracker::multiply_transform(
                O_T_EE_rm, panda_tracker::invert_transform(F_T_EE_rm));
            const std::array<double, 16> O_T_F = row_major_to_franka(O_T_F_rm);

            if (!startup_flange_initialized) {
              startup_O_T_F = O_T_F;
              startup_flange_initialized = true;
            }

            if (robot_mutex.try_lock()) {
              shared_robot.O_T_F = O_T_F;
              shared_robot.arrival = now;
              shared_robot.available = true;
              robot_mutex.unlock();
            }

            const double elapsed_s = seconds_between(now, control_start);

            // Convert all application-level stops into a jerk/accel-limited
            // soft stop first. Factory/libfranka safety reflexes remain active
            // independently and can still stop the robot immediately.
            if (!soft_stop_requested && startup_flange_initialized) {
              double dx_m = 0.0, dy_m = 0.0, dz_m = 0.0, da_rad = 0.0;
              if (!startup_envelope_ok(
                      O_T_F, startup_O_T_F, safety,
                      &dx_m, &dy_m, &dz_m, &da_rad)) {
                std::cerr
                    << "STARTUP ENVELOPE EXCEEDED"
                    << " dx_mm=" << dx_m * 1000.0
                    << " dy_mm=" << dy_m * 1000.0
                    << " dz_mm=" << dz_m * 1000.0
                    << " da_deg=" << rad_to_deg(da_rad)
                    << " limits=[6,2,2]mm,1deg\n";
                request_soft_stop("startup flange travel envelope exceeded");
              }
            }
            if (g_stop_requested.load(std::memory_order_relaxed)) {
              request_soft_stop("stop requested");
            }
            if (elapsed_s >= safety.max_runtime_s) {
              request_soft_stop("max runtime reached");
            }
            if (!ever_armed && elapsed_s >= safety.max_arm_wait_s) {
              request_soft_stop("arming timeout");
            }
            if (elapsed_s >= safety.communication_grace_s &&
                state.control_command_success_rate <
                    safety.min_control_command_success_rate) {
              request_soft_stop("control command success rate below threshold");
            }

            CommandSnapshot command{};
            bool got_command = false;
            if (command_mutex.try_lock()) {
              command = shared_command;
              command_mutex.unlock();
              got_command = true;
            }

            if (!soft_stop_requested &&
                got_command && command.armed && command.target_available) {
              if (seconds_between(now, command.generated) >
                  safety.max_command_age_s) {
                request_soft_stop("command stale");
              } else {
                ever_armed = true;

                // Independent lead checks remain in the physical flange frame.
                if (translation_distance(command.target_O_T_F, O_T_F) >
                    safety.hard_max_command_lead_m + 1e-6) {
                  request_soft_stop("flange translation command lead exceeded");
                }
                if (angular_distance_rad(command.target_O_T_F, O_T_F) >
                    deg_to_rad(safety.hard_max_angular_command_lead_deg) + 1e-6) {
                  request_soft_stop("flange angular command lead exceeded");
                }

                if (!soft_stop_requested) {
                  // Convert physical flange target back to libfranka EE frame.
                  const Transform target_O_T_F_rm =
                      panda_tracker::franka_column_major_transform(
                          command.target_O_T_F);
                  const Transform target_O_T_EE_rm =
                      panda_tracker::multiply_transform(
                          target_O_T_F_rm, F_T_EE_rm);
                  const std::array<double, 16> target_O_T_EE =
                      row_major_to_franka(target_O_T_EE_rm);

                  // CONSERVATIVE X-ONLY command generation.
                  // Y/Z and orientation remain exactly at the initial commanded
                  // EE pose. X uses explicit speed, acceleration and jerk caps.
                  const double effective_lead = std::min(
                      safety.hard_max_command_lead_m,
                      ConservativeXMotionLimiter::kMaxCommandLeadM);
                  const double measured_ee_x = state.O_T_EE[12];
                  const double conservative_target_x = std::clamp(
                      target_O_T_EE[12],
                      measured_ee_x - effective_lead,
                      measured_ee_x + effective_lead);

                  x_motion_limiter.step_toward(
                      command_pose,
                      conservative_target_x,
                      dt,
                      safety.hard_max_linear_speed_mps);
                }
              }
            } else if (!soft_stop_requested &&
                       ever_armed && safety.stop_on_tracking_loss && got_command) {
              request_soft_stop("tracking lost after arming");
            }

            if (soft_stop_requested) {
              if (!soft_stop_logged) {
                std::cerr << "SOFT STOP: " << soft_stop_reason << '\n';
                soft_stop_logged = true;
              }
              if (x_motion_limiter.step_to_stop(command_pose, dt)) {
                std::cerr << "STOPPED: X velocity/acceleration ramped to zero\n";
                return franka::MotionFinished(franka::CartesianPose(command_pose));
              }
              return franka::CartesianPose(command_pose);
            }

            return franka::CartesianPose(command_pose);
          },
          franka::ControllerMode::kCartesianImpedance,
          true);  // keep libfranka internal command rate limiting enabled

    } catch (...) {
      // Do not rethrow until both worker threads have been stopped and joined.
      robot_failure = std::current_exception();
    }

    // Always stop/join workers, including when franka::Robot or robot.control() throws.
    workers_running.store(false, std::memory_order_relaxed);
    g_stop_requested.store(true, std::memory_order_relaxed);

    if (tracker_thread.joinable()) {
      tracker_thread.join();
    }
    if (pbvs_thread.joinable()) {
      pbvs_thread.join();
    }

    // Only now is it safe to propagate the real robot/control exception.
    if (robot_failure) {
      std::rethrow_exception(robot_failure);
    }

    {
      std::lock_guard<std::mutex> lock(failure_mutex);
      if (worker_failure) {
        std::rethrow_exception(worker_failure);
      }
    }

    std::cout << "Robot control loop ended safely.\n";
    return EXIT_SUCCESS;
  } catch (const franka::Exception& exception) {
    std::cerr << "Franka error: " << exception.what() << '\n';
  } catch (const std::exception& exception) {
    std::cerr << "Error: " << exception.what() << '\n';
  }
  return EXIT_FAILURE;
}
