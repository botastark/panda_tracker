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
  bool translation_only{true};
  bool require_tracker_source_filter{true};
  bool stop_on_tracking_loss{true};
  bool allow_orientation_motion{false};

  double max_arm_wait_s{15.0};
  double max_runtime_s{10.0};
  double hard_max_linear_speed_mps{0.010};
  double hard_max_command_lead_m{0.002};
  double hard_max_position_error_m{0.080};
  double hard_max_tracker_age_s{0.120};
  double max_command_age_s{0.050};
  double min_control_command_success_rate{0.95};
  double communication_grace_s{0.50};
  std::size_t arm_tracking_cycles{20};
};

struct RobotSnapshot {
  std::array<double, 16> O_T_EE{};
  Clock::time_point arrival{};
  bool available{false};
};

struct CommandSnapshot {
  std::array<double, 16> target_O_T_EE{};
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
  cfg.translation_only = parse_bool(require("translation_only"), "translation_only");
  cfg.require_tracker_source_filter = parse_bool(
      require("require_tracker_source_filter"), "require_tracker_source_filter");
  cfg.stop_on_tracking_loss = parse_bool(
      require("stop_on_tracking_loss"), "stop_on_tracking_loss");
  cfg.allow_orientation_motion = parse_bool(
      require("allow_orientation_motion"), "allow_orientation_motion");

  cfg.max_arm_wait_s = parse_double(require("max_arm_wait_s"), "max_arm_wait_s");
  cfg.max_runtime_s = parse_double(require("max_runtime_s"), "max_runtime_s");
  cfg.hard_max_linear_speed_mps = parse_double(
      require("hard_max_linear_speed_mps"), "hard_max_linear_speed_mps");
  cfg.hard_max_command_lead_m = parse_double(
      require("hard_max_command_lead_m"), "hard_max_command_lead_m");
  cfg.hard_max_position_error_m = parse_double(
      require("hard_max_position_error_m"), "hard_max_position_error_m");
  cfg.hard_max_tracker_age_s = parse_double(
      require("hard_max_tracker_age_s"), "hard_max_tracker_age_s");
  cfg.max_command_age_s = parse_double(
      require("max_command_age_s"), "max_command_age_s");
  cfg.min_control_command_success_rate = parse_double(
      require("min_control_command_success_rate"),
      "min_control_command_success_rate");
  cfg.communication_grace_s = parse_double(
      require("communication_grace_s"), "communication_grace_s");
  cfg.arm_tracking_cycles = parse_size(
      require("arm_tracking_cycles"), "arm_tracking_cycles");

  if (!(cfg.max_arm_wait_s > 0.0) || !(cfg.max_runtime_s > 0.0) ||
      !(cfg.hard_max_linear_speed_mps > 0.0) ||
      !(cfg.hard_max_command_lead_m > 0.0) ||
      !(cfg.hard_max_position_error_m > 0.0) ||
      !(cfg.hard_max_tracker_age_s > 0.0) ||
      !(cfg.max_command_age_s > 0.0) ||
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
    if (!safety.translation_only || safety.allow_orientation_motion) {
      throw std::runtime_error(
          "This first-motion controller only supports translation-only mode. "
          "Keep translation_only: true and allow_orientation_motion: false.");
    }
    if (pbvs_config.control_orientation) {
      throw std::runtime_error(
          "For the first motion test set control_orientation=false in the PBVS config.");
    }
    if (pbvs_config.max_linear_speed > safety.hard_max_linear_speed_mps + 1e-12) {
      throw std::runtime_error(
          "PBVS max_linear_speed exceeds hard_max_linear_speed_mps in safe_config.yml.");
    }
    if (pbvs_config.max_command_lead > safety.hard_max_command_lead_m + 1e-12) {
      throw std::runtime_error(
          "PBVS max_command_lead exceeds hard_max_command_lead_m in safe_config.yml.");
    }
    if (pbvs_config.max_enable_position_error > safety.hard_max_position_error_m + 1e-12) {
      throw std::runtime_error(
          "PBVS max_enable_position_error exceeds the hard safety cap.");
    }
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

          std::optional<Transform> T_BE;
          double robot_age_s = std::numeric_limits<double>::infinity();
          if (robot.available) {
            T_BE = panda_tracker::franka_column_major_transform(robot.O_T_EE);
            robot_age_s = seconds_between(now, robot.arrival);
          }

          std::optional<TaskPoseMeasurement> measurement;
          double tracker_age_s = std::numeric_limits<double>::infinity();
          if (tracker.available) {
            tracker_age_s = seconds_between(now, tracker.arrival);
            TaskPoseMeasurement m{};
            m.T_TS = tracker.packet.T_TS;
            m.timestamp_s = std::chrono::duration<double>(
                tracker.arrival.time_since_epoch()).count();
            // Confidence is not used as a graded safety signal. The publisher's
            // valid flag plus geometric/freshness checks are the safety gates.
            m.valid = tracker.packet.valid;
            m.sequence_id = tracker.packet.sequence_id;
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

          const bool tracker_fresh = tracker.available &&
              tracker.packet.valid &&
              tracker_age_s >= 0.0 &&
              tracker_age_s <= safety.hard_max_tracker_age_s;
          const bool safe_tracking =
              result.state == PbvsState::kTracking &&
              result.has_proposed_pose &&
              tracker_fresh &&
              result.position_error_norm_m <= safety.hard_max_position_error_m &&
              result.proposed_linear_speed_mps <= safety.hard_max_linear_speed_mps + 1e-9 &&
              result.proposed_command_lead_m <= safety.hard_max_command_lead_m + 1e-9;

          if (safe_tracking) ++tracking_cycles;
          else tracking_cycles = 0;

          CommandSnapshot command{};
          command.state = result.state;
          command.position_error_m = result.position_error_norm_m;
          command.proposed_linear_speed_mps = result.proposed_linear_speed_mps;
          command.tracker_sequence = tracker.available ? tracker.packet.sequence_id : 0;
          command.generated = now;
          command.armed = safe_tracking && tracking_cycles >= safety.arm_tracking_cycles;
          if (safe_tracking) {
            command.target_O_T_EE = row_major_to_franka(result.proposed_T_BE);
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
                      << " p_err_mm=" << result.position_error_norm_m * 1000.0
                      << " v_mmps=" << result.proposed_linear_speed_mps * 1000.0
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
        << "Mode: TRANSLATION ONLY\n"
        << "Hard max speed: " << safety.hard_max_linear_speed_mps * 1000.0 << " mm/s\n"
        << "Hard max lead: " << safety.hard_max_command_lead_m * 1000.0 << " mm\n"
        << "Max runtime: " << safety.max_runtime_s << " s\n"
        << "Waiting for safe PBVS arming...\n";

    franka::Robot robot(options.robot_ip);
    const auto control_start = Clock::now();
    bool ever_armed = false;
    std::array<double, 16> command_pose{};
    bool command_initialized = false;

    robot.control(
        [&](const franka::RobotState& state, franka::Duration period)
            -> franka::CartesianPose {
          const auto now = Clock::now();

          if (robot_mutex.try_lock()) {
            shared_robot.O_T_EE = state.O_T_EE;
            shared_robot.arrival = now;
            shared_robot.available = true;
            robot_mutex.unlock();
          }

          if (!command_initialized) {
            command_pose = state.O_T_EE_c;
            command_initialized = true;
          }

          const double elapsed_s = seconds_between(now, control_start);
          if (g_stop_requested.load(std::memory_order_relaxed) ||
              elapsed_s >= safety.max_runtime_s) {
            return franka::MotionFinished(franka::CartesianPose(command_pose));
          }
          if (!ever_armed && elapsed_s >= safety.max_arm_wait_s) {
            return franka::MotionFinished(franka::CartesianPose(command_pose));
          }
          if (elapsed_s >= safety.communication_grace_s &&
              state.control_command_success_rate <
                  safety.min_control_command_success_rate) {
            return franka::MotionFinished(franka::CartesianPose(command_pose));
          }

          CommandSnapshot command{};
          bool got_command = false;
          if (command_mutex.try_lock()) {
            command = shared_command;
            command_mutex.unlock();
            got_command = true;
          }

          if (got_command && command.armed && command.target_available) {
            if (seconds_between(now, command.generated) > safety.max_command_age_s) {
              return franka::MotionFinished(franka::CartesianPose(command_pose));
            }
            ever_armed = true;

            // Independent lead check against current measured EE position.
            if (translation_distance(command.target_O_T_EE, state.O_T_EE) >
                safety.hard_max_command_lead_m + 1e-6) {
              return franka::MotionFinished(franka::CartesianPose(command_pose));
            }

            const double dt = std::max(0.0, period.toSec());
            const double max_step = safety.hard_max_linear_speed_mps * dt;
            move_translation_toward(command_pose, command.target_O_T_EE, max_step);

            // Translation-only invariant: preserve the orientation and homogeneous
            // row established by the first commanded pose.
            command_pose[3] = 0.0;
            command_pose[7] = 0.0;
            command_pose[11] = 0.0;
            command_pose[15] = 1.0;
          } else if (ever_armed && safety.stop_on_tracking_loss && got_command) {
            return franka::MotionFinished(franka::CartesianPose(command_pose));
          }

          return franka::CartesianPose(command_pose);
        },
        franka::ControllerMode::kCartesianImpedance,
        true);

    workers_running.store(false, std::memory_order_relaxed);
    g_stop_requested.store(true, std::memory_order_relaxed);
    tracker_thread.join();
    pbvs_thread.join();

    {
      std::lock_guard<std::mutex> lock(failure_mutex);
      if (worker_failure) std::rethrow_exception(worker_failure);
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
