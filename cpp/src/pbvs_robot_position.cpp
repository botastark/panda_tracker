#include "panda_tracker/position_servo.h"
#include "panda_tracker/position_tracking_config.h"
#include "panda_tracker/robot_safety.h"
#include "panda_tracker/tracker_position_filter.h"
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
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
using panda_tracker::DerateSource;
using panda_tracker::FilterUpdate;
using panda_tracker::PositionServo;
using panda_tracker::PositionTrackingConfig;
using panda_tracker::RuntimeSafetyMonitor;
using panda_tracker::StopDiagnostics;
using panda_tracker::StopReason;
using panda_tracker::TrackerPositionFilter;
using panda_tracker::TrackerReceiver;
using panda_tracker::Transform;
using panda_tracker::Vector3;
using panda_tracker::Wrench6;

constexpr double kPi = 3.14159265358979323846;
std::atomic_bool g_stop_requested{false};

void request_stop(int) {
  g_stop_requested.store(true, std::memory_order_relaxed);
}

double deg_to_rad(double value) {
  return value * kPi / 180.0;
}

double rad_to_deg(double value) {
  return value * 180.0 / kPi;
}

double seconds_between(
    Clock::time_point later,
    Clock::time_point earlier) {
  return std::chrono::duration<double>(later - earlier).count();
}


std::int64_t clock_time_to_ns(Clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             t.time_since_epoch())
      .count();
}

Clock::time_point clock_time_from_ns(std::int64_t ns) {
  return Clock::time_point(std::chrono::nanoseconds(ns));
}

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string tracker_bind_ip{"0.0.0.0"};
  std::string tracker_source_ip{};
  std::uint16_t tracker_port{5000};
  std::string config_path{};
  bool enable_motion{false};
  bool recover{false};
  bool preflight_only{false};
  bool apply_load_model{false};
};

std::uint16_t parse_port(const std::string& text) {
  const long value = std::stol(text);
  if (value < 1 || value > 65535) {
    throw std::invalid_argument("--tracker-port must be in [1,65535]");
  }
  return static_cast<std::uint16_t>(value);
}

void print_help(const char* argv0) {
  std::cout
      << "Position-only PBVS test controller (X/Y/Z selectable; orientation ignored).\n\n"
      << "Usage: " << argv0 << " [options]\n"
      << "  --robot-ip IP\n"
      << "  --tracker-bind-ip IP\n"
      << "  --tracker-source-ip IP\n"
      << "  --tracker-port PORT\n"
      << "  --config PATH\n"
      << "  --enable-motion       Required runtime motion enable\n"
      << "  --recover             Explicit automaticErrorRecovery before preflight\n"
      << "  --apply-load-model    Apply configured combined flange load\n"
      << "  --preflight-only      Validate robot/load/wrench and exit\n"
      << "  --help\n";
}

Options parse_options(int argc, char** argv) {
  Options o{};

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) {
      if (++i >= argc) {
        throw std::invalid_argument(std::string("Missing value for ") + name);
      }
      return std::string(argv[i]);
    };

    if (arg == "--robot-ip") o.robot_ip = next("--robot-ip");
    else if (arg == "--tracker-bind-ip") o.tracker_bind_ip = next("--tracker-bind-ip");
    else if (arg == "--tracker-source-ip") o.tracker_source_ip = next("--tracker-source-ip");
    else if (arg == "--tracker-port") o.tracker_port = parse_port(next("--tracker-port"));
    else if (arg == "--config") o.config_path = next("--config");
    else if (arg == "--enable-motion") o.enable_motion = true;
    else if (arg == "--recover") o.recover = true;
    else if (arg == "--preflight-only") o.preflight_only = true;
    else if (arg == "--apply-load-model") o.apply_load_model = true;
    else if (arg == "--help" || arg == "-h") {
      print_help(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }

  if (o.config_path.empty()) {
    throw std::invalid_argument("--config is required");
  }
  return o;
}

struct RobotSnapshot {
  Transform T_BF{};
  Clock::time_point arrival{};
  bool available{false};
};

struct ServoCommand {
  Vector3 velocity_B_mps{};
  Vector3 error_B_m{};
  Clock::time_point generated{};
  std::uint64_t tracker_sequence{0};
  double tracker_age_s{std::numeric_limits<double>::infinity()};
  bool valid{false};
  bool armed{false};
};

struct RtDiagnostics {
  Vector3 requested_after_safety_B_mps{};
  Vector3 limited_B_mps{};
  std::array<double, 3> last_commanded_B_mps{};
  std::array<double, 3> last_commanded_accel_B_mps2{};
  double safety_speed_scale{1.0};

  DerateSource derate_source{DerateSource::kNone};
  std::size_t derate_joint_index{0};
  double derate_value{0.0};
  double derate_limit{0.0};
  double derate_ratio{0.0};

  double control_command_success_rate{0.0};

  bool tracking_paused{false};
  double tracking_loss_elapsed_s{0.0};

  bool available{false};
};


class AtomicRobotSnapshot {
 public:
  AtomicRobotSnapshot() {
    for (auto& value : T_BF_) value.store(0.0, std::memory_order_relaxed);
  }

  void publish(const Transform& T_BF, Clock::time_point arrival) {
    // Odd sequence = write in progress, even = stable.
    sequence_.fetch_add(1, std::memory_order_acq_rel);
    for (std::size_t i = 0; i < T_BF.size(); ++i) {
      T_BF_[i].store(T_BF[i], std::memory_order_relaxed);
    }
    arrival_ns_.store(clock_time_to_ns(arrival), std::memory_order_relaxed);
    available_.store(true, std::memory_order_relaxed);
    sequence_.fetch_add(1, std::memory_order_release);
  }

  bool read(RobotSnapshot& out) const {
    for (int attempt = 0; attempt < 8; ++attempt) {
      const std::uint64_t before =
          sequence_.load(std::memory_order_acquire);
      if (before & 1U) continue;

      RobotSnapshot candidate{};
      for (std::size_t i = 0; i < candidate.T_BF.size(); ++i) {
        candidate.T_BF[i] = T_BF_[i].load(std::memory_order_relaxed);
      }
      candidate.arrival =
          clock_time_from_ns(arrival_ns_.load(std::memory_order_relaxed));
      candidate.available =
          available_.load(std::memory_order_relaxed);

      const std::uint64_t after =
          sequence_.load(std::memory_order_acquire);
      if (before == after && !(after & 1U)) {
        out = candidate;
        return candidate.available;
      }
    }
    return false;
  }

 private:
  mutable std::atomic<std::uint64_t> sequence_{0};
  std::array<std::atomic<double>, 16> T_BF_{};
  std::atomic<std::int64_t> arrival_ns_{0};
  std::atomic<bool> available_{false};
};

class AtomicServoCommand {
 public:
  AtomicServoCommand() {
    for (auto& value : velocity_) value.store(0.0, std::memory_order_relaxed);
    for (auto& value : error_) value.store(0.0, std::memory_order_relaxed);
  }

  void publish(const ServoCommand& command) {
    sequence_.fetch_add(1, std::memory_order_acq_rel);

    for (std::size_t i = 0; i < 3; ++i) {
      velocity_[i].store(
          command.velocity_B_mps[i], std::memory_order_relaxed);
      error_[i].store(
          command.error_B_m[i], std::memory_order_relaxed);
    }

    generated_ns_.store(
        clock_time_to_ns(command.generated), std::memory_order_relaxed);
    tracker_sequence_.store(
        command.tracker_sequence, std::memory_order_relaxed);
    tracker_age_s_.store(
        command.tracker_age_s, std::memory_order_relaxed);
    valid_.store(command.valid, std::memory_order_relaxed);
    armed_.store(command.armed, std::memory_order_relaxed);

    sequence_.fetch_add(1, std::memory_order_release);
  }

  bool read(ServoCommand& out) const {
    for (int attempt = 0; attempt < 8; ++attempt) {
      const std::uint64_t before =
          sequence_.load(std::memory_order_acquire);
      if (before & 1U) continue;

      ServoCommand candidate{};
      for (std::size_t i = 0; i < 3; ++i) {
        candidate.velocity_B_mps[i] =
            velocity_[i].load(std::memory_order_relaxed);
        candidate.error_B_m[i] =
            error_[i].load(std::memory_order_relaxed);
      }

      candidate.generated =
          clock_time_from_ns(
              generated_ns_.load(std::memory_order_relaxed));
      candidate.tracker_sequence =
          tracker_sequence_.load(std::memory_order_relaxed);
      candidate.tracker_age_s =
          tracker_age_s_.load(std::memory_order_relaxed);
      candidate.valid = valid_.load(std::memory_order_relaxed);
      candidate.armed = armed_.load(std::memory_order_relaxed);

      const std::uint64_t after =
          sequence_.load(std::memory_order_acquire);
      if (before == after && !(after & 1U)) {
        out = candidate;
        return true;
      }
    }
    return false;
  }

 private:
  mutable std::atomic<std::uint64_t> sequence_{0};
  std::array<std::atomic<double>, 3> velocity_{};
  std::array<std::atomic<double>, 3> error_{};
  std::atomic<std::int64_t> generated_ns_{0};
  std::atomic<std::uint64_t> tracker_sequence_{0};
  std::atomic<double> tracker_age_s_{
      std::numeric_limits<double>::infinity()};
  std::atomic<bool> valid_{false};
  std::atomic<bool> armed_{false};
};

std::string axis_text(const PositionTrackingConfig& config) {
  std::string text;
  if (config.control_axes.x) text += "X";
  if (config.control_axes.y) text += text.empty() ? "Y" : "+Y";
  if (config.control_axes.z) text += text.empty() ? "Z" : "+Z";
  return text;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);

    PositionTrackingConfig config{};
    std::string config_error;
    if (!panda_tracker::load_position_tracking_config(
            options.config_path, config, config_error)) {
      throw std::runtime_error("Unable to load config: " + config_error);
    }

    if (config.load.enabled && !options.apply_load_model) {
      throw std::runtime_error(
          "This config requires the flange load model; add --apply-load-model");
    }
    if (options.apply_load_model && !config.load.enabled) {
      throw std::runtime_error(
          "--apply-load-model supplied but load_model_enabled is false");
    }
    if (!options.preflight_only &&
        (!config.motion_enabled || !options.enable_motion)) {
      throw std::runtime_error(
          "Motion requires motion_enabled:true and --enable-motion");
    }
    if (!options.preflight_only &&
        config.require_tracker_source_filter &&
        options.tracker_source_ip.empty()) {
      throw std::runtime_error(
          "Active motion requires --tracker-source-ip");
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    const franka::RealtimeConfig realtime =
        config.realtime_enforced
            ? franka::RealtimeConfig::kEnforce
            : franka::RealtimeConfig::kIgnore;

    franka::Robot robot(options.robot_ip, realtime);

    if (options.recover) {
      robot.automaticErrorRecovery();
      std::cout << "Explicit robot error recovery complete.\n";
    }

    if (options.apply_load_model) {
      panda_tracker::apply_and_verify_load_model(robot, config);
    }

    const Wrench6 wrench_bias =
        panda_tracker::acquire_wrench_bias(robot, config);
    const franka::RobotState preflight_state = robot.readOnce();

    panda_tracker::print_preflight_measurements(
        preflight_state, wrench_bias);

    std::string preflight_error;
    if (!panda_tracker::preflight_ok(
            preflight_state, config, wrench_bias, preflight_error)) {
      throw std::runtime_error("Preflight rejected: " + preflight_error);
    }

    if (options.preflight_only) {
      std::cout
          << "PREFLIGHT PASSED: no control loop or motion command was started.\n";
      return EXIT_SUCCESS;
    }

    std::cout
        << "POSITION-ONLY PBVS ROBOT CONTROLLER\n"
        << "Robot: " << options.robot_ip << '\n'
        << "Tracker: " << options.tracker_bind_ip << ':'
        << options.tracker_port
        << " source=" << options.tracker_source_ip << '\n'
        << "Enabled translation axes: " << axis_text(config) << '\n'
        << "Diagnostic position bias B [mm]: ["
        << config.diagnostic_position_bias_B_m[0] * 1000.0 << ", "
        << config.diagnostic_position_bias_B_m[1] * 1000.0 << ", "
        << config.diagnostic_position_bias_B_m[2] * 1000.0 << "]\n"
        << "Orientation control: OFF (tracker PnP orientation ignored)\n"
        << "Worker rate: " << config.worker_rate_hz << " Hz\n"
        << "Filter: median=" << config.filter.median_window_samples
        << " samples, EMA alpha=" << config.filter.ema_alpha
        << ", post-filter jump="
        << config.filter.max_filtered_jump_m * 1000.0 << " mm\n"
        << "Tracker stale/grace: "
        << config.tracker_timeout_s * 1000.0 << " / "
        << config.tracking_loss_grace_s * 1000.0 << " ms\n"
        << "Max speed/acceleration/jerk: "
        << config.max_linear_speed_mps * 1000.0 << " mm/s, "
        << config.max_linear_acceleration_mps2 * 1000.0 << " mm/s^2, "
        << config.max_linear_jerk_mps3 * 1000.0 << " mm/s^3\n"
        << "Motion runtime: ";

    if (config.max_motion_runtime_s <= 0.0) {
      std::cout << "UNLIMITED (safety stops remain active)\n";
    } else {
      std::cout << config.max_motion_runtime_s << " s\n";
    }

    std::cout
        << "Realtime enforcement: "
        << (config.realtime_enforced ? "ON" : "OFF (diagnostic)") << '\n'
        << "Waiting for filtered tracker packets and robot state...\n";

    AtomicRobotSnapshot robot_mailbox;
    AtomicServoCommand command_mailbox;

    std::mutex rt_diag_mutex;
    RtDiagnostics shared_rt_diag{};

    std::mutex log_mutex;
    std::mutex failure_mutex;
    std::exception_ptr worker_failure;
    std::atomic_bool worker_failed{false};
    std::atomic_bool worker_running{true};

    PositionServo servo(config);
    TrackerPositionFilter filter(
        config.filter,
        servo.reference_camera_rotation_R_CT());

    std::thread worker([&]() {
      try {
        TrackerReceiver receiver(
            options.tracker_bind_ip,
            options.tracker_source_ip,
            options.tracker_port);

        const auto period =
            std::chrono::duration<double>(1.0 / config.worker_rate_hz);
        auto next = Clock::now();
        auto next_log = next;

        std::size_t arm_packets = 0;
        std::uint64_t last_counted_sequence = 0;
        bool have_counted_sequence = false;

        while (worker_running.load(std::memory_order_relaxed) &&
               !g_stop_requested.load(std::memory_order_relaxed)) {
          const auto loop_start = Clock::now();
          if (loop_start < next) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
          }

          // IMPORTANT: acquire all asynchronous data first, then timestamp the
          // evaluation. In the previous version `now` was captured before
          // receiver.poll() and before copying the 1-kHz robot snapshot. That
          // allowed arrival timestamps to be newer than `now`, producing small
          // negative ages and preventing the servo from ever becoming valid.
          receiver.poll();
          const auto tracker = receiver.latest();
          const FilterUpdate update = filter.process(tracker);
          const auto filtered = filter.output();

          RobotSnapshot robot_snapshot{};
          robot_mailbox.read(robot_snapshot);

          const auto now = Clock::now();

          ServoCommand command{};
          command.generated = now;

          const double robot_age_s =
              robot_snapshot.available
                  ? seconds_between(now, robot_snapshot.arrival)
                  : std::numeric_limits<double>::infinity();

          const double tracker_age_s =
              filtered.available
                  ? seconds_between(now, filtered.arrival)
                  : std::numeric_limits<double>::infinity();

          command.tracker_age_s = tracker_age_s;
          command.tracker_sequence = filtered.available ? filtered.sequence : 0;

          bool servo_safe = false;
          panda_tracker::PositionServoResult servo_result{};
          const char* gate = "WAIT";

          if (!robot_snapshot.available) {
            gate = "NO_ROBOT";
          } else if (robot_age_s < 0.0 ||
                     robot_age_s > config.robot_state_timeout_s) {
            gate = "ROBOT_STALE";
          } else if (!filtered.available) {
            gate = "FILTER_WARMING";
          } else if (tracker_age_s < 0.0 ||
                     tracker_age_s > config.tracker_timeout_s) {
            gate = "TRACKER_STALE";
          } else {
            servo_result = servo.compute(
                robot_snapshot.T_BF,
                filtered.T_CT);

            // Keep computed error visible in diagnostics even when it is too
            // large to arm. Command validity remains false until every gate
            // passes.
            if (servo_result.valid) {
              command.error_B_m = servo_result.active_error_B_m;
              command.velocity_B_mps = servo_result.velocity_B_mps;

              if (servo_result.active_error_norm_m >
                  config.max_position_error_m) {
                gate = "ERROR_LIMIT";
              } else {
                gate = "OK";
                servo_safe = true;
                command.valid = true;
              }
            } else {
              gate = "SERVO_INVALID";
            }
          }

          // Arming counts newly accepted FILTERED tracker packets, not worker
          // iterations. Therefore 10 means 10 camera updates passed every
          // servo gate.
          if (update == FilterUpdate::kAccepted &&
              filtered.available &&
              (!have_counted_sequence ||
               filtered.sequence != last_counted_sequence)) {
            last_counted_sequence = filtered.sequence;
            have_counted_sequence = true;
            arm_packets = servo_safe ? arm_packets + 1 : 0;
          }

          if (!servo_safe &&
              (!filtered.available ||
               tracker_age_s > config.tracker_timeout_s ||
               robot_age_s > config.robot_state_timeout_s)) {
            arm_packets = 0;
          }

          command.armed =
              command.valid &&
              arm_packets >= config.arm_valid_packets;

          command_mailbox.publish(command);

          if (now >= next_log) {
            RtDiagnostics rt_diag{};
            {
              std::lock_guard<std::mutex> diag_lock(rt_diag_mutex);
              rt_diag = shared_rt_diag;
            }

            std::lock_guard<std::mutex> lock(log_mutex);
            std::cout << std::fixed << std::setprecision(3)
                      << "filter="
                      << (filtered.available ? "READY" : "warming")
                      << " gate=" << gate
                      << " raw_window=" << filter.raw_window_size()
                      << " accepted=" << filter.accepted_packets()
                      << " rejected_jump=" << filter.rejected_jumps()
                      << " rejected_invalid=" << filter.rejected_invalid()
                      << " tracker_age_ms=" << tracker_age_s * 1000.0
                      << " robot_age_ms=" << robot_age_s * 1000.0
                      << " err_mm=["
                      << command.error_B_m[0] * 1000.0 << ','
                      << command.error_B_m[1] * 1000.0 << ','
                      << command.error_B_m[2] * 1000.0 << ']'
                      << " err_norm_mm="
                      << servo_result.active_error_norm_m * 1000.0
                      << " v_mmps=["
                      << command.velocity_B_mps[0] * 1000.0 << ','
                      << command.velocity_B_mps[1] * 1000.0 << ','
                      << command.velocity_B_mps[2] * 1000.0 << ']'
                      << " arm_packets=" << arm_packets
                      << " armed=" << (command.armed ? "YES" : "no");

            if (rt_diag.available) {
              std::cout
                  << " safety_scale=" << rt_diag.safety_speed_scale
                  << " derate="
                  << panda_tracker::derate_source_text(
                         rt_diag.derate_source);

              if (rt_diag.derate_source ==
                      DerateSource::kJointTorque ||
                  rt_diag.derate_source ==
                      DerateSource::kExternalJointTorque ||
                  rt_diag.derate_source ==
                      DerateSource::kJointTorqueRate) {
                std::cout
                    << "(J" << rt_diag.derate_joint_index + 1
                    << " value=" << rt_diag.derate_value
                    << " limit=" << rt_diag.derate_limit
                    << " ratio=" << rt_diag.derate_ratio << ')';
              } else if (rt_diag.derate_source !=
                         DerateSource::kNone) {
                std::cout
                    << "(value=" << rt_diag.derate_value
                    << " limit=" << rt_diag.derate_limit
                    << " ratio=" << rt_diag.derate_ratio << ')';
              }

              std::cout
                  << " rt_req_mmps=["
                  << rt_diag.requested_after_safety_B_mps[0] * 1000.0 << ','
                  << rt_diag.requested_after_safety_B_mps[1] * 1000.0 << ','
                  << rt_diag.requested_after_safety_B_mps[2] * 1000.0 << ']'
                  << " limited_mmps=["
                  << rt_diag.limited_B_mps[0] * 1000.0 << ','
                  << rt_diag.limited_B_mps[1] * 1000.0 << ','
                  << rt_diag.limited_B_mps[2] * 1000.0 << ']'
                  << " O_dP_EE_c_mmps=["
                  << rt_diag.last_commanded_B_mps[0] * 1000.0 << ','
                  << rt_diag.last_commanded_B_mps[1] * 1000.0 << ','
                  << rt_diag.last_commanded_B_mps[2] * 1000.0 << ']'
                  << " O_ddP_EE_c_mmps2=["
                  << rt_diag.last_commanded_accel_B_mps2[0] * 1000.0 << ','
                  << rt_diag.last_commanded_accel_B_mps2[1] * 1000.0 << ','
                  << rt_diag.last_commanded_accel_B_mps2[2] * 1000.0 << ']'
                  << " cmd_success=" << rt_diag.control_command_success_rate
                  << " tracking_pause="
                  << (rt_diag.tracking_paused ? "YES" : "no")
                  << " tracking_loss_ms="
                  << rt_diag.tracking_loss_elapsed_s * 1000.0;
            }
            std::cout << '\n';
            next_log = now + std::chrono::seconds(1);
          }

          next = loop_start +
              std::chrono::duration_cast<Clock::duration>(period);
        }
      } catch (...) {
        {
          std::lock_guard<std::mutex> lock(failure_mutex);
          worker_failure = std::current_exception();
        }
        worker_failed.store(true, std::memory_order_relaxed);
        g_stop_requested.store(true, std::memory_order_relaxed);
      }
    });

    StopReason stop_reason = StopReason::kNone;
    StopDiagnostics stop_diagnostics{};
    std::exception_ptr robot_failure;

    try {
      RuntimeSafetyMonitor safety_monitor(config, wrench_bias);

      bool ever_armed = false;
      bool stopping = false;
      double control_elapsed_s = 0.0;
      double motion_elapsed_s = 0.0;
      double stopping_elapsed_s = 0.0;
      double tracking_loss_elapsed_s = 0.0;

      ServoCommand rt_command_cache{};
      bool have_rt_command_cache = false;

      auto begin_stop = [&](StopReason reason) {
        if (!stopping) {
          stopping = true;
          stop_reason = reason;
          stopping_elapsed_s = 0.0;
        }
      };

      robot.control(
          [&](const franka::RobotState& state,
              franka::Duration period) -> franka::CartesianVelocities {
            const double dt = std::max(0.0, period.toSec());
            control_elapsed_s += dt;
            if (ever_armed && !stopping) motion_elapsed_s += dt;
            if (stopping) stopping_elapsed_s += dt;

            const auto now = Clock::now();
            const Transform T_BF =
                panda_tracker::physical_flange_pose(state);

            robot_mailbox.publish(T_BF, now);

            const auto safety =
                safety_monitor.update(state, T_BF, control_elapsed_s);

            // Keep the latest diagnostics while running. Once a stop starts,
            // preserve the diagnostics from the triggering instant instead of
            // overwriting joint/value information during the deceleration.
            if (!stopping) {
              stop_diagnostics = safety.diagnostics;
            }

            if (safety.reason != StopReason::kNone) {
              begin_stop(safety.reason);
            }

            if (g_stop_requested.load(std::memory_order_relaxed)) {
              begin_stop(
                  worker_failed.load(std::memory_order_relaxed)
                      ? StopReason::kWorkerFailure
                      : StopReason::kSignal);
            }

            if (!ever_armed &&
                control_elapsed_s >= config.max_arm_wait_s) {
              begin_stop(StopReason::kArmTimeout);
            }

            // max_motion_runtime_s <= 0 explicitly means unlimited runtime.
            // Travel, tracking-loss, torque, wrench, contact, communication,
            // joint-speed, signal and worker-failure stops remain active.
            if (ever_armed &&
                config.max_motion_runtime_s > 0.0 &&
                motion_elapsed_s >= config.max_motion_runtime_s) {
              begin_stop(StopReason::kRuntime);
            }

            // The worker owns the tracker/filter/servo at 100 Hz. The
            // command mailbox is lock-free for the RT path, so a preempted
            // non-RT worker can no longer hold a mutex that blocks command
            // exchange or robot-state publication.
            ServoCommand newest_command{};
            if (command_mailbox.read(newest_command)) {
              rt_command_cache = newest_command;
              have_rt_command_cache = true;
            }

            const ServoCommand& command = rt_command_cache;
            const bool got_command = have_rt_command_cache;

            Vector3 desired_velocity{};
            bool tracking_paused = false;

            if (!stopping &&
                got_command &&
                command.valid &&
                command.armed) {
              if (seconds_between(now, command.generated) >
                  config.command_timeout_s) {
                begin_stop(StopReason::kCommandStale);
              } else {
                ever_armed = true;
                tracking_loss_elapsed_s = 0.0;

                for (std::size_t i = 0; i < 3; ++i) {
                  desired_velocity[i] =
                      command.velocity_B_mps[i] * safety.speed_scale;
                }

                if (panda_tracker::moving_toward_soft_travel_limit(
                        desired_velocity,
                        safety.diagnostics.travel_B_m,
                        config)) {
                  desired_velocity = Vector3{};
                  begin_stop(StopReason::kBrakingBoundary);
                }
              }
            } else if (!stopping &&
                       ever_armed &&
                       config.stop_on_tracking_loss &&
                       got_command &&
                       (!command.valid || !command.armed)) {
              // A short tracker dropout is not a reason to continue blindly
              // with stale target motion. Immediately request zero velocity
              // and decelerate. If fresh packets return and pass the normal
              // re-arming gate before the grace interval expires, tracking
              // resumes. Only a persistent outage becomes a terminal stop.
              tracking_paused = true;
              tracking_loss_elapsed_s += dt;

              if (tracking_loss_elapsed_s >=
                  config.tracking_loss_grace_s) {
                begin_stop(StopReason::kTrackingLost);
              }
            }

            std::array<double, 6> desired_twist{{
                desired_velocity[0],
                desired_velocity[1],
                desired_velocity[2],
                0.0, 0.0, 0.0,
            }};

            if (stopping) desired_twist.fill(0.0);

            // Explicit conservative translational speed/acceleration/jerk
            // limits. robot.control() below disables libfranka's second rate
            // limiter and command low-pass filter so this is the one explicit
            // command-shaping stage.
            const std::array<double, 6> limited_twist =
                franka::limitRate(
                    config.max_linear_speed_mps,
                    config.max_linear_acceleration_mps2,
                    config.max_linear_jerk_mps3,
                    deg_to_rad(1.0),
                    deg_to_rad(5.0),
                    deg_to_rad(50.0),
                    desired_twist,
                    state.O_dP_EE_c,
                    state.O_ddP_EE_c);

            // Publish diagnostics without ever blocking the 1-kHz callback.
            if (rt_diag_mutex.try_lock()) {
              shared_rt_diag.requested_after_safety_B_mps = {{
                  desired_twist[0], desired_twist[1], desired_twist[2]}};
              shared_rt_diag.limited_B_mps = {{
                  limited_twist[0], limited_twist[1], limited_twist[2]}};
              shared_rt_diag.last_commanded_B_mps = {{
                  state.O_dP_EE_c[0],
                  state.O_dP_EE_c[1],
                  state.O_dP_EE_c[2]}};
              shared_rt_diag.last_commanded_accel_B_mps2 = {{
                  state.O_ddP_EE_c[0],
                  state.O_ddP_EE_c[1],
                  state.O_ddP_EE_c[2]}};
              shared_rt_diag.safety_speed_scale = safety.speed_scale;
              shared_rt_diag.derate_source = safety.derate_source;
              shared_rt_diag.derate_joint_index = safety.derate_joint_index;
              shared_rt_diag.derate_value = safety.derate_value;
              shared_rt_diag.derate_limit = safety.derate_limit;
              shared_rt_diag.derate_ratio = safety.derate_ratio;
              shared_rt_diag.control_command_success_rate =
                  state.control_command_success_rate;
              shared_rt_diag.tracking_paused = tracking_paused;
              shared_rt_diag.tracking_loss_elapsed_s =
                  tracking_loss_elapsed_s;
              shared_rt_diag.available = true;
              rt_diag_mutex.unlock();
            }

            if (stopping) {
              bool limited_velocity_zero = true;
              bool panda_commanded_velocity_zero = true;

              // Stop completion is a velocity condition. We require both the
              // velocity produced by our explicit limiter and libfranka's
              // last commanded Cartesian velocity to be essentially zero.
              //
              // O_ddP_EE_c is deliberately NOT used as a completion gate:
              // it is a commanded acceleration and can still be relatively
              // large during the final jerk-limited approach to zero even
              // when velocity is already negligible.
              for (std::size_t i = 0; i < 3; ++i) {
                limited_velocity_zero =
                    limited_velocity_zero &&
                    std::abs(limited_twist[i]) <=
                        config.stop_velocity_epsilon_mps;
                panda_commanded_velocity_zero =
                    panda_commanded_velocity_zero &&
                    std::abs(state.O_dP_EE_c[i]) <=
                        config.stop_velocity_epsilon_mps;
              }

              if (limited_velocity_zero &&
                  panda_commanded_velocity_zero) {
                return franka::MotionFinished(
                    franka::CartesianVelocities(limited_twist));
              }

              if (stopping_elapsed_s >= config.max_stop_time_s) {
                stop_diagnostics.timeout_elapsed_s =
                    stopping_elapsed_s;
                for (std::size_t i = 0; i < 3; ++i) {
                  stop_diagnostics.timeout_limited_velocity_B_mps[i] =
                      limited_twist[i];
                  stop_diagnostics
                      .timeout_last_commanded_velocity_B_mps[i] =
                      state.O_dP_EE_c[i];
                  stop_diagnostics
                      .timeout_last_commanded_acceleration_B_mps2[i] =
                      state.O_ddP_EE_c[i];
                }
                stop_diagnostics.forced_finish_after_stop_timeout = true;

                std::array<double, 6> zero{};
                return franka::MotionFinished(
                    franka::CartesianVelocities(zero));
              }
            }

            return franka::CartesianVelocities(limited_twist);
          },
          franka::ControllerMode::kCartesianImpedance,
          false,
          franka::kMaxCutoffFrequency);
    } catch (...) {
      robot_failure = std::current_exception();
    }

    worker_running.store(false, std::memory_order_relaxed);
    g_stop_requested.store(true, std::memory_order_relaxed);
    if (worker.joinable()) worker.join();

    if (stop_reason != StopReason::kNone) {
      std::cerr << std::fixed << std::setprecision(4)
                << "STOP: " << panda_tracker::stop_reason_text(stop_reason)
                << " travel_mm=["
                << stop_diagnostics.travel_B_m[0] * 1000.0 << ','
                << stop_diagnostics.travel_B_m[1] * 1000.0 << ','
                << stop_diagnostics.travel_B_m[2] * 1000.0 << ']'
                << " rotation_deg="
                << rad_to_deg(stop_diagnostics.rotation_rad)
                << " external_force_N="
                << stop_diagnostics.external_force_n
                << " external_torque_Nm="
                << stop_diagnostics.external_torque_nm
                << " forced_finish="
                << (stop_diagnostics.forced_finish_after_stop_timeout
                        ? "YES" : "no");

      if (stop_reason == StopReason::kJointSpeed ||
          stop_reason == StopReason::kJointTorque ||
          stop_reason == StopReason::kExternalJointTorque ||
          stop_reason == StopReason::kJointTorqueRate) {
        std::cerr << " joint="
                  << stop_diagnostics.joint_index + 1
                  << " value=" << stop_diagnostics.joint_value;
      }
      std::cerr << '\n';

      if (stop_diagnostics.forced_finish_after_stop_timeout) {
        std::cerr << std::fixed << std::setprecision(4)
                  << "STOP TIMEOUT:"
                  << " elapsed_s="
                  << stop_diagnostics.timeout_elapsed_s
                  << " limited_mmps=["
                  << stop_diagnostics
                         .timeout_limited_velocity_B_mps[0] * 1000.0
                  << ','
                  << stop_diagnostics
                         .timeout_limited_velocity_B_mps[1] * 1000.0
                  << ','
                  << stop_diagnostics
                         .timeout_limited_velocity_B_mps[2] * 1000.0
                  << ']'
                  << " O_dP_EE_c_mmps=["
                  << stop_diagnostics
                         .timeout_last_commanded_velocity_B_mps[0] * 1000.0
                  << ','
                  << stop_diagnostics
                         .timeout_last_commanded_velocity_B_mps[1] * 1000.0
                  << ','
                  << stop_diagnostics
                         .timeout_last_commanded_velocity_B_mps[2] * 1000.0
                  << ']'
                  << " O_ddP_EE_c_mmps2=["
                  << stop_diagnostics
                         .timeout_last_commanded_acceleration_B_mps2[0] * 1000.0
                  << ','
                  << stop_diagnostics
                         .timeout_last_commanded_acceleration_B_mps2[1] * 1000.0
                  << ','
                  << stop_diagnostics
                         .timeout_last_commanded_acceleration_B_mps2[2] * 1000.0
                  << ']'
                  << '\n';
      }
    }

    if (robot_failure) std::rethrow_exception(robot_failure);

    {
      std::lock_guard<std::mutex> lock(failure_mutex);
      if (worker_failure) std::rethrow_exception(worker_failure);
    }

    std::cout << "Robot control loop ended after a rate-limited stop.\n";
    return EXIT_SUCCESS;

  } catch (const franka::Exception& e) {
    std::cerr << "Franka error: " << e.what() << '\n';
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
  }

  return EXIT_FAILURE;
}
