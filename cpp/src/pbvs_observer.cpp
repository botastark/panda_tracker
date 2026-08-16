#include "panda_tracker/pbvs.h"
#include "panda_tracker/task_pose_protocol.h"

#include <franka/exception.h>
#include <franka/robot.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
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

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) {
  stop_requested = 1;
}

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string tracker_bind_ip{"127.0.0.1"};
  std::string tracker_source_ip{};
  std::uint16_t tracker_port{6501};
  double sample_rate_hz{20.0};
  bool sample_rate_explicit{false};
  double duration_s{30.0};
  double tracker_timeout_s{0.1};
  bool tracker_timeout_explicit{false};
  float minimum_confidence{0.5F};
  std::string csv_path{};
  std::string pbvs_config_path{};
};

void print_help(const char* program) {
  std::cout
      << "Read-only Panda and PTP2 tracker observer.\n\n"
      << "This program never starts a robot control loop and never sends a "
         "motion command.\n\n"
      << "Usage: " << program << " [options]\n"
      << "  --robot-ip IP              FCI address (default 172.16.0.2)\n"
      << "  --tracker-bind-ip IP       Local PTP2 bind address "
         "(default 127.0.0.1)\n"
      << "  --tracker-source-ip IP     Accept PTP2 only from this source\n"
      << "  --tracker-port PORT        PTP2 UDP port (default 6501)\n"
      << "  --sample-rate HZ           CSV/console sample rate "
         "(default 20)\n"
      << "  --duration SECONDS         Stop automatically; 0 means no limit "
         "(default 30)\n"
      << "  --tracker-timeout SECONDS  Freshness threshold "
         "(default 0.1)\n"
      << "  --minimum-confidence VALUE Accepted range [0,1] "
         "(default 0.5)\n"
      << "  --csv PATH                 Write measurements to CSV\n"
      << "  --pbvs-config PATH         Enable compute-only PBVS logging\n"
      << "  --help, -h                 Show this message\n";
}

double parse_double(const std::string& text, const char* option) {
  std::size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(std::string(option) + " requires a number.");
  }
  return value;
}

std::uint16_t parse_port(const std::string& text) {
  std::size_t parsed = 0;
  const unsigned long value = std::stoul(text, &parsed);
  if (parsed != text.size() || value == 0 || value > 65535) {
    throw std::invalid_argument("--tracker-port must be in [1, 65535].");
  }
  return static_cast<std::uint16_t>(value);
}

Options parse_options(int argc, char** argv) {
  Options options{};

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];

    auto next_value = [&](const char* option) -> std::string {
      if (index + 1 >= argc) {
        throw std::invalid_argument(std::string(option) + " requires a value.");
      }
      return argv[++index];
    };

    if (argument == "--robot-ip") {
      options.robot_ip = next_value("--robot-ip");
    } else if (argument == "--tracker-bind-ip") {
      options.tracker_bind_ip = next_value("--tracker-bind-ip");
    } else if (argument == "--tracker-source-ip") {
      options.tracker_source_ip = next_value("--tracker-source-ip");
    } else if (argument == "--tracker-port") {
      options.tracker_port = parse_port(next_value("--tracker-port"));
    } else if (argument == "--sample-rate") {
      options.sample_rate_hz =
          parse_double(next_value("--sample-rate"), "--sample-rate");
      options.sample_rate_explicit = true;
    } else if (argument == "--duration") {
      options.duration_s =
          parse_double(next_value("--duration"), "--duration");
    } else if (argument == "--tracker-timeout") {
      options.tracker_timeout_s = parse_double(
          next_value("--tracker-timeout"), "--tracker-timeout");
      options.tracker_timeout_explicit = true;
    } else if (argument == "--minimum-confidence") {
      options.minimum_confidence = static_cast<float>(parse_double(
          next_value("--minimum-confidence"), "--minimum-confidence"));
    } else if (argument == "--csv") {
      options.csv_path = next_value("--csv");
    } else if (argument == "--pbvs-config") {
      options.pbvs_config_path = next_value("--pbvs-config");
    } else if (argument == "--help" || argument == "-h") {
      print_help(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("Unsupported option: " + argument);
    }
  }

  if (!(options.sample_rate_hz > 0.0) || options.sample_rate_hz > 1000.0) {
    throw std::invalid_argument("--sample-rate must be in (0, 1000].");
  }
  if (options.duration_s < 0.0) {
    throw std::invalid_argument("--duration must be non-negative.");
  }
  if (!(options.tracker_timeout_s > 0.0)) {
    throw std::invalid_argument("--tracker-timeout must be positive.");
  }
  if (options.minimum_confidence < 0.0F ||
      options.minimum_confidence > 1.0F) {
    throw std::invalid_argument("--minimum-confidence must be in [0, 1].");
  }

  return options;
}

struct RobotSnapshot {
  std::array<double, 16> O_T_EE{};
  Clock::time_point arrival{};
  std::uint64_t sequence{0};
  bool available{false};
};

class SharedRobotState {
 public:
  void update(const franka::RobotState& state) {
    if (!mutex_.try_lock()) {
      return;
    }
    snapshot_.O_T_EE = state.O_T_EE;
    snapshot_.arrival = Clock::now();
    ++snapshot_.sequence;
    snapshot_.available = true;
    mutex_.unlock();
  }

  RobotSnapshot get() {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  std::mutex mutex_;
  RobotSnapshot snapshot_{};
};

struct TrackerSnapshot {
  panda_tracker::TaskPosePacket packet{};
  Clock::time_point arrival{};
  std::string source_ip{};
  std::uint16_t source_port{0};
  bool available{false};
};

class TrackerReceiver {
 public:
  explicit TrackerReceiver(const Options& options)
      : expected_source_ip_(options.tracker_source_ip) {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error(
          std::string("Unable to create tracker socket: ") +
          std::strerror(errno));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options.tracker_port);
    if (inet_pton(
            AF_INET,
            options.tracker_bind_ip.c_str(),
            &address.sin_addr) != 1) {
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::invalid_argument(
          "Invalid --tracker-bind-ip: " + options.tracker_bind_ip);
    }

    if (bind(
            socket_fd_,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)) < 0) {
      const std::string message = std::strerror(errno);
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error("Unable to bind tracker socket: " + message);
    }

    const int current_flags = fcntl(socket_fd_, F_GETFL, 0);
    if (current_flags < 0 ||
        fcntl(socket_fd_, F_SETFL, current_flags | O_NONBLOCK) < 0) {
      const std::string message = std::strerror(errno);
      close(socket_fd_);
      socket_fd_ = -1;
      throw std::runtime_error(
          "Unable to make tracker socket non-blocking: " + message);
    }
  }

  TrackerReceiver(const TrackerReceiver&) = delete;
  TrackerReceiver& operator=(const TrackerReceiver&) = delete;

  ~TrackerReceiver() {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

  void poll() {
    while (true) {
      std::array<std::uint8_t, 2048> buffer{};
      sockaddr_in source{};
      socklen_t source_size = sizeof(source);
      const ssize_t received = recvfrom(
          socket_fd_,
          buffer.data(),
          buffer.size(),
          0,
          reinterpret_cast<sockaddr*>(&source),
          &source_size);

      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error(
            std::string("Tracker recvfrom failed: ") + std::strerror(errno));
      }

      char source_text[INET_ADDRSTRLEN]{};
      if (inet_ntop(
              AF_INET,
              &source.sin_addr,
              source_text,
              sizeof(source_text)) == nullptr) {
        ++rejected_packets_;
        continue;
      }

      const std::string source_ip = source_text;
      if (!expected_source_ip_.empty() &&
          source_ip != expected_source_ip_) {
        ++wrong_source_packets_;
        continue;
      }

      panda_tracker::TaskPosePacket packet{};
      const auto status = panda_tracker::decode_task_pose(
          buffer.data(), static_cast<std::size_t>(received), packet);
      if (status != panda_tracker::DecodeStatus::kOk) {
        ++rejected_packets_;
        last_decode_error_ = status;
        continue;
      }

      if (snapshot_.available &&
          packet.sequence_id <= snapshot_.packet.sequence_id) {
        ++duplicate_or_old_packets_;
        continue;
      }

      snapshot_.packet = packet;
      snapshot_.arrival = Clock::now();
      snapshot_.source_ip = source_ip;
      snapshot_.source_port = ntohs(source.sin_port);
      snapshot_.available = true;
      ++accepted_packets_;
    }
  }

  TrackerSnapshot latest() const {
    return snapshot_;
  }

  std::uint64_t accepted_packets() const { return accepted_packets_; }
  std::uint64_t rejected_packets() const { return rejected_packets_; }
  std::uint64_t duplicate_or_old_packets() const {
    return duplicate_or_old_packets_;
  }
  std::uint64_t wrong_source_packets() const {
    return wrong_source_packets_;
  }
  panda_tracker::DecodeStatus last_decode_error() const {
    return last_decode_error_;
  }

 private:
  int socket_fd_{-1};
  std::string expected_source_ip_;
  TrackerSnapshot snapshot_{};
  std::uint64_t accepted_packets_{0};
  std::uint64_t rejected_packets_{0};
  std::uint64_t duplicate_or_old_packets_{0};
  std::uint64_t wrong_source_packets_{0};
  panda_tracker::DecodeStatus last_decode_error_{
      panda_tracker::DecodeStatus::kOk};
};

std::array<double, 6> pose_from_franka_transform(
    const std::array<double, 16>& transform) {
  // libfranka stores O_T_EE column-major. RPY uses Rz(yaw) Ry(pitch) Rx(roll).
  return {{
      transform[12],
      transform[13],
      transform[14],
      std::atan2(transform[6], transform[10]),
      std::atan2(-transform[2], std::hypot(transform[0], transform[1])),
      std::atan2(transform[1], transform[0]),
  }};
}

double seconds_between(Clock::time_point later, Clock::time_point earlier) {
  return std::chrono::duration<double>(later - earlier).count();
}

enum class TrackerHealth {
  kMissing,
  kStale,
  kPublisherInvalid,
  kLowConfidence,
  kValid,
};

const char* tracker_health_text(TrackerHealth health) {
  switch (health) {
    case TrackerHealth::kMissing:
      return "MISSING";
    case TrackerHealth::kStale:
      return "STALE";
    case TrackerHealth::kPublisherInvalid:
      return "INVALID";
    case TrackerHealth::kLowConfidence:
      return "LOW_CONFIDENCE";
    case TrackerHealth::kValid:
      return "VALID";
  }
  return "UNKNOWN";
}

TrackerHealth tracker_health(
    const TrackerSnapshot& tracker,
    Clock::time_point now,
    const Options& options) {
  if (!tracker.available) {
    return TrackerHealth::kMissing;
  }
  if (seconds_between(now, tracker.arrival) > options.tracker_timeout_s) {
    return TrackerHealth::kStale;
  }
  if (!tracker.packet.valid) {
    return TrackerHealth::kPublisherInvalid;
  }
  if (tracker.packet.confidence < options.minimum_confidence) {
    return TrackerHealth::kLowConfidence;
  }
  return TrackerHealth::kValid;
}

class CsvLogger {
 public:
  CsvLogger(const std::string& path, bool log_pbvs)
      : log_pbvs_(log_pbvs) {
    if (path.empty()) {
      return;
    }
    stream_.open(path);
    if (!stream_) {
      throw std::runtime_error("Unable to open CSV: " + path);
    }
    stream_
        << "elapsed_s,robot_sequence,robot_age_s,"
        << "O_T_EE_x_m,O_T_EE_y_m,O_T_EE_z_m,"
        << "O_T_EE_roll_rad,O_T_EE_pitch_rad,O_T_EE_yaw_rad,"
        << "tracker_health,tracker_age_s,tracker_sequence,"
        << "tracker_confidence,tracker_valid,tracker_source";
    for (std::size_t index = 0; index < 16; ++index) {
      stream_ << ",T_TS_" << index;
    }
    if (log_pbvs_) {
      stream_
          << ",pbvs_state,pbvs_reason"
          << ",p_error_base_x_m,p_error_base_y_m,p_error_base_z_m"
          << ",p_error_norm_m"
          << ",r_error_body_x_rad,r_error_body_y_rad,r_error_body_z_rad"
          << ",r_error_norm_rad"
          << ",proposed_v_base_x_mps,proposed_v_base_y_mps,"
             "proposed_v_base_z_mps"
          << ",proposed_w_body_x_radps,proposed_w_body_y_radps,"
             "proposed_w_body_z_radps"
          << ",target_linear_speed_mps,target_angular_speed_radps"
          << ",proposed_linear_speed_mps,proposed_angular_speed_radps"
          << ",proposed_command_lead_m"
          << ",proposed_O_T_EE_x_m,proposed_O_T_EE_y_m,"
             "proposed_O_T_EE_z_m";
    }
    stream_ << '\n';
    stream_ << std::setprecision(12);
  }

  void write(
      double elapsed_s,
      const RobotSnapshot& robot,
      double robot_age_s,
      const TrackerSnapshot& tracker,
      double tracker_age_s,
      TrackerHealth health,
      const std::optional<panda_tracker::PbvsResult>& pbvs_result) {
    if (!stream_) {
      return;
    }

    stream_ << elapsed_s << ',';
    if (robot.available) {
      const auto pose = pose_from_franka_transform(robot.O_T_EE);
      stream_ << robot.sequence << ',' << robot_age_s;
      for (const double value : pose) {
        stream_ << ',' << value;
      }
    } else {
      stream_ << ",,,,,,,";
    }

    stream_ << ',' << tracker_health_text(health) << ',';
    if (tracker.available) {
      stream_ << tracker_age_s << ','
              << tracker.packet.sequence_id << ','
              << tracker.packet.confidence << ','
              << (tracker.packet.valid ? 1 : 0) << ','
              << tracker.source_ip << ':' << tracker.source_port;
      for (const double value : tracker.packet.T_TS) {
        stream_ << ',' << value;
      }
    } else {
      stream_ << ",,,,";
      for (std::size_t index = 0; index < 16; ++index) {
        stream_ << ',';
      }
    }

    if (log_pbvs_) {
      if (pbvs_result) {
        const auto& result = *pbvs_result;
        stream_ << ',' << panda_tracker::pbvs_state_text(result.state)
                << ',' << result.reason;
        for (const double value : result.position_error_base_m) {
          stream_ << ',' << value;
        }
        stream_ << ',' << result.position_error_norm_m;
        for (const double value : result.orientation_error_body_rad) {
          stream_ << ',' << value;
        }
        stream_ << ',' << result.orientation_error_norm_rad;
        for (const double value :
             result.proposed_linear_velocity_base_mps) {
          stream_ << ',' << value;
        }
        for (const double value :
             result.proposed_angular_velocity_body_radps) {
          stream_ << ',' << value;
        }
        stream_ << ',' << result.target_linear_speed_mps
                << ',' << result.target_angular_speed_radps
                << ',' << result.proposed_linear_speed_mps
                << ',' << result.proposed_angular_speed_radps
                << ',' << result.proposed_command_lead_m;
        if (result.has_proposed_pose) {
          const auto translation =
              panda_tracker::transform_translation(result.proposed_T_BE);
          for (const double value : translation) {
            stream_ << ',' << value;
          }
        } else {
          stream_ << ",,,";
        }
      } else {
        for (std::size_t index = 0; index < 24; ++index) {
          stream_ << ',';
        }
      }
    }
    stream_ << '\n';
  }

 private:
  std::ofstream stream_;
  bool log_pbvs_{false};
};

}  // namespace

int main(int argc, char** argv) {
  try {
    Options options = parse_options(argc, argv);

    std::optional<panda_tracker::PbvsConfig> pbvs_config;
    std::optional<panda_tracker::PbvsController> pbvs_controller;
    if (!options.pbvs_config_path.empty()) {
      panda_tracker::PbvsConfig loaded{};
      std::string config_error;
      if (!panda_tracker::load_pbvs_config(
              options.pbvs_config_path, loaded, config_error)) {
        throw std::runtime_error("Unable to load PBVS config: " + config_error);
      }
      if (options.sample_rate_explicit &&
          std::abs(options.sample_rate_hz - loaded.control_rate_hz) > 1e-9) {
        throw std::invalid_argument(
            "--sample-rate must match control_rate_hz in --pbvs-config.");
      }
      if (options.tracker_timeout_explicit &&
          std::abs(options.tracker_timeout_s - loaded.tracker_timeout) > 1e-9) {
        throw std::invalid_argument(
            "--tracker-timeout must match tracker_timeout in --pbvs-config.");
      }
      options.sample_rate_hz = loaded.control_rate_hz;
      options.tracker_timeout_s = loaded.tracker_timeout;
      pbvs_config = loaded;
      pbvs_controller.emplace(loaded);
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    std::cout
        << (pbvs_controller
                ? "READ-ONLY PBVS COMPUTE OBSERVER\n"
                : "READ-ONLY HARDWARE OBSERVER\n")
        << "No robot control mode is started. No motion command can be sent.\n"
        << "Robot: " << options.robot_ip << '\n'
        << "PTP2: " << options.tracker_bind_ip << ':'
        << options.tracker_port << '\n';
    if (pbvs_config) {
      std::cout
          << "PBVS config: " << options.pbvs_config_path << '\n'
          << "PBVS rate_hz: " << pbvs_config->control_rate_hz << '\n'
          << "Tool geometry status: "
          << pbvs_config->tool_geometry_status << '\n'
          << "COMPUTE ONLY: proposed poses and velocities are logged but "
             "never sent.\n";
    }
    if (!options.tracker_source_ip.empty()) {
      std::cout << "Accepted tracker source: "
                << options.tracker_source_ip << '\n';
    }
    if (options.tracker_bind_ip == "0.0.0.0" &&
        options.tracker_source_ip.empty()) {
      std::cout
          << "Warning: tracker input is open on all interfaces and no source "
             "filter is set.\n";
    }

    TrackerReceiver tracker_receiver(options);
    CsvLogger csv(options.csv_path, pbvs_controller.has_value());
    SharedRobotState robot_state;

    std::atomic_bool reader_running{true};
    std::mutex failure_mutex;
    std::exception_ptr reader_failure;

    std::thread robot_reader([&]() {
      try {
        franka::Robot robot(options.robot_ip);
        robot.read([&](const franka::RobotState& state) {
          robot_state.update(state);
          return reader_running.load(std::memory_order_relaxed) &&
                 stop_requested == 0;
        });
      } catch (...) {
        {
          std::lock_guard<std::mutex> lock(failure_mutex);
          reader_failure = std::current_exception();
        }
        reader_running.store(false, std::memory_order_relaxed);
      }
    });

    const auto start = Clock::now();
    auto next_sample = start;
    auto next_console = start;
    const auto sample_period = std::chrono::duration<double>(
        1.0 / options.sample_rate_hz);
    auto last_pbvs_sample = start -
        std::chrono::duration_cast<Clock::duration>(sample_period);

    bool observed_robot_state = false;
    bool observed_valid_tracker = false;
    bool observed_pbvs_tracking = false;

    while (reader_running.load(std::memory_order_relaxed) &&
           stop_requested == 0) {
      tracker_receiver.poll();

      const auto loop_time = Clock::now();
      if (options.duration_s > 0.0 &&
          seconds_between(loop_time, start) >= options.duration_s) {
        break;
      }

      if (loop_time >= next_sample) {
        const RobotSnapshot robot = robot_state.get();
        const TrackerSnapshot tracker = tracker_receiver.latest();
        const auto sample_time = Clock::now();
        const double robot_age_s = robot.available
            ? seconds_between(sample_time, robot.arrival)
            : std::numeric_limits<double>::infinity();
        const double tracker_age_s = tracker.available
            ? seconds_between(sample_time, tracker.arrival)
            : std::numeric_limits<double>::infinity();
        const TrackerHealth health =
            tracker_health(tracker, sample_time, options);

        std::optional<panda_tracker::PbvsResult> pbvs_result;
        if (pbvs_controller) {
          std::optional<panda_tracker::Transform> T_BE;
          if (robot.available) {
            T_BE = panda_tracker::franka_column_major_transform(
                robot.O_T_EE);
          }

          std::optional<panda_tracker::TaskPoseMeasurement> measurement;
          if (tracker.available) {
            panda_tracker::TaskPoseMeasurement converted{};
            converted.T_TS = tracker.packet.T_TS;
            converted.timestamp_s = std::chrono::duration<double>(
                tracker.arrival.time_since_epoch()).count();
            converted.valid = tracker.packet.valid &&
                              tracker.packet.confidence >=
                                  options.minimum_confidence;
            converted.sequence_id = tracker.packet.sequence_id;
            measurement = converted;
          }

          const double now_s = std::chrono::duration<double>(
              sample_time.time_since_epoch()).count();
          double dt_s = seconds_between(sample_time, last_pbvs_sample);
          if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
            dt_s = 1.0 / options.sample_rate_hz;
          }
          last_pbvs_sample = sample_time;
          pbvs_result = pbvs_controller->step(
              T_BE,
              robot_age_s,
              measurement,
              now_s,
              dt_s);
          observed_pbvs_tracking = observed_pbvs_tracking ||
              pbvs_result->state == panda_tracker::PbvsState::kTracking;
        }

        observed_robot_state = observed_robot_state || robot.available;
        observed_valid_tracker = observed_valid_tracker ||
                                 health == TrackerHealth::kValid;

        csv.write(
            seconds_between(sample_time, start),
            robot,
            robot_age_s,
            tracker,
            tracker_age_s,
            health,
            pbvs_result);

        if (sample_time >= next_console) {
          std::cout << std::fixed << std::setprecision(4);
          if (robot.available) {
            const auto pose = pose_from_franka_transform(robot.O_T_EE);
            constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
            std::cout
                << "robot xyz_m=[" << pose[0] << ' ' << pose[1] << ' '
                << pose[2] << "] rpy_deg=["
                << pose[3] * kRadToDeg << ' '
                << pose[4] * kRadToDeg << ' '
                << pose[5] * kRadToDeg << "] age_ms="
                << robot_age_s * 1000.0;
          } else {
            std::cout << "robot MISSING";
          }

          std::cout << " | tracker=" << tracker_health_text(health);
          if (tracker.available) {
            std::cout
                << " seq=" << tracker.packet.sequence_id
                << " confidence=" << tracker.packet.confidence
                << " age_ms=" << tracker_age_s * 1000.0
                << " p_TS_m=["
                << tracker.packet.T_TS[3] << ' '
                << tracker.packet.T_TS[7] << ' '
                << tracker.packet.T_TS[11] << ']';
          }
          if (pbvs_result) {
            constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
            std::cout
                << " | pbvs="
                << panda_tracker::pbvs_state_text(pbvs_result->state)
                << " p_err_mm="
                << pbvs_result->position_error_norm_m * 1000.0
                << " r_err_deg="
                << pbvs_result->orientation_error_norm_rad * kRadToDeg
                << " proposed_v_mps="
                << pbvs_result->proposed_linear_speed_mps
                << " proposed_w_degps="
                << pbvs_result->proposed_angular_speed_radps * kRadToDeg;
            if (!pbvs_result->reason.empty()) {
              std::cout << " reason=" << pbvs_result->reason;
            }
          }
          std::cout << '\n';
          next_console = sample_time + std::chrono::seconds(1);
        }

        next_sample = sample_time +
                      std::chrono::duration_cast<Clock::duration>(
                          sample_period);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    reader_running.store(false, std::memory_order_relaxed);
    robot_reader.join();

    {
      std::lock_guard<std::mutex> lock(failure_mutex);
      if (reader_failure) {
        std::rethrow_exception(reader_failure);
      }
    }

    std::cout
        << "PTP2 counters: accepted=" << tracker_receiver.accepted_packets()
        << " rejected=" << tracker_receiver.rejected_packets()
        << " duplicate_or_old="
        << tracker_receiver.duplicate_or_old_packets()
        << " wrong_source=" << tracker_receiver.wrong_source_packets();
    if (tracker_receiver.rejected_packets() > 0) {
      std::cout
          << " last_decode_error="
          << panda_tracker::decode_status_message(
                 tracker_receiver.last_decode_error());
    }
    std::cout << '\n';

    if (!observed_robot_state) {
      std::cerr << "FAIL: no Panda state was observed.\n";
      return EXIT_FAILURE;
    }
    if (!observed_valid_tracker) {
      std::cerr << "FAIL: no fresh, valid PTP2 tracker pose was observed.\n";
      return EXIT_FAILURE;
    }
    if (pbvs_controller && !observed_pbvs_tracking) {
      std::cerr << "FAIL: compute-only PBVS never entered TRACKING.\n";
      return EXIT_FAILURE;
    }

    if (pbvs_controller) {
      std::cout
          << "PASS: compute-only PBVS entered TRACKING; no command was sent.\n";
    } else {
      std::cout << "PASS: Panda state and fresh PTP2 tracker data observed.\n";
    }
    return EXIT_SUCCESS;
  } catch (const franka::NetworkException& exception) {
    std::cerr << "Franka network error: " << exception.what() << '\n';
  } catch (const franka::Exception& exception) {
    std::cerr << "Franka error: " << exception.what() << '\n';
  } catch (const std::exception& exception) {
    std::cerr << "Error: " << exception.what() << '\n';
  }

  return EXIT_FAILURE;
}
