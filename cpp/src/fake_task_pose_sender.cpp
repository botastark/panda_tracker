#include "panda_tracker/task_pose_protocol.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) {
  stop_requested = 1;
}

struct Options {
  std::string destination_ip{"127.0.0.1"};
  std::string source_bind_ip{};
  std::uint16_t destination_port{6501};
  double rate_hz{50.0};
  double duration_s{10.0};
  double base_x_m{0.0};
  double base_y_m{0.0};
  double base_z_m{0.05};
  double amplitude_m{0.01};
  double frequency_hz{0.2};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
  float confidence{1.0F};
  std::string pattern{"static"};
  bool valid{true};
  bool read_only_receiver_confirmed{false};
};

void print_help(const char* program) {
  std::cout
      << "Send synthetic PTP2 tracker poses over UDP.\n\n"
      << "TEST DATA ONLY. Use only with the read-only pbvs_observer.\n\n"
      << "Usage: " << program << " [options]\n"
      << "  --confirm-read-only-receiver Required safety acknowledgement\n"
      << "  --destination-ip IP         Receiver IP (default 127.0.0.1)\n"
      << "  --destination-port PORT     Receiver port (default 6501)\n"
      << "  --source-bind-ip IP         Optional local interface address\n"
      << "  --rate HZ                   Packet rate in (0,1000] (default 50)\n"
      << "  --duration SECONDS          0 means until interrupted (default 10)\n"
      << "  --pattern NAME              static, sine-x, sine-y, sine-z, "
         "or circle-xy\n"
      << "  --base-x METERS             Base T_TS x (default 0)\n"
      << "  --base-y METERS             Base T_TS y (default 0)\n"
      << "  --base-z METERS             Base T_TS z (default 0.05)\n"
      << "  --amplitude METERS          Pattern amplitude (default 0.01)\n"
      << "  --frequency HZ              Pattern frequency (default 0.2)\n"
      << "  --roll-deg DEGREES          Fixed T_TS roll (default 0)\n"
      << "  --pitch-deg DEGREES         Fixed T_TS pitch (default 0)\n"
      << "  --yaw-deg DEGREES           Fixed T_TS yaw (default 0)\n"
      << "  --confidence VALUE          Value in [0,1] (default 1)\n"
      << "  --invalid                   Publish valid=false test packets\n"
      << "  --help, -h                  Show this message\n";
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
    throw std::invalid_argument("--destination-port must be in [1, 65535].");
  }
  return static_cast<std::uint16_t>(value);
}

bool valid_pattern(const std::string& pattern) {
  return pattern == "static" || pattern == "sine-x" ||
         pattern == "sine-y" || pattern == "sine-z" ||
         pattern == "circle-xy";
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

    if (argument == "--confirm-read-only-receiver") {
      options.read_only_receiver_confirmed = true;
    } else if (argument == "--destination-ip") {
      options.destination_ip = next_value("--destination-ip");
    } else if (argument == "--destination-port") {
      options.destination_port = parse_port(
          next_value("--destination-port"));
    } else if (argument == "--source-bind-ip") {
      options.source_bind_ip = next_value("--source-bind-ip");
    } else if (argument == "--rate") {
      options.rate_hz = parse_double(next_value("--rate"), "--rate");
    } else if (argument == "--duration") {
      options.duration_s = parse_double(
          next_value("--duration"), "--duration");
    } else if (argument == "--pattern") {
      options.pattern = next_value("--pattern");
    } else if (argument == "--base-x") {
      options.base_x_m = parse_double(next_value("--base-x"), "--base-x");
    } else if (argument == "--base-y") {
      options.base_y_m = parse_double(next_value("--base-y"), "--base-y");
    } else if (argument == "--base-z") {
      options.base_z_m = parse_double(next_value("--base-z"), "--base-z");
    } else if (argument == "--amplitude") {
      options.amplitude_m = parse_double(
          next_value("--amplitude"), "--amplitude");
    } else if (argument == "--frequency") {
      options.frequency_hz = parse_double(
          next_value("--frequency"), "--frequency");
    } else if (argument == "--roll-deg") {
      options.roll_deg = parse_double(
          next_value("--roll-deg"), "--roll-deg");
    } else if (argument == "--pitch-deg") {
      options.pitch_deg = parse_double(
          next_value("--pitch-deg"), "--pitch-deg");
    } else if (argument == "--yaw-deg") {
      options.yaw_deg = parse_double(
          next_value("--yaw-deg"), "--yaw-deg");
    } else if (argument == "--confidence") {
      options.confidence = static_cast<float>(parse_double(
          next_value("--confidence"), "--confidence"));
    } else if (argument == "--invalid") {
      options.valid = false;
    } else if (argument == "--help" || argument == "-h") {
      print_help(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("Unsupported option: " + argument);
    }
  }

  if (!options.read_only_receiver_confirmed) {
    throw std::invalid_argument(
        "Refusing to send test poses without "
        "--confirm-read-only-receiver.");
  }
  if (!(options.rate_hz > 0.0) || options.rate_hz > 1000.0) {
    throw std::invalid_argument("--rate must be in (0, 1000].");
  }
  if (options.duration_s < 0.0) {
    throw std::invalid_argument("--duration must be non-negative.");
  }
  if (!valid_pattern(options.pattern)) {
    throw std::invalid_argument(
        "--pattern must be static, sine-x, sine-y, sine-z, or circle-xy.");
  }
  if (options.amplitude_m < 0.0) {
    throw std::invalid_argument("--amplitude must be non-negative.");
  }
  if (options.frequency_hz < 0.0) {
    throw std::invalid_argument("--frequency must be non-negative.");
  }
  if (options.confidence < 0.0F || options.confidence > 1.0F) {
    throw std::invalid_argument("--confidence must be in [0, 1].");
  }

  return options;
}

class UdpSocket {
 public:
  UdpSocket() {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error(
          std::string("Unable to create UDP socket: ") +
          std::strerror(errno));
    }
  }

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  ~UdpSocket() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  int fd() const { return fd_; }

 private:
  int fd_{-1};
};

sockaddr_in ipv4_address(
    const std::string& ip,
    std::uint16_t port,
    const char* option) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1) {
    throw std::invalid_argument(std::string("Invalid ") + option + ": " + ip);
  }
  return address;
}

panda_tracker::TaskPosePacket make_packet(
    const Options& options,
    std::uint64_t sequence,
    double elapsed_s) {
  panda_tracker::TaskPosePacket packet{};
  const double roll = options.roll_deg * kPi / 180.0;
  const double pitch = options.pitch_deg * kPi / 180.0;
  const double yaw = options.yaw_deg * kPi / 180.0;
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  packet.T_TS = {{
      cy * cp, cy * sp * sr - sy * cr,
      cy * sp * cr + sy * sr, options.base_x_m,
      sy * cp, sy * sp * sr + cy * cr,
      sy * sp * cr - cy * sr, options.base_y_m,
      -sp, cp * sr, cp * cr, options.base_z_m,
      0.0, 0.0, 0.0, 1.0,
  }};

  const double phase = 2.0 * kPi * options.frequency_hz * elapsed_s;
  if (options.pattern == "sine-x") {
    packet.T_TS[3] += options.amplitude_m * std::sin(phase);
  } else if (options.pattern == "sine-y") {
    packet.T_TS[7] += options.amplitude_m * std::sin(phase);
  } else if (options.pattern == "sine-z") {
    packet.T_TS[11] += options.amplitude_m * std::sin(phase);
  } else if (options.pattern == "circle-xy") {
    packet.T_TS[3] += options.amplitude_m * std::cos(phase);
    packet.T_TS[7] += options.amplitude_m * std::sin(phase);
  }

  packet.sequence_id = sequence;
  packet.confidence = options.confidence;
  packet.valid = options.valid;
  return packet;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    UdpSocket socket;
    if (!options.source_bind_ip.empty()) {
      const sockaddr_in source = ipv4_address(
          options.source_bind_ip, 0, "--source-bind-ip");
      if (bind(
              socket.fd(),
              reinterpret_cast<const sockaddr*>(&source),
              sizeof(source)) < 0) {
        throw std::runtime_error(
            std::string("Unable to bind source interface: ") +
            std::strerror(errno));
      }
    }

    const sockaddr_in destination = ipv4_address(
        options.destination_ip,
        options.destination_port,
        "--destination-ip");

    std::cout
        << "SYNTHETIC TRACKER TEST DATA\n"
        << "Confirmed receiver: read-only pbvs_observer\n"
        << "Destination: " << options.destination_ip << ':'
        << options.destination_port << '\n'
        << "Pattern: " << options.pattern << " rate_hz="
        << options.rate_hz << " duration_s=" << options.duration_s
        << " rpy_deg=[" << options.roll_deg << ' '
        << options.pitch_deg << ' ' << options.yaw_deg << "]\n";

    const auto start = Clock::now();
    auto next_send = start;
    auto next_console = start;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / options.rate_hz));
    std::uint64_t sequence = 1;

    while (stop_requested == 0) {
      const auto now = Clock::now();
      const double elapsed_s =
          std::chrono::duration<double>(now - start).count();
      if (options.duration_s > 0.0 && elapsed_s >= options.duration_s) {
        break;
      }

      const auto packet = make_packet(options, sequence, elapsed_s);
      const auto bytes = panda_tracker::encode_task_pose(packet);
      const ssize_t sent = sendto(
          socket.fd(),
          bytes.data(),
          bytes.size(),
          0,
          reinterpret_cast<const sockaddr*>(&destination),
          sizeof(destination));
      if (sent != static_cast<ssize_t>(bytes.size())) {
        throw std::runtime_error(
            std::string("UDP sendto failed: ") + std::strerror(errno));
      }

      if (now >= next_console) {
        std::cout
            << std::fixed << std::setprecision(4)
            << "seq=" << sequence << " p_TS_m=["
            << packet.T_TS[3] << ' ' << packet.T_TS[7] << ' '
            << packet.T_TS[11] << "] confidence="
            << packet.confidence << " valid="
            << (packet.valid ? 1 : 0) << '\n';
        next_console = now + std::chrono::seconds(1);
      }

      ++sequence;
      next_send += period;
      std::this_thread::sleep_until(next_send);
    }

    std::cout << "Sent " << sequence - 1 << " PTP2 packets.\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& exception) {
    std::cerr << "Error: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
