// Read-only Panda state broadcaster for laptop-side hand-eye calibration.
// No control mode, configuration write, or gripper API is used.

#include <franka/exception.h>
#include <franka/robot.h>

#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

std::atomic<bool> g_running{true};

void stop(int) { g_running.store(false, std::memory_order_relaxed); }

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string destination_ip;
  std::uint16_t destination_port{6510};
  double rate_hz{50.0};
};

struct Sample {
  std::array<double, 16> O_T_EE{};
  std::array<double, 16> F_T_EE{};
  std::uint64_t robot_time_ms{0};
  bool valid{false};
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    const auto next = [&](const char* flag) -> std::string {
      if (++i >= argc) throw std::invalid_argument(std::string("Missing value for ") + flag);
      return argv[i];
    };
    if (arg == "--robot-ip") options.robot_ip = next("--robot-ip");
    else if (arg == "--destination-ip") options.destination_ip = next("--destination-ip");
    else if (arg == "--destination-port") {
      const unsigned long value = std::stoul(next("--destination-port"));
      if (value == 0 || value > 65535) throw std::invalid_argument("Invalid destination port");
      options.destination_port = static_cast<std::uint16_t>(value);
    } else if (arg == "--rate") {
      options.rate_hz = std::stod(next("--rate"));
      if (!(options.rate_hz > 0.0 && options.rate_hz <= 200.0))
        throw std::invalid_argument("--rate must be in (0, 200]");
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Read-only Panda pose broadcaster for hand-eye calibration.\n\n"
          << "This program never starts a robot control loop and never writes robot configuration.\n\n"
          << "Usage: " << argv[0] << " --destination-ip LAPTOP_IP [options]\n"
          << "  --robot-ip IP          FCI address (default 172.16.0.2)\n"
          << "  --destination-ip IP    Laptop address (required)\n"
          << "  --destination-port P   UDP port (default 6510)\n"
          << "  --rate HZ              Stream rate, <= 200 (default 50)\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }
  if (options.destination_ip.empty())
    throw std::invalid_argument("--destination-ip is required");
  return options;
}

// libfranka stores matrices column-major; hand-eye files and the network
// protocol use row-major T_XY (frame Y expressed in frame X).
std::array<double, 16> row_major(const std::array<double, 16>& column_major) {
  std::array<double, 16> result{};
  for (std::size_t row = 0; row < 4; ++row)
    for (std::size_t col = 0; col < 4; ++col)
      result[row * 4 + col] = column_major[col * 4 + row];
  return result;
}

std::string encode_packet(std::uint64_t sequence, const Sample& sample) {
  // Text packets avoid floating-point byte-order ambiguity between hosts.
  // RPS1,version,sequence,robot_time_ms,16 O_T_EE values,16 F_T_EE values
  std::ostringstream output;
  output << std::setprecision(17) << "RPS1,1," << sequence << ','
         << sample.robot_time_ms;
  const auto O_T_EE = row_major(sample.O_T_EE);
  const auto F_T_EE = row_major(sample.F_T_EE);
  for (const double value : O_T_EE) output << ',' << value;
  for (const double value : F_T_EE) output << ',' << value;
  return output.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) throw std::runtime_error("Cannot create UDP socket");
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(options.destination_port);
    if (::inet_pton(AF_INET, options.destination_ip.c_str(), &destination.sin_addr) != 1) {
      ::close(socket_fd);
      throw std::invalid_argument("--destination-ip must be an IPv4 address");
    }

    std::mutex sample_mutex;
    Sample latest;
    std::atomic<bool> reader_done{false};
    std::string reader_error;

    std::thread reader([&] {
      try {
        franka::Robot robot(options.robot_ip);
        robot.read([&](const franka::RobotState& state) {
          {
            std::lock_guard<std::mutex> lock(sample_mutex);
            latest.O_T_EE = state.O_T_EE;
            latest.F_T_EE = state.F_T_EE;
            latest.robot_time_ms = state.time.toMSec();
            latest.valid = true;
          }
          return g_running.load(std::memory_order_relaxed);
        });
      } catch (const std::exception& error) {
        reader_error = error.what();
        g_running.store(false, std::memory_order_relaxed);
      }
      reader_done.store(true, std::memory_order_release);
    });

    std::cout << "READ-ONLY PANDA POSE STREAMER\n"
              << "Robot: " << options.robot_ip << "\n"
              << "Destination: " << options.destination_ip << ':' << options.destination_port << "\n"
              << "Protocol: RPS1 text UDP, row-major O_T_EE and F_T_EE\n"
              << "Rate: " << options.rate_hz << " Hz\n";

    const auto period = std::chrono::duration<double>(1.0 / options.rate_hz);
    auto next = std::chrono::steady_clock::now();
    std::uint64_t sequence = 0;
    std::uint64_t sent = 0;
    while (g_running.load(std::memory_order_relaxed)) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      Sample sample;
      {
        std::lock_guard<std::mutex> lock(sample_mutex);
        sample = latest;
      }
      if (sample.valid) {
        const std::string packet = encode_packet(++sequence, sample);
        const ssize_t result = ::sendto(socket_fd, packet.data(), packet.size(), 0,
                                        reinterpret_cast<const sockaddr*>(&destination),
                                        sizeof(destination));
        if (result != static_cast<ssize_t>(packet.size()))
          std::cerr << "WARNING: UDP send failed\n";
        else if (++sent % static_cast<std::uint64_t>(options.rate_hz) == 0)
          std::cout << "sent=" << sent << " robot_time_ms=" << sample.robot_time_ms << "\r"
                    << std::flush;
      }
      std::this_thread::sleep_until(next);
    }
    ::close(socket_fd);
    if (reader.joinable()) reader.join();
    if (!reader_error.empty()) throw std::runtime_error("libfranka read error: " + reader_error);
    std::cout << "\nStopped. Packets sent: " << sent << '\n';
    return reader_done.load(std::memory_order_acquire) ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
