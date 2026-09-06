#include "panda_tracker/pbvs.h"

#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using panda_tracker::Transform;

struct Options {
  std::string robot_ip{"172.16.0.2"};
  std::string output_path{"touch_calibration_samples.csv"};
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto next = [&](const char* name) -> std::string {
      if (++i >= argc) {
        throw std::invalid_argument(std::string("Missing value for ") + name);
      }
      return argv[i];
    };

    if (arg == "--robot-ip") {
      options.robot_ip = next("--robot-ip");
    } else if (arg == "--output") {
      options.output_path = next("--output");
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Collect flange poses for stick-tip pivot/TCP calibration.\n\n"
          << "The robot is NEVER commanded. Move it manually / via your normal\n"
          << "safe positioning method, touch the same marked point with the tip,\n"
          << "then press ENTER to record one sample.\n\n"
          << "Usage: " << argv[0] << " [options]\n"
          << "  --robot-ip IP    Panda FCI address (default 172.16.0.2)\n"
          << "  --output PATH    CSV output (default touch_calibration_samples.csv)\n"
          << "  --help, -h       Show this help\n\n"
          << "Interactive commands:\n"
          << "  ENTER            record one sample\n"
          << "  q + ENTER        quit\n"
          << "  d + ENTER        delete the most recent sample from this run\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("Unknown option: " + arg);
    }
  }
  return options;
}

Transform from_franka(const std::array<double, 16>& value) {
  return panda_tracker::franka_column_major_transform(value);
}

struct Sample {
  std::uint64_t index{0};
  double timestamp_s{0.0};
  Transform O_T_F{};
};

double wall_time_s() {
  using Clock = std::chrono::system_clock;
  return std::chrono::duration<double>(
      Clock::now().time_since_epoch()).count();
}

void write_csv(
    const std::string& path,
    const std::vector<Sample>& samples) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    throw std::runtime_error("Cannot open output file: " + path);
  }

  out << "sample,timestamp_s";
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out << ",O_T_F_" << r << c;
    }
  }
  out << "\n";

  out << std::setprecision(17);
  for (const auto& s : samples) {
    out << s.index << ',' << s.timestamp_s;
    for (double v : s.O_T_F) {
      out << ',' << v;
    }
    out << '\n';
  }
}

void print_transform(const Transform& T) {
  std::cout << std::fixed << std::setprecision(6);
  for (int r = 0; r < 4; ++r) {
    std::cout << "  [";
    for (int c = 0; c < 4; ++c) {
      if (c) std::cout << ", ";
      std::cout << std::setw(10) << T[r * 4 + c];
    }
    std::cout << "]\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);

    std::cout
        << "READ-ONLY TOUCH CALIBRATION COLLECTOR\n"
        << "No robot controller is started and no robot command is sent.\n"
        << "Robot: " << options.robot_ip << "\n"
        << "Output: " << options.output_path << "\n\n"
        << "Procedure:\n"
        << "  1) Put the stick tip on ONE fixed marked point.\n"
        << "  2) Change robot orientation substantially between samples.\n"
        << "  3) Keep the same physical tip point on the same mark.\n"
        << "  4) Press ENTER to capture.\n"
        << "  5) Collect about 15-30 diverse poses.\n\n";

    franka::Robot robot(options.robot_ip);
    std::vector<Sample> samples;

    while (true) {
      std::cout << "[ENTER=capture, d=delete last, q=quit] > " << std::flush;

      std::string command;
      if (!std::getline(std::cin, command)) {
        break;
      }

      if (command == "q" || command == "Q") {
        break;
      }

      if (command == "d" || command == "D") {
        if (samples.empty()) {
          std::cout << "No sample to delete.\n";
        } else {
          const auto removed = samples.back().index;
          samples.pop_back();
          write_csv(options.output_path, samples);
          std::cout << "Deleted sample " << removed
                    << ". Remaining: " << samples.size() << "\n";
        }
        continue;
      }

      if (!command.empty()) {
        std::cout << "Unknown command. Press ENTER to capture, d to delete, q to quit.\n";
        continue;
      }

      const franka::RobotState state = robot.readOnce();

      // libfranka convention:
      //   O_T_EE = EE frame expressed in robot base/world O
      //   F_T_EE = EE frame expressed in flange F
      //
      // Therefore:
      //   O_T_F = O_T_EE * inverse(F_T_EE)
      //
      // This deliberately calibrates the stick tip with respect to the
      // physical Panda flange, not a configured EE/hand frame.
      const Transform O_T_EE = from_franka(state.O_T_EE);
      const Transform F_T_EE = from_franka(state.F_T_EE);
      const Transform O_T_F = panda_tracker::multiply_transform(
          O_T_EE,
          panda_tracker::invert_transform(F_T_EE));

      Sample sample{};
      sample.index = static_cast<std::uint64_t>(samples.size() + 1);
      sample.timestamp_s = wall_time_s();
      sample.O_T_F = O_T_F;
      samples.push_back(sample);

      write_csv(options.output_path, samples);

      std::cout << "\nCaptured sample " << sample.index << "\n";
      std::cout << "O_T_F (flange pose in robot base):\n";
      print_transform(O_T_F);
      std::cout << "Saved " << samples.size()
                << " sample(s) to " << options.output_path << "\n\n";
    }

    std::cout << "\nFinished with " << samples.size()
              << " sample(s) in " << options.output_path << "\n";
    return 0;

  } catch (const franka::NetworkException& e) {
    std::cerr << "Franka network error: " << e.what() << '\n';
  } catch (const franka::Exception& e) {
    std::cerr << "Franka error: " << e.what() << '\n';
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
  }

  return 1;
}
