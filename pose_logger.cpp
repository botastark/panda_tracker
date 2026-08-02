#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <ctime>

namespace {
constexpr const char* kRobotIp = "172.16.0.2";

std::string defaultLogPath() {
  std::time_t t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  char buf[128];
  std::strftime(buf, sizeof(buf), "pose_log_%Y%m%d_%H%M%S.csv", &tm);
  return std::string(buf);
}

void writeHeader(std::ofstream& out) {
  out << "sample_idx,x,y,z,a_deg,b_deg,g_deg,";
  out << "r00,r01,r02,r03,r10,r11,r12,r13,r20,r21,r22,r23,r30,r31,r32,r33\n";
}
}

int main(int argc, char** argv) {
  std::string csv_path = (argc >= 2) ? argv[1] : defaultLogPath();

  try {
    franka::Robot robot(kRobotIp);
    std::ofstream csv(csv_path, std::ios::out);
    if (!csv.is_open()) {
      std::cerr << "Failed to open log file: " << csv_path << std::endl;
      return -1;
    }

    writeHeader(csv);
    std::cout << "Connected to Panda at " << kRobotIp << std::endl;
    std::cout << "Logging to: " << csv_path << std::endl;
    std::cout << "Move robot by hand, then press Enter to record current pose." << std::endl;
    std::cout << "Type q then Enter to quit." << std::endl;

    std::string line;
    int sample_idx = 0;
    while (true) {
      std::cout << "[Enter=record, q=quit] > ";
      std::getline(std::cin, line);
      if (!std::cin.good()) {
        break;
      }
      if (line == "q" || line == "Q") {
        break;
      }

      franka::RobotState state = robot.readOnce();
      const auto& T = state.O_T_EE;
      double x = T[12];
      double y = T[13];
      double z = T[14];
      double b = std::atan2(-T[2], std::sqrt(T[0] * T[0] + T[1] * T[1]));
      double a = std::atan2(T[6], T[10]);
      double g = std::atan2(T[1], T[0]);

      csv << sample_idx << ','
          << std::fixed << std::setprecision(6)
          << x << ',' << y << ',' << z << ','
          << a * 180.0 / M_PI << ','
          << b * 180.0 / M_PI << ','
          << g * 180.0 / M_PI;
      for (size_t i = 0; i < T.size(); ++i) {
        csv << ',' << T[i];
      }
      csv << '\n';
      csv.flush();

      std::printf("saved %03d | %.3f | %.3f | %.3f | %.3f | %.3f | %.3f\n",
                  sample_idx,
                  x,
                  y,
                  z,
                  a * 180.0 / M_PI,
                  b * 180.0 / M_PI,
                  g * 180.0 / M_PI);
      sample_idx++;
    }

    std::cout << "Done. Saved " << sample_idx << " poses." << std::endl;
    return 0;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
