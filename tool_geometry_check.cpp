#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr const char* kRobotIp = "172.16.0.2";

using Mat4 = std::array<double, 16>;

std::vector<double> extractNumbers(const std::string& text) {
  std::vector<double> values;
  std::string token;
  for (char c : text) {
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
      token.push_back(c);
    } else if (!token.empty()) {
      values.push_back(std::stod(token));
      token.clear();
    }
  }
  if (!token.empty()) {
    values.push_back(std::stod(token));
  }
  return values;
}

Mat4 rowMajorToColMajor(const std::vector<double>& row_major) {
  Mat4 out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out[c * 4 + r] = row_major[r * 4 + c];
    }
  }
  return out;
}

Mat4 multiply(const Mat4& A, const Mat4& B) {
  Mat4 C{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += A[k * 4 + r] * B[c * 4 + k];
      }
      C[c * 4 + r] = sum;
    }
  }
  return C;
}

bool loadTransform(const std::string& json_path, const std::string& key, Mat4& T) {
  std::ifstream in(json_path);
  if (!in.is_open()) {
    std::cerr << "Failed to open config: " << json_path << std::endl;
    return false;
  }
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  std::string needle = "\"" + key + "\"";
  std::size_t pos = text.find(needle);
  if (pos == std::string::npos) {
    std::cerr << "Key not found: " << key << std::endl;
    return false;
  }
  std::size_t start = text.find('[', pos);
  std::size_t end = text.find(']', start);
  while (end != std::string::npos && text.substr(start, end - start).find("[[") == std::string::npos) {
    end = text.find(']', end + 1);
  }
  std::size_t close = text.find("]]", start);
  if (start == std::string::npos || close == std::string::npos) {
    std::cerr << "Could not parse matrix for key: " << key << std::endl;
    return false;
  }
  std::string block = text.substr(start, close - start + 2);
  std::vector<double> vals = extractNumbers(block);
  if (vals.size() != 16) {
    std::cerr << "Expected 16 numbers for " << key << ", got " << vals.size() << std::endl;
    return false;
  }
  T = rowMajorToColMajor(vals);
  return true;
}

void printPoseLine(const char* name, const Mat4& T) {
  double x = T[12], y = T[13], z = T[14];
  double b = std::atan2(-T[2], std::sqrt(T[0] * T[0] + T[1] * T[1]));
  double a = std::atan2(T[6], T[10]);
  double g = std::atan2(T[1], T[0]);
  std::printf("%s: %.4f %.4f %.4f | %.2f %.2f %.2f deg\n",
              name, x, y, z,
              a * 180.0 / M_PI,
              b * 180.0 / M_PI,
              g * 180.0 / M_PI);
}

void writeCsvHeader(std::ofstream& out) {
  out << "sample_idx,";
  out << "E_x,E_y,E_z,E_a_deg,E_b_deg,E_g_deg,";
  out << "C_x,C_y,C_z,C_a_deg,C_b_deg,C_g_deg,";
  out << "S_x,S_y,S_z,S_a_deg,S_b_deg,S_g_deg\n";
}

void appendPose(std::ofstream& out, const Mat4& T) {
  double x = T[12], y = T[13], z = T[14];
  double b = std::atan2(-T[2], std::sqrt(T[0] * T[0] + T[1] * T[1]));
  double a = std::atan2(T[6], T[10]);
  double g = std::atan2(T[1], T[0]);
  out << std::fixed << std::setprecision(6)
      << x << ',' << y << ',' << z << ','
      << a * 180.0 / M_PI << ','
      << b * 180.0 / M_PI << ','
      << g * 180.0 / M_PI;
}
}

int main(int argc, char** argv) {
  std::string json_path = (argc >= 2) ? argv[1] : "pbvs_robot.json";
  std::string csv_path = (argc >= 3) ? argv[2] : "tool_geometry_check.csv";

  Mat4 T_EC{}, T_CS{}, T_ES{};
  if (!loadTransform(json_path, "T_EC", T_EC)) return -1;
  if (!loadTransform(json_path, "T_CS", T_CS)) return -1;
  if (!loadTransform(json_path, "T_ES", T_ES)) return -1;
  Mat4 T_EC_times_T_CS = multiply(T_EC, T_CS);

  std::cout << "Loaded transforms from " << json_path << std::endl;
  printPoseLine("Configured E->C", T_EC);
  printPoseLine("Configured C->S", T_CS);
  printPoseLine("Configured E->S", T_ES);
  printPoseLine("Computed E->C->S", T_EC_times_T_CS);

  try {
    franka::Robot robot(kRobotIp);
    std::ofstream csv(csv_path, std::ios::out);
    if (!csv.is_open()) {
      std::cerr << "Failed to open CSV: " << csv_path << std::endl;
      return -1;
    }
    writeCsvHeader(csv);

    std::cout << "Connected to Panda. Press Enter to sample current tool geometry; q + Enter to quit." << std::endl;
    std::string line;
    int sample_idx = 0;
    while (true) {
      std::cout << "[Enter=sample, q=quit] > ";
      std::getline(std::cin, line);
      if (!std::cin.good() || line == "q" || line == "Q") break;

      franka::RobotState state = robot.readOnce();
      Mat4 T_OE = state.O_T_EE;
      Mat4 T_OC = multiply(T_OE, T_EC);
      Mat4 T_OS_from_CS = multiply(T_OC, T_CS);
      Mat4 T_OS_from_ES = multiply(T_OE, T_ES);

      std::cout << "Sample " << sample_idx << std::endl;
      printPoseLine("O->E", T_OE);
      printPoseLine("O->C", T_OC);
      printPoseLine("O->S via E->C->S", T_OS_from_CS);
      printPoseLine("O->S via E->S", T_OS_from_ES);
      std::printf("S-position delta [m]: %.6f %.6f %.6f\n",
                  T_OS_from_CS[12] - T_OS_from_ES[12],
                  T_OS_from_CS[13] - T_OS_from_ES[13],
                  T_OS_from_CS[14] - T_OS_from_ES[14]);

      csv << sample_idx << ',';
      appendPose(csv, T_OE); csv << ',';
      appendPose(csv, T_OC); csv << ',';
      appendPose(csv, T_OS_from_CS); csv << '\n';
      csv.flush();
      sample_idx++;
    }

    std::cout << "Saved samples to " << csv_path << std::endl;
    return 0;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
