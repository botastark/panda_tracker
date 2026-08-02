#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  std::size_t key_pos = text.find(needle);
  if (key_pos == std::string::npos) {
    std::cerr << "Key not found: " << key << std::endl;
    return false;
  }
  std::size_t start = text.find('[', key_pos);
  if (start == std::string::npos) {
    std::cerr << "Could not find opening '[' for key: " << key << std::endl;
    return false;
  }
  int depth = 0;
  std::size_t end = std::string::npos;
  for (std::size_t i = start; i < text.size(); ++i) {
    if (text[i] == '[') depth++;
    else if (text[i] == ']') {
      depth--;
      if (depth == 0) {
        end = i;
        break;
      }
    }
  }
  if (end == std::string::npos) {
    std::cerr << "Could not find matching closing ']' for key: " << key << std::endl;
    return false;
  }

  std::string block = text.substr(start, end - start + 1);
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
}

int main(int argc, char** argv) {
  std::string json_path = (argc >= 2) ? argv[1] : "pbvs_robot.json";
  Mat4 T_EC{}, T_CS{}, T_ES{};
  if (!loadTransform(json_path, "T_EC", T_EC)) return -1;
  if (!loadTransform(json_path, "T_CS", T_CS)) return -1;
  if (!loadTransform(json_path, "T_ES", T_ES)) return -1;

  Mat4 T_ECS = multiply(T_EC, T_CS);
  std::cout << "Loaded transforms from " << json_path << std::endl;
  printPoseLine("E->C", T_EC);
  printPoseLine("C->S", T_CS);
  printPoseLine("E->S(config)", T_ES);
  printPoseLine("E->S(from E->C->S)", T_ECS);
  std::printf("ES delta [m]: %.6f %.6f %.6f\n",
              T_ECS[12] - T_ES[12],
              T_ECS[13] - T_ES[13],
              T_ECS[14] - T_ES[14]);

  try {
    franka::Robot robot(kRobotIp);
    std::cout << "Connected to Panda. Press Enter to inspect current predicted tool poses; q + Enter to quit.\n";
    std::string line;
    while (true) {
      std::cout << "[Enter=sample, q=quit] > ";
      std::getline(std::cin, line);
      if (!std::cin.good() || line == "q" || line == "Q") break;
      franka::RobotState state = robot.readOnce();
      Mat4 T_OE = state.O_T_EE;
      Mat4 T_OC = multiply(T_OE, T_EC);
      Mat4 T_OS_cfg = multiply(T_OE, T_ES);
      Mat4 T_OS_via_c = multiply(T_OC, T_CS);
      printPoseLine("O->E", T_OE);
      printPoseLine("O->C", T_OC);
      printPoseLine("O->S(config)", T_OS_cfg);
      printPoseLine("O->S(via camera)", T_OS_via_c);
      std::printf("OS delta [m]: %.6f %.6f %.6f\n",
                  T_OS_via_c[12] - T_OS_cfg[12],
                  T_OS_via_c[13] - T_OS_cfg[13],
                  T_OS_via_c[14] - T_OS_cfg[14]);
    }
    return 0;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
}
