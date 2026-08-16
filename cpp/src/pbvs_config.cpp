#include "panda_tracker/pbvs.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace panda_tracker {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct JsonValue {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  Type type{Type::kNull};
  bool boolean{false};
  double number{0.0};
  std::string string{};
  std::vector<JsonValue> array{};
  std::map<std::string, JsonValue> object{};
};

class JsonParser {
 public:
  explicit JsonParser(std::string source) : source_(std::move(source)) {}

  JsonValue parse() {
    skip_whitespace();
    JsonValue value = parse_value();
    skip_whitespace();
    if (position_ != source_.size()) {
      fail("unexpected trailing content");
    }
    return value;
  }

 private:
  [[noreturn]] void fail(const std::string& message) const {
    throw std::runtime_error(
        "JSON parse error at byte " + std::to_string(position_) +
        ": " + message);
  }

  void skip_whitespace() {
    while (position_ < source_.size()) {
      const char character = source_[position_];
      if (character != ' ' && character != '\t' &&
          character != '\r' && character != '\n') {
        return;
      }
      ++position_;
    }
  }

  bool consume(char expected) {
    skip_whitespace();
    if (position_ < source_.size() && source_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  void expect(char expected) {
    if (!consume(expected)) {
      fail(std::string("expected '") + expected + "'");
    }
  }

  bool consume_literal(const char* literal) {
    const std::size_t start = position_;
    for (const char* current = literal; *current != '\0'; ++current) {
      if (position_ >= source_.size() ||
          source_[position_] != *current) {
        position_ = start;
        return false;
      }
      ++position_;
    }
    return true;
  }

  JsonValue parse_value() {
    skip_whitespace();
    if (position_ >= source_.size()) {
      fail("expected a value");
    }

    const char character = source_[position_];
    if (character == '{') {
      return parse_object();
    }
    if (character == '[') {
      return parse_array();
    }
    if (character == '"') {
      JsonValue value{};
      value.type = JsonValue::Type::kString;
      value.string = parse_string();
      return value;
    }
    if (character == '-' || (character >= '0' && character <= '9')) {
      return parse_number();
    }
    if (consume_literal("true")) {
      JsonValue value{};
      value.type = JsonValue::Type::kBool;
      value.boolean = true;
      return value;
    }
    if (consume_literal("false")) {
      JsonValue value{};
      value.type = JsonValue::Type::kBool;
      value.boolean = false;
      return value;
    }
    if (consume_literal("null")) {
      return JsonValue{};
    }
    fail("unsupported value");
  }

  JsonValue parse_object() {
    expect('{');
    JsonValue value{};
    value.type = JsonValue::Type::kObject;
    if (consume('}')) {
      return value;
    }

    while (true) {
      skip_whitespace();
      if (position_ >= source_.size() || source_[position_] != '"') {
        fail("expected an object key");
      }
      const std::string key = parse_string();
      expect(':');
      JsonValue child = parse_value();
      if (!value.object.emplace(key, std::move(child)).second) {
        fail("duplicate object key: " + key);
      }
      if (consume('}')) {
        return value;
      }
      expect(',');
    }
  }

  JsonValue parse_array() {
    expect('[');
    JsonValue value{};
    value.type = JsonValue::Type::kArray;
    if (consume(']')) {
      return value;
    }

    while (true) {
      value.array.push_back(parse_value());
      if (consume(']')) {
        return value;
      }
      expect(',');
    }
  }

  std::string parse_string() {
    expect('"');
    std::string result;
    while (position_ < source_.size()) {
      const char character = source_[position_++];
      if (character == '"') {
        return result;
      }
      if (static_cast<unsigned char>(character) < 0x20) {
        fail("control character in string");
      }
      if (character != '\\') {
        result.push_back(character);
        continue;
      }
      if (position_ >= source_.size()) {
        fail("unterminated string escape");
      }
      const char escaped = source_[position_++];
      switch (escaped) {
        case '"':
        case '\\':
        case '/':
          result.push_back(escaped);
          break;
        case 'b':
          result.push_back('\b');
          break;
        case 'f':
          result.push_back('\f');
          break;
        case 'n':
          result.push_back('\n');
          break;
        case 'r':
          result.push_back('\r');
          break;
        case 't':
          result.push_back('\t');
          break;
        case 'u':
          fail("unicode escapes are not supported in PBVS config strings");
        default:
          fail("invalid string escape");
      }
    }
    fail("unterminated string");
  }

  JsonValue parse_number() {
    const char* begin = source_.c_str() + position_;
    char* end = nullptr;
    errno = 0;
    const double number = std::strtod(begin, &end);
    if (end == begin || errno == ERANGE || !std::isfinite(number)) {
      fail("invalid finite number");
    }
    position_ += static_cast<std::size_t>(end - begin);

    JsonValue value{};
    value.type = JsonValue::Type::kNumber;
    value.number = number;
    return value;
  }

  std::string source_;
  std::size_t position_{0};
};

const JsonValue& require_type(
    const JsonValue& value,
    JsonValue::Type expected,
    const std::string& name) {
  if (value.type != expected) {
    throw std::runtime_error(name + " has the wrong JSON type");
  }
  return value;
}

const JsonValue& required_member(
    const JsonValue& object,
    const std::string& key) {
  require_type(object, JsonValue::Type::kObject, "root");
  const auto iterator = object.object.find(key);
  if (iterator == object.object.end()) {
    throw std::runtime_error("missing required config key: " + key);
  }
  return iterator->second;
}

const JsonValue* optional_member(
    const JsonValue& object,
    const std::string& key) {
  require_type(object, JsonValue::Type::kObject, "config object");
  const auto iterator = object.object.find(key);
  return iterator == object.object.end() ? nullptr : &iterator->second;
}

double required_number(const JsonValue& root, const std::string& key) {
  return require_type(
      required_member(root, key),
      JsonValue::Type::kNumber,
      key).number;
}

double optional_number(
    const JsonValue& root,
    const std::string& key,
    double fallback) {
  const JsonValue* value = optional_member(root, key);
  return value == nullptr
      ? fallback
      : require_type(*value, JsonValue::Type::kNumber, key).number;
}

bool optional_bool(
    const JsonValue& root,
    const std::string& key,
    bool fallback) {
  const JsonValue* value = optional_member(root, key);
  return value == nullptr
      ? fallback
      : require_type(*value, JsonValue::Type::kBool, key).boolean;
}

std::string optional_string(
    const JsonValue& root,
    const std::string& key,
    const std::string& fallback) {
  const JsonValue* value = optional_member(root, key);
  return value == nullptr
      ? fallback
      : require_type(*value, JsonValue::Type::kString, key).string;
}

Vector3 vector3(const JsonValue& value, const std::string& name) {
  require_type(value, JsonValue::Type::kArray, name);
  if (value.array.size() != 3) {
    throw std::runtime_error(name + " must contain three numbers");
  }
  Vector3 result{};
  for (std::size_t index = 0; index < 3; ++index) {
    result[index] = require_type(
        value.array[index],
        JsonValue::Type::kNumber,
        name).number;
  }
  return result;
}

Transform transform4(const JsonValue& value, const std::string& name) {
  require_type(value, JsonValue::Type::kArray, name);
  if (value.array.size() != 4) {
    throw std::runtime_error(name + " must have four rows");
  }

  Transform result{};
  for (std::size_t row = 0; row < 4; ++row) {
    require_type(value.array[row], JsonValue::Type::kArray, name);
    if (value.array[row].array.size() != 4) {
      throw std::runtime_error(name + " rows must have four numbers");
    }
    for (std::size_t column = 0; column < 4; ++column) {
      result[row * 4 + column] = require_type(
          value.array[row].array[column],
          JsonValue::Type::kNumber,
          name).number;
    }
  }
  return result;
}

std::size_t positive_integer(double number, const std::string& name) {
  if (!std::isfinite(number) || number < 1.0 ||
      std::floor(number) != number ||
      number > static_cast<double>(
          std::numeric_limits<std::size_t>::max())) {
    throw std::runtime_error(name + " must be a positive integer");
  }
  return static_cast<std::size_t>(number);
}

double radians(double degrees) {
  return degrees * kPi / 180.0;
}

}  // namespace

bool load_pbvs_config(
    const std::string& path,
    PbvsConfig& config,
    std::string& error) {
  try {
    std::ifstream stream(path);
    if (!stream) {
      throw std::runtime_error("unable to open config: " + path);
    }
    std::ostringstream contents;
    contents << stream.rdbuf();
    if (!stream.good() && !stream.eof()) {
      throw std::runtime_error("unable to read config: " + path);
    }

    const JsonValue root = JsonParser(contents.str()).parse();
    require_type(root, JsonValue::Type::kObject, "root");

    PbvsConfig loaded{};
    loaded.control_rate_hz = required_number(root, "control_rate_hz");
    loaded.control_orientation = optional_bool(
        root, "control_orientation", true);
    loaded.kp_position = required_number(root, "kp_position");
    loaded.kp_orientation = required_number(root, "kp_orientation");
    loaded.max_linear_speed = required_number(root, "max_linear_speed");
    loaded.max_angular_speed = radians(
        required_number(root, "max_angular_speed_deg"));
    loaded.max_command_lead = required_number(root, "max_command_lead");
    loaded.panda_state_timeout = required_number(
        root, "panda_state_timeout");
    loaded.tracker_timeout = required_number(root, "tracker_timeout");
    loaded.max_tracker_position_jump = required_number(
        root, "max_tracker_position_jump");
    loaded.max_tracker_angle_jump = radians(
        required_number(root, "max_tracker_angle_jump_deg"));
    loaded.max_enable_position_error = required_number(
        root, "max_enable_position_error");
    loaded.max_enable_orientation_error = radians(
        required_number(root, "max_enable_orientation_error_deg"));
    loaded.consecutive_valid_required = positive_integer(
        required_number(root, "consecutive_valid_required"),
        "consecutive_valid_required");
    loaded.target_feedforward_enabled = optional_bool(
        root, "target_feedforward_enabled", false);
    loaded.target_velocity_filter_alpha = optional_number(
        root, "target_velocity_filter_alpha", 0.25);
    loaded.max_target_linear_speed = optional_number(
        root,
        "max_target_linear_speed",
        loaded.max_linear_speed);
    loaded.max_target_angular_speed = radians(optional_number(
        root,
        "max_target_angular_speed_deg",
        required_number(root, "max_angular_speed_deg")));
    loaded.T_ES = transform4(required_member(root, "T_ES"), "T_ES");
    loaded.T_TS_des = transform4(
        required_member(root, "T_TS_des"), "T_TS_des");
    loaded.tool_geometry_status = optional_string(
        root, "tool_geometry_status", "unspecified");

    const auto workspace_iterator = root.object.find("workspace");
    if (workspace_iterator == root.object.end()) {
      throw std::runtime_error("missing required config key: workspace");
    }
    const JsonValue& workspace = workspace_iterator->second;
    require_type(workspace, JsonValue::Type::kObject, "workspace");
    loaded.workspace_min = vector3(
        required_member(workspace, "min"), "workspace.min");
    loaded.workspace_max = vector3(
        required_member(workspace, "max"), "workspace.max");

    std::string validation_error;
    if (!validate_pbvs_config(loaded, validation_error)) {
      throw std::runtime_error(validation_error);
    }

    config = std::move(loaded);
    error.clear();
    return true;
  } catch (const std::exception& exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace panda_tracker
