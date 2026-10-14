#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#include "jaguar_control/robstride_protocol.hpp"

namespace robstride {
// RS00 is the motor supported by the Jaguar calibration and safety configuration.
inline MotorParams loadCybergearRS00(const std::string & path) {
  const auto motor = YAML::LoadFile(path)["motor_types"]["RS00"];
  if (!motor) throw std::invalid_argument("Missing motor_types.RS00 in " + path);
  MotorParams params;
  const auto read = [&](const char * key) {
    const double value = motor[key].as<double>();
    if (!std::isfinite(value)) throw std::invalid_argument(std::string("Nonfinite motor parameter: ") + key);
    return value;
  };
  params.p_min = read("min_position");
  params.p_max = read("max_position");
  params.v_min = read("min_velocity");
  params.v_max = read("max_velocity");
  params.t_min = read("min_effort");
  params.t_max = read("max_effort");
  params.kp_min = read("min_gain_kp");
  params.kp_max = read("max_gain_kp");
  params.kd_min = read("min_gain_kd");
  params.kd_max = read("max_gain_kd");
  params.firmware_torque_limit = read("limit_torque");
  params.temperature_scale = read("temperature_scale");
  if (!(params.p_min < 0 && params.p_max > 0 && params.v_min < 0 && params.v_max > 0 &&
        params.t_min <= -RS00_CONTROL_TORQUE_LIMIT_NM && params.t_max >= RS00_CONTROL_TORQUE_LIMIT_NM &&
        params.kp_min == 0 && params.kp_max > 0 && params.kd_min == 0 && params.kd_max > 0 &&
        params.firmware_torque_limit >= RS00_CONTROL_TORQUE_LIMIT_NM &&
        params.firmware_torque_limit <= RS00_TORQUE_LIMIT_NM && params.temperature_scale > 0)) {
    throw std::invalid_argument("RS00 motor parameters are incompatible with Jaguar safety limits");
  }
  return params;
}
}  // namespace robstride
