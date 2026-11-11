#pragma once

#include <array>
#include <chrono>
#include <string>

namespace jaguar_control
{
constexpr size_t JOINT_COUNT = 12;
constexpr size_t COMMAND_COUNT = 5;
constexpr size_t STATE_COUNT = 4;
inline const std::array<std::string, JOINT_COUNT> JOINT_NAMES = {
  "Bl_roll_joint", "Bl_hip_pitch_joint", "Bl_knee_joint",
  "Br_roll_joint", "Br_hip_pitch_joint", "Br_knee_joint",
  "Fl_roll_joint", "Fl_hip_pitch_joint", "Fl_knee_joint",
  "Fr_roll_joint", "Fr_hip_pitch_joint", "Fr_knee_joint"};
inline const std::array<std::string, JOINT_COUNT> LEGACY_NAMES = {
  "BL_collar_joint", "BL_hip_joint", "BL_knee_joint",
  "BR_collar_joint", "BR_hip_joint", "BR_knee_joint",
  "FL_collar_joint", "FL_hip_joint", "FL_knee_joint",
  "FR_collar_joint", "FR_hip_joint", "FR_knee_joint"};
inline const std::array<std::string, COMMAND_COUNT> COMMAND_NAMES = {
  "position", "velocity", "kp", "kd", "effort"};
inline const std::array<std::string, STATE_COUNT> STATE_NAMES = {
  "position", "velocity", "effort", "temperature"};
inline const std::array<std::string, 3> SAFETY_COMMANDS = {
  "emergency_stop", "safe_park_active", "reset_fault"};
inline const std::array<std::string, 5> SAFETY_STATES = {
  "emergency_stopped", "passive", "park_requested", "resetting", "torque_limit"};

inline int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace jaguar_control
