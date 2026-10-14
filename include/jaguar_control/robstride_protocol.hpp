#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <string>
#include <linux/can.h>
#include "cybergear_driver_core/cybergear_packet.hpp"

namespace robstride
{

// =============================================================================
// RobStride RS00 Motor Physical & Dynamic Parameters
// =============================================================================
struct MotorParams
{
  double p_min = -4.0 * M_PI;  // -12.56637 rad
  double p_max =  4.0 * M_PI;  // +12.56637 rad
  double v_min = -33.0;        // -33.0 rad/s
  double v_max =  33.0;        // +33.0 rad/s
  double kp_min =  0.0;
  double kp_max = 500.0;
  double kd_min =  0.0;
  double kd_max =  5.0;
  double t_min = -14.0;        // -14.0 Nm
  double t_max =  14.0;        // +14.0 Nm
  double firmware_torque_limit = 14.0;
  double temperature_scale = 0.1;
  int direction = 1;
};

constexpr double RS00_TORQUE_LIMIT_NM = 14.0;
// Match the Isaac Lab Tibia training actuator limit during normal operation.
constexpr double RS00_CONTROL_TORQUE_LIMIT_NM = 13.5;
constexpr uint32_t PARAM_LIMIT_TORQUE = 0x700B;

// Adapt the copied CyberGear packet implementation to the existing Jaguar API.
inline cybergear_driver_core::CybergearPacket makeCybergearPacket(
  uint8_t motor_id, const MotorParams & params, uint8_t master_id = 0xFE)
{
  cybergear_driver_core::CybergearPacketParam packet_params;
  packet_params.device_id = motor_id;
  packet_params.primary_id = master_id;
  packet_params.min_position = params.p_min;
  packet_params.max_position = params.p_max;
  packet_params.min_velocity = params.v_min;
  packet_params.max_velocity = params.v_max;
  packet_params.min_effort = params.t_min;
  packet_params.max_effort = params.t_max;
  packet_params.min_gain_kp = params.kp_min;
  packet_params.max_gain_kp = params.kp_max;
  packet_params.min_gain_kd = params.kd_min;
  packet_params.max_gain_kd = params.kd_max;
  packet_params.temperature_scale = params.temperature_scale;
  return cybergear_driver_core::CybergearPacket(packet_params);
}

inline struct can_frame toSocketCan(const cybergear_driver_core::CanFrame & packet)
{
  struct can_frame frame{};
  frame.can_id = packet.id | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::copy(packet.data.begin(), packet.data.end(), frame.data);
  return frame;
}

// =============================================================================
// CAN Protocol Command Modes & Run Modes
// =============================================================================
enum CommandMode : uint8_t
{
  CMD_GET_DEVICE_ID      = 0,
  CMD_MOTOR_CONTROL      = 1,  // MIT Impedance Control Mode
  CMD_MOTOR_FEEDBACK     = 2,
  CMD_MOTOR_ENABLE       = 3,
  CMD_MOTOR_STOP         = 4,
  CMD_SET_MECHANICAL_ZERO= 6,
  CMD_SET_MOTOR_CAN_ID   = 7,
  CMD_PARAM_TABLE_WRITE  = 8,
  CMD_SINGLE_PARAM_READ  = 17,
  CMD_SINGLE_PARAM_WRITE = 18,
  CMD_FAULT_FEEDBACK     = 21
};

enum RunMode : uint8_t
{
  RUN_CONTROL_MODE  = 0,  // MIT Mode (p, v, kp, kd, tau_ff)
  RUN_POSITION_MODE = 1,
  RUN_VELOCITY_MODE = 2,
  RUN_CURRENT_MODE  = 3
};

// =============================================================================
// Helper Functions: Linear Mapping & Value Packing
// =============================================================================
inline uint16_t floatToUint(double val, double min_val, double max_val, int bits = 16)
{
  double clamped = std::clamp(val, min_val, max_val);
  double span = max_val - min_val;
  if (span <= 0.0) return 0;
  uint32_t max_int = (1U << bits) - 1U;
  return static_cast<uint16_t>(((clamped - min_val) * max_int) / span);
}

inline double uintToFloat(uint16_t val, double min_val, double max_val, int bits = 16)
{
  uint32_t max_int = (1U << bits) - 1U;
  double span = max_val - min_val;
  return min_val + (static_cast<double>(val) * span) / static_cast<double>(max_int);
}

inline uint16_t linearMapping(double val, double in_min, double in_max, uint16_t out_min = 0, uint16_t out_max = 65535)
{
  double clamped = std::clamp(val, in_min, in_max);
  double span = in_max - in_min;
  if (span <= 0.0) return out_min;
  return static_cast<uint16_t>(
    ((clamped - in_min) / span) * (out_max - out_min) + out_min
  );
}

// =============================================================================
// RobStride CAN Frame Builders
// =============================================================================

/**
 * @brief Builds an MIT Mode Control Frame (Command Mode 1)
 * @param motor_id Target motor CAN ID (1..6)
 * @param p_des Desired joint position [rad]
 * @param v_des Desired joint velocity [rad/s]
 * @param kp Position gain [0..500]
 * @param kd Velocity gain [0..5]
 * @param tau_ff Feedforward torque [Nm]
 * @param params Motor parameter limits
 * @return struct can_frame ready to send via SocketCAN
 */
inline struct can_frame buildMitControlFrame(
  uint8_t motor_id,
  double p_des,
  double v_des,
  double kp,
  double kd,
  double tau_ff,
  const MotorParams & params)
{
  auto packet = makeCybergearPacket(motor_id, params);
  cybergear_driver_core::MoveParam command;
  command.position = p_des * params.direction;
  command.velocity = v_des * params.direction;
  command.effort = tau_ff * params.direction;
  command.kp = kp;
  command.kd = kd;
  return toSocketCan(packet.createMoveCommand(command));
}

/**
 * @brief Builds Motor Enable Frame
 */
inline struct can_frame buildEnableMotorFrame(uint8_t motor_id, uint8_t master_id = 0xFE)
{
  struct can_frame frame{};
  auto packet = makeCybergearPacket(motor_id, MotorParams{}, master_id);
  frame.can_id = packet.frameId().getEnableTorqueId() | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  return frame;
}

/**
 * @brief Builds Motor Stop / Disable Frame
 */
inline struct can_frame buildStopMotorFrame(uint8_t motor_id, uint8_t master_id = 0xFE,
                                            bool clear_fault = false)
{
  struct can_frame frame{};
  auto packet = makeCybergearPacket(motor_id, MotorParams{}, master_id);
  frame.can_id = packet.frameId().getResetTorqueId() | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  frame.data[0] = clear_fault ? 1 : 0;
  return frame;
}

/**
 * @brief Builds Set Run Mode Frame (Sets parameter 0x7005)
 */
inline struct can_frame buildSetRunModeFrame(uint8_t motor_id, RunMode mode, uint8_t master_id = 0xFE)
{
  auto packet = makeCybergearPacket(motor_id, MotorParams{}, master_id);
  return toSocketCan(packet.createChangeRunMode(static_cast<uint8_t>(mode)));
}

/**
 * @brief Builds a write frame for the RobStride firmware torque limit.
 *
 * The motor applies this limit to its internal MIT impedance controller.
 * Command and feedback use the CyberGear RS00 encoding range [-14, 14] N m.
 * The Jaguar operating cap is configured separately.
 */
inline struct can_frame buildSetTorqueLimitFrame(
  uint8_t motor_id, double torque_limit_nm = RS00_TORQUE_LIMIT_NM, uint8_t master_id = 0xFE)
{
  auto packet = makeCybergearPacket(motor_id, MotorParams{}, master_id);
  return toSocketCan(packet.createLimitTorqueCommand(
    static_cast<float>(std::clamp(torque_limit_nm, 0.0, RS00_TORQUE_LIMIT_NM))));
}

// =============================================================================
// RobStride Feedback Parser
// =============================================================================
struct MotorFeedback
{
  bool valid = false;
  uint8_t motor_id = 0;
  double position = 0.0;     // rad
  double velocity = 0.0;     // rad/s
  double torque = 0.0;       // Nm
  double temperature = 0.0;  // deg C
  bool error = false;
};

inline MotorFeedback parseFeedbackFrame(const struct can_frame & frame, const MotorParams & params)
{
  MotorFeedback fb;
  if (!(frame.can_id & CAN_EFF_FLAG) ||
      (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) || frame.can_dlc != 8) {
    return fb;
  }
  const uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
  fb.motor_id = static_cast<uint8_t>((raw_id >> 8) & 0xFF);
  // The packet class requires the device and primary IDs to differ.
  if (fb.motor_id == 0xFE) return fb;
  auto packet = makeCybergearPacket(fb.motor_id, params);
  if (!packet.frameId().isFeedback(raw_id)) return fb;
  cybergear_driver_core::CanData data{};
  std::copy(std::begin(frame.data), std::end(frame.data), data.begin());
  fb.position = packet.parsePosition(data) * params.direction;
  fb.velocity = packet.parseVelocity(data) * params.direction;
  fb.torque = packet.parseEffort(data) * params.direction;
  fb.temperature = packet.parseTemperature(data);
  fb.error = packet.frameId().hasError(raw_id);
  fb.valid = true;
  return fb;
}

}  // namespace robstride
