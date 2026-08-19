#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <string>
#include <linux/can.h>

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
  double t_min = -17.0;        // -17.0 Nm
  double t_max =  17.0;        // +17.0 Nm
  int direction = 1;
};

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
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  double p_target = p_des * params.direction;
  double v_target = v_des * params.direction;
  double t_target = tau_ff * params.direction;

  uint16_t p_mapped = linearMapping(p_target, params.p_min, params.p_max);
  uint16_t v_mapped = linearMapping(v_target, params.v_min, params.v_max);
  uint16_t kp_mapped = linearMapping(kp, params.kp_min, params.kp_max);
  uint16_t kd_mapped = linearMapping(kd, params.kd_min, params.kd_max);
  uint16_t tau_mapped = linearMapping(t_target, params.t_min, params.t_max);

  // Arbitration ID: (CMD_MOTOR_CONTROL << 24) | (tau_mapped << 8) | motor_id
  frame.can_id = ((static_cast<uint32_t>(CMD_MOTOR_CONTROL) & 0x1F) << 24) |
                 ((static_cast<uint32_t>(tau_mapped) & 0xFFFF) << 8) |
                 (static_cast<uint32_t>(motor_id) & 0xFF) |
                 CAN_EFF_FLAG;

  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>((p_mapped >> 8) & 0xFF);
  frame.data[1] = static_cast<uint8_t>(p_mapped & 0xFF);
  frame.data[2] = static_cast<uint8_t>((v_mapped >> 8) & 0xFF);
  frame.data[3] = static_cast<uint8_t>(v_mapped & 0xFF);
  frame.data[4] = static_cast<uint8_t>((kp_mapped >> 8) & 0xFF);
  frame.data[5] = static_cast<uint8_t>(kp_mapped & 0xFF);
  frame.data[6] = static_cast<uint8_t>((kd_mapped >> 8) & 0xFF);
  frame.data[7] = static_cast<uint8_t>(kd_mapped & 0xFF);

  return frame;
}

/**
 * @brief Builds Motor Enable Frame
 */
inline struct can_frame buildEnableMotorFrame(uint8_t motor_id, uint8_t master_id = 0xFD)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id = ((static_cast<uint32_t>(CMD_MOTOR_ENABLE) & 0x1F) << 24) |
                 ((static_cast<uint32_t>(master_id) & 0xFF) << 8) |
                 (static_cast<uint32_t>(motor_id) & 0xFF) |
                 CAN_EFF_FLAG;
  frame.can_dlc = 0;
  return frame;
}

/**
 * @brief Builds Motor Stop / Disable Frame
 */
inline struct can_frame buildStopMotorFrame(uint8_t motor_id, uint8_t master_id = 0xFD)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id = ((static_cast<uint32_t>(CMD_MOTOR_STOP) & 0x1F) << 24) |
                 ((static_cast<uint32_t>(master_id) & 0xFF) << 8) |
                 (static_cast<uint32_t>(motor_id) & 0xFF) |
                 CAN_EFF_FLAG;
  frame.can_dlc = 8;
  return frame;
}

/**
 * @brief Builds Set Run Mode Frame (Sets parameter 0x7005)
 */
inline struct can_frame buildSetRunModeFrame(uint8_t motor_id, RunMode mode, uint8_t master_id = 0xFD)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));

  frame.can_id = ((static_cast<uint32_t>(CMD_SINGLE_PARAM_WRITE) & 0x1F) << 24) |
                 ((static_cast<uint32_t>(master_id) & 0xFF) << 8) |
                 (static_cast<uint32_t>(motor_id) & 0xFF) |
                 CAN_EFF_FLAG;
  frame.can_dlc = 8;

  // Parameter index 0x7005 (run_mode) in Little Endian
  frame.data[0] = 0x05;
  frame.data[1] = 0x70;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;

  // Value in byte 4
  frame.data[4] = static_cast<uint8_t>(mode);
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  return frame;
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
  if (!(frame.can_id & CAN_EFF_FLAG)) {
    return fb;
  }

  uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
  fb.motor_id = static_cast<uint8_t>((raw_id >> 8) & 0xFF);

  if (frame.can_dlc < 8) {
    return fb;
  }

  uint16_t p_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  uint16_t v_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  uint16_t t_raw = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  uint16_t tem_raw = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

  fb.position = uintToFloat(p_raw, params.p_min, params.p_max) * params.direction;
  fb.velocity = uintToFloat(v_raw, params.v_min, params.v_max) * params.direction;
  fb.torque = uintToFloat(t_raw, params.t_min, params.t_max) * params.direction;
  fb.temperature = static_cast<double>(tem_raw) / 10.0;
  fb.valid = true;

  return fb;
}

}  // namespace robstride
