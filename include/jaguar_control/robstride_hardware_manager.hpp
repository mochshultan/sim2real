#pragma once

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "jaguar_control/robstride_protocol.hpp"
#include "jaguar_control/robstride_can_bus.hpp"

namespace robstride
{

constexpr size_t N_JOINTS = 12;

struct JointConfig
{
  std::string name;
  std::string bus_name;
  uint8_t can_id;
  int direction;
  double angle_offset;
  double pos_min;
  double pos_max;
  double max_vel;
  double max_effort;
  MotorParams motor_params;
};

struct JointCommand
{
  double position = 0.0;
  double velocity = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double effort = 0.0;
};

struct JointStateData
{
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  double temperature = 25.0;
  bool feedback_valid = false;
  std::chrono::steady_clock::time_point last_feedback_time;
};

class RobStrideHardwareManager
{
public:
  RobStrideHardwareManager()
  : initialized_(false),
    emergency_stopped_(false),
    is_passive_mode_(true),
    watchdog_timeout_sec_(0.1)
  {
    setupDefaultJointConfigs();
  }

  ~RobStrideHardwareManager()
  {
    shutdown();
  }

  void setupDefaultJointConfigs()
  {
    // Order: BL (0..2), BR (3..5), FL (6..8), FR (9..11)
    joint_configs_.resize(N_JOINTS);

    // BL (can1)
    joint_configs_[0] = {"BL_collar_joint", "can1", 4,  1, 0.0, -0.40,  0.40, 20.0, 17.0, {}};
    joint_configs_[1] = {"BL_hip_joint",    "can1", 5, -1, 0.0, -3.14,  3.14, 20.0, 17.0, {}};
    joint_configs_[2] = {"BL_knee_joint",   "can1", 6, -1, 0.0, -0.10,  2.80, 20.0, 17.0, {}};

    // BR (can0)
    joint_configs_[3] = {"BR_collar_joint", "can0", 4,  1, 0.0, -0.40,  0.40, 20.0, 17.0, {}};
    joint_configs_[4] = {"BR_hip_joint",    "can0", 5,  1, 0.0, -3.14,  3.14, 20.0, 17.0, {}};
    joint_configs_[5] = {"BR_knee_joint",   "can0", 6,  1, 0.0, -0.10,  2.80, 20.0, 17.0, {}};

    // FL (can1)
    joint_configs_[6] = {"FL_collar_joint", "can1", 1, -1, 0.0, -0.40,  0.40, 20.0, 17.0, {}};
    joint_configs_[7] = {"FL_hip_joint",    "can1", 2, -1, 0.0, -3.14,  3.14, 20.0, 17.0, {}};
    joint_configs_[8] = {"FL_knee_joint",   "can1", 3, -1, 0.0, -0.10,  2.80, 20.0, 17.0, {}};

    // FR (can0)
    joint_configs_[9] = {"FR_collar_joint", "can0", 1, -1, 0.0, -0.40,  0.40, 20.0, 17.0, {}};
    joint_configs_[10] ={"FR_hip_joint",    "can0", 2,  1, 0.0, -3.14,  3.14, 20.0, 17.0, {}};
    joint_configs_[11] ={"FR_knee_joint",   "can0", 3,  1, 0.0, -0.10,  2.80, 20.0, 17.0, {}};

    for (size_t i = 0; i < N_JOINTS; ++i) {
      joint_configs_[i].motor_params.direction = joint_configs_[i].direction;
    }

    commands_.resize(N_JOINTS);
    states_.resize(N_JOINTS);
  }

  bool initializeBuses()
  {
    bus_can0_ = std::make_unique<RobStrideCanBus>("can0");
    bus_can1_ = std::make_unique<RobStrideCanBus>("can1");

    bool can0_ok = bus_can0_->openBus();
    bool can1_ok = bus_can1_->openBus();

    if (!can0_ok && !can1_ok) {
      std::cerr << "[RobStrideHardwareManager] Failed to open both can0 and can1!" << std::endl;
      return false;
    }

    std::cout << "[RobStrideHardwareManager] SocketCAN buses initialized: can0=" 
              << (can0_ok ? "OK" : "FAILED") << ", can1=" 
              << (can1_ok ? "OK" : "FAILED") << std::endl;
    return true;
  }

  bool enableAndConfigureMotors()
  {
    if (!bus_can0_ || !bus_can1_) {
      return false;
    }

    std::cout << "[RobStrideHardwareManager] Enabling all 12 RobStride RS00 motors..." << std::endl;
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
      if (!bus || !bus->isOpen()) continue;

      // 1. Send Enable Frame
      struct can_frame enable_frame = buildEnableMotorFrame(cfg.can_id);
      bus->sendFrame(enable_frame);
      usleep(10000);  // 10ms delay between commands

      // 2. Set MIT Control Mode (0x7005 = RUN_CONTROL_MODE)
      struct can_frame mode_frame = buildSetRunModeFrame(cfg.can_id, RUN_CONTROL_MODE);
      bus->sendFrame(mode_frame);
      usleep(10000);

      // 3. Set to Safe Passive Zero-Torque MIT Mode
      struct can_frame mit_passive = buildMitControlFrame(
        cfg.can_id, 0.0, 0.0, 0.0, 0.0, 0.0, cfg.motor_params
      );
      bus->sendFrame(mit_passive);
      usleep(10000);
    }

    // Flush RX queues to capture initial positions
    readIncomingFeedbacks();

    initialized_ = true;
    is_passive_mode_ = true;
    last_command_time_ = std::chrono::steady_clock::now();
    std::cout << "[RobStrideHardwareManager] All motors initialized in Safe Passive Zero-Torque Mode." << std::endl;
    return true;
  }

  void setJointCommand(size_t joint_index, const JointCommand & cmd)
  {
    if (joint_index >= N_JOINTS) return;

    // Safety checks: Reject NaN or Inf
    if (!std::isfinite(cmd.position) || !std::isfinite(cmd.velocity) ||
        !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd) || !std::isfinite(cmd.effort))
    {
      triggerEmergencyStop("Received NaN or Inf in joint command!");
      return;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    const auto & cfg = joint_configs_[joint_index];

    commands_[joint_index].position = std::clamp(cmd.position, cfg.pos_min, cfg.pos_max);
    commands_[joint_index].velocity = std::clamp(cmd.velocity, -cfg.max_vel, cfg.max_vel);
    commands_[joint_index].kp = std::clamp(cmd.kp, cfg.motor_params.kp_min, cfg.motor_params.kp_max);
    commands_[joint_index].kd = std::clamp(cmd.kd, cfg.motor_params.kd_min, cfg.motor_params.kd_max);
    commands_[joint_index].effort = std::clamp(cmd.effort, -cfg.max_effort, cfg.max_effort);

    is_passive_mode_ = false;
    last_command_time_ = std::chrono::steady_clock::now();
  }

  void setPassiveMode(bool passive)
  {
    is_passive_mode_ = passive;
  }

  void triggerEmergencyStop(const std::string & reason)
  {
    emergency_stopped_ = true;
    std::cerr << "[RobStrideHardwareManager] EMERGENCY STOP: " << reason << std::endl;
    disableAllMotors();
  }

  void disableAllMotors()
  {
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
      if (!bus || !bus->isOpen()) continue;

      struct can_frame stop_frame = buildStopMotorFrame(cfg.can_id);
      bus->sendFrame(stop_frame);
    }
  }

  void stepCommunicationCycle()
  {
    if (!initialized_) return;

    auto now = std::chrono::steady_clock::now();
    double cmd_age_sec = std::chrono::duration<double>(now - last_command_time_).count();

    bool timeout = (cmd_age_sec > watchdog_timeout_sec_);
    if (timeout && !is_passive_mode_) {
      // Fallback to passive sensing mode when commands are stale
      is_passive_mode_ = true;
    }

    // 1. Send Commands to Motors
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      for (size_t i = 0; i < N_JOINTS; ++i) {
        const auto & cfg = joint_configs_[i];
        RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
        if (!bus || !bus->isOpen()) continue;

        struct can_frame frame;
        if (emergency_stopped_) {
          frame = buildStopMotorFrame(cfg.can_id);
        } else if (is_passive_mode_) {
          // Zero torque, zero gains
          frame = buildMitControlFrame(cfg.can_id, 0.0, 0.0, 0.0, 0.0, 0.0, cfg.motor_params);
        } else {
          const auto & cmd = commands_[i];
          frame = buildMitControlFrame(
            cfg.can_id,
            cmd.position + cfg.angle_offset,
            cmd.velocity,
            cmd.kp,
            cmd.kd,
            cmd.effort,
            cfg.motor_params
          );
        }
        bus->sendFrame(frame);
      }
    }

    // 2. Read Incoming Motor Feedback
    readIncomingFeedbacks();
  }

  void readIncomingFeedbacks()
  {
    auto now = std::chrono::steady_clock::now();

    // Read all pending frames on can0
    if (bus_can0_ && bus_can0_->isOpen()) {
      struct can_frame frame;
      while (bus_can0_->receiveFrame(frame)) {
        processFeedbackFrame(frame, "can0", now);
      }
    }

    // Read all pending frames on can1
    if (bus_can1_ && bus_can1_->isOpen()) {
      struct can_frame frame;
      while (bus_can1_->receiveFrame(frame)) {
        processFeedbackFrame(frame, "can1", now);
      }
    }
  }

  void processFeedbackFrame(const struct can_frame & frame, const std::string & bus_name, const std::chrono::steady_clock::time_point & now)
  {
    uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
    uint8_t motor_id = static_cast<uint8_t>((raw_id >> 8) & 0xFF);

    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      if (cfg.bus_name == bus_name && cfg.can_id == motor_id) {
        MotorFeedback fb = parseFeedbackFrame(frame, cfg.motor_params);
        if (fb.valid) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          states_[i].position = fb.position - cfg.angle_offset;
          states_[i].velocity = fb.velocity;
          states_[i].effort = fb.torque;
          states_[i].temperature = fb.temperature;
          states_[i].feedback_valid = true;
          states_[i].last_feedback_time = now;
        }
        break;
      }
    }
  }

  JointStateData getJointState(size_t joint_index)
  {
    if (joint_index >= N_JOINTS) return {};
    std::lock_guard<std::mutex> lock(state_mutex_);
    return states_[joint_index];
  }

  std::vector<JointStateData> getAllJointStates()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return states_;
  }

  const std::vector<JointConfig> & getJointConfigs() const
  {
    return joint_configs_;
  }

  void shutdown()
  {
    if (initialized_) {
      std::cout << "[RobStrideHardwareManager] Shutting down and disabling all motors..." << std::endl;
      disableAllMotors();
      if (bus_can0_) bus_can0_->closeBus();
      if (bus_can1_) bus_can1_->closeBus();
      initialized_ = false;
    }
  }

  bool isInitialized() const { return initialized_; }
  bool isEmergencyStopped() const { return emergency_stopped_; }
  bool isPassiveMode() const { return is_passive_mode_; }

private:
  std::vector<JointConfig> joint_configs_;
  std::vector<JointCommand> commands_;
  std::vector<JointStateData> states_;

  std::unique_ptr<RobStrideCanBus> bus_can0_;
  std::unique_ptr<RobStrideCanBus> bus_can1_;

  std::mutex cmd_mutex_;
  std::mutex state_mutex_;

  std::atomic<bool> initialized_;
  std::atomic<bool> emergency_stopped_;
  std::atomic<bool> is_passive_mode_;

  double watchdog_timeout_sec_;
  std::chrono::steady_clock::time_point last_command_time_;
};

}  // namespace robstride
