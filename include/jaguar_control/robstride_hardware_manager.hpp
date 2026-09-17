#pragma once

#include <vector>
#include <array>
#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include <atomic>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

#include "jaguar_control/robstride_protocol.hpp"
#include "jaguar_control/robstride_can_bus.hpp"

namespace robstride
{

constexpr size_t N_JOINTS = 12;
// Feedback tolerance only; commanded positions retain their nominal limits.
constexpr double POSITION_LIMIT_TOLERANCE = 0.2;

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
    joint_configs_[0] = {"BL_collar_joint", "can1", 4,  1, -0.3245, -0.40,  0.40, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[1] = {"BL_hip_joint",    "can1", 5, -1, +1.3483, -3.14,  3.14, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[2] = {"BL_knee_joint",   "can1", 6, -1, +0.0488, -0.10,  2.80, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};

    // BR (can0)
    joint_configs_[3] = {"BR_collar_joint", "can0", 4,  1, +0.3017, -0.40,  0.40, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[4] = {"BR_hip_joint",    "can0", 5,  1, +1.3476, -3.14,  3.14, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[5] = {"BR_knee_joint",   "can0", 6,  1, +0.0039, -0.10,  2.80, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};

    // FL (can1)
    joint_configs_[6] = {"FL_collar_joint", "can1", 1, -1, -0.3826, -0.40,  0.40, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[7] = {"FL_hip_joint",    "can1", 2, -1, +1.2127, -3.14,  3.14, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[8] = {"FL_knee_joint",   "can1", 3, -1, +0.0967, -0.10,  2.80, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};

    // FR (can0)
    joint_configs_[9] = {"FR_collar_joint", "can0", 1, -1, +0.3281, -0.40,  0.40, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[10] ={ "FR_hip_joint",    "can0", 2,  1, +1.1767, -3.14,  3.14, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};
    joint_configs_[11] ={ "FR_knee_joint",   "can0", 3,  1, +0.0927, -0.10,  2.80, 20.0, RS00_CONTROL_TORQUE_LIMIT_NM, {}};

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

    if (!can0_ok || !can1_ok) {
      std::cerr << "[RobStrideHardwareManager] Both CAN buses are required." << std::endl;
      return false;
    }

    std::cout << "[RobStrideHardwareManager] SocketCAN buses initialized: can0=" 
              << (can0_ok ? "OK" : "FAILED") << ", can1=" 
              << (can1_ok ? "OK" : "FAILED") << std::endl;
    return true;
  }

  bool enableAndConfigureMotors(bool clear_faults_on_startup = false)
  {
    if (!bus_can0_ || !bus_can1_ || !bus_can0_->isOpen() || !bus_can1_->isOpen()) {
      return false;
    }

    if (clear_faults_on_startup) {
      std::cout << "[RobStrideHardwareManager] Startup fault-clear requested for all 12 motors (motors disabled)..." << std::endl;
      startup_clearing_ = true;
      for (const auto & cfg : joint_configs_) {
        auto * bus = cfg.bus_name == "can0" ? bus_can0_.get() : bus_can1_.get();
        if (!bus || !bus->sendFrame(buildStopMotorFrame(cfg.can_id, 0xFE, true))) {
          startup_clearing_ = false;
          std::cerr << "[RobStrideHardwareManager] Startup fault-clear CAN write failed on " << cfg.name << std::endl;
          disableAllMotors();
          return false;
        }
        usleep(10000);
      }
      usleep(50000);
      // Log pre-clear reports, but do not latch an old fault before the motor
      // has had a chance to process the clear command. Any subsequent fault
      // during enable/operation still triggers the ordinary emergency stop.
      readIncomingFeedbacks();
      startup_clearing_ = false;
    }

    std::cout << "[RobStrideHardwareManager] Enabling all 12 RobStride RS00 motors..." << std::endl;
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
      if (!bus || !bus->isOpen()) continue;

      // Configure impedance mode while the motor is stopped.
      const can_frame sequence[] = {
        buildStopMotorFrame(cfg.can_id),
        buildSetTorqueLimitFrame(cfg.can_id, cfg.max_effort),
        buildSetRunModeFrame(cfg.can_id, RUN_CONTROL_MODE),
        buildMitControlFrame(cfg.can_id, 0, 0, 0, 0, 0, cfg.motor_params),
        buildEnableMotorFrame(cfg.can_id)};
      for (const auto & frame : sequence) {
        if (!bus->sendFrame(frame)) {
          disableAllMotors();
          return false;
        }
        usleep(10000);
        readIncomingFeedbacks();
        if (emergency_stopped_) {
          disableAllMotors();
          return false;
        }
      }
    }

    // Flush RX queues to capture initial positions
    readIncomingFeedbacks();
    if (emergency_stopped_) {
      disableAllMotors();
      return false;
    }
    // Startup readiness requires replies to passive commands sent after enable,
    // not feedback left over from the fault-clear/configuration sequence.
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (auto & state : states_) state.feedback_valid = false;
    }

    initialized_ = true;
    is_passive_mode_ = true;
    last_command_time_ = std::chrono::steady_clock::now();
    std::cout << "[RobStrideHardwareManager] Configuration sent; active control requires fresh feedback from all motors." << std::endl;
    return true;
  }

  bool setJointCommands(const std::vector<JointCommand> & commands)
  {
    if (commands.size() != N_JOINTS || emergency_stopped_) return false;
    bool active = false;
    auto bounded = commands;
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cmd = commands[i];
      const auto & cfg = joint_configs_[i];
      if (!std::isfinite(cmd.position) || !std::isfinite(cmd.velocity) ||
          !std::isfinite(cmd.kp) || !std::isfinite(cmd.kd) || !std::isfinite(cmd.effort) ||
          cmd.kp < 0 || cmd.kd < 0) {
        triggerEmergencyStop("Invalid joint command");
        return false;
      }
      active |= cmd.kp != 0 || cmd.kd != 0 || cmd.effort != 0;
      bounded[i].position = std::clamp(cmd.position, cfg.pos_min, cfg.pos_max);
      bounded[i].velocity = std::clamp(cmd.velocity, -cfg.max_vel, cfg.max_vel);
      bounded[i].kp = std::clamp(cmd.kp, cfg.motor_params.kp_min, cfg.motor_params.kp_max);
      bounded[i].kd = std::clamp(cmd.kd, cfg.motor_params.kd_min, cfg.motor_params.kd_max);
      bounded[i].effort = std::clamp(cmd.effort, -cfg.max_effort, cfg.max_effort);
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    commands_ = std::move(bounded);
    is_passive_mode_ = !active;
    last_command_time_ = std::chrono::steady_clock::now();
    return true;
  }

  bool safeParkRequested() const { return safe_park_requested_; }

  void setSafeParkActive(bool active)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (active && !safe_park_active_) {
      soft_fault_since_ = {};
      safe_park_started_ = std::chrono::steady_clock::now();
    }
    safe_park_active_ = active;
    if (active) safe_park_requested_ = false;
  }

  void setPassiveMode(bool passive)
  {
    is_passive_mode_ = passive;
  }

  void triggerEmergencyStop(const std::string & reason)
  {
    if (!emergency_stopped_.exchange(true) || reset_in_progress_) {
      std::cerr << "[RobStrideHardwareManager] EMERGENCY STOP: " << reason << std::endl;
    }
    reset_in_progress_ = false;
    // Only the communication thread (or shutdown after join) writes to CAN.
  }

  bool requestFaultReset()
  {
    if (!initialized_ || !emergency_stopped_ || reset_in_progress_ || reset_requested_) return false;
    reset_requested_ = true;
    return true;
  }

  bool isResetInProgress() const { return reset_in_progress_ || reset_requested_; }

  void disableAllMotors()
  {
    // 1. Send passive zero-torque frame to all motors first
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
      if (!bus || !bus->isOpen()) continue;

      struct can_frame passive_frame = buildMitControlFrame(cfg.can_id, 0.0, 0.0, 0.0, 0.0, 0.0, cfg.motor_params);
      bus->sendFrame(passive_frame);
    }
    usleep(5000);  // 5ms buffer flush

    // 2. Send stop frames
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
      if (!bus || !bus->isOpen()) continue;

      struct can_frame stop_frame = buildStopMotorFrame(cfg.can_id);
      bus->sendFrame(stop_frame);
    }
    usleep(5000);
  }

  void stepCommunicationCycle()
  {
    if (!initialized_) return;
    readIncomingFeedbacks();
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock_commands(cmd_mutex_);
    if (reset_requested_.exchange(false)) startFaultReset(now);
    if (reset_in_progress_) verifyFaultReset(now);
    double cmd_age_sec = std::chrono::duration<double>(now - last_command_time_).count();

    bool timeout = (cmd_age_sec > watchdog_timeout_sec_);
    if (timeout && !is_passive_mode_) {
      triggerEmergencyStop("Command watchdog expired");
    }

    checkMotionSafety(now);

    // 1. Send Commands to Motors
    {
      for (size_t i = 0; i < N_JOINTS; ++i) {
        const auto & cfg = joint_configs_[i];
        RobStrideCanBus * bus = (cfg.bus_name == "can0") ? bus_can0_.get() : bus_can1_.get();
        if (!bus || !bus->isOpen()) continue;

        struct can_frame frame;
        if (reset_in_progress_ && reset_enable_sent_) {
          frame = buildMitControlFrame(cfg.can_id, 0, 0, 0, 0, 0, cfg.motor_params);
        } else if (emergency_stopped_) {
          frame = buildStopMotorFrame(cfg.can_id);
        } else if (is_passive_mode_) {
          // Zero torque, zero gains
          frame = buildMitControlFrame(cfg.can_id, 0.0, 0.0, 0.0, 0.0, 0.0, cfg.motor_params);
        } else {
          auto cmd = commands_[i];
          if (safe_park_requested_) {
            // Bounded low-gain hold while the controller starts its park trajectory.
            cmd = {park_hold_pos_[i], 0.0, 14.0, 0.5, 0.0};
          }
          double p_clamped = std::clamp(cmd.position, cfg.pos_min, cfg.pos_max);
          frame = buildMitControlFrame(
            cfg.can_id,
            p_clamped + cfg.angle_offset,
            cmd.velocity,
            cmd.kp,
            cmd.kd,
            cmd.effort,
            cfg.motor_params
          );
        }
        if (!bus->sendFrame(frame)) triggerEmergencyStop("CAN write failed");
      }
    }

    // 2. Read Incoming Motor Feedback
    readIncomingFeedbacks();
  }

  // Called under cmd_mutex_ by the communication loop; public for offline tests.
  void checkMotionSafety(const std::chrono::steady_clock::time_point & now)
  {
    if (is_passive_mode_ || emergency_stopped_) { soft_fault_since_ = {}; return; }
    if (safe_park_requested_ &&
        std::chrono::duration<double>(now - park_request_time_).count() > 0.25) {
      triggerEmergencyStop("SAFE_PARK controller acknowledgement timed out");
      return;
    }
    if (safe_park_active_ &&
        std::chrono::duration<double>(now - safe_park_started_).count() > 10.0) {
      triggerEmergencyStop("SAFE_PARK completion timed out");
      return;
    }
    std::lock_guard<std::mutex> lock_state(state_mutex_);
    bool soft_fault = false;
    std::string reason;
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & state = states_[i];
      const auto & cfg = joint_configs_[i];
      const double age = std::chrono::duration<double>(now - state.last_feedback_time).count();
      const double extra = std::max({cfg.pos_min - state.position,
                                    state.position - cfg.pos_max, 0.0});
      if (!state.feedback_valid || age > feedback_timeout_sec_ ||
          !std::isfinite(state.position) || extra > POSITION_LIMIT_TOLERANCE + 0.2) {
        triggerEmergencyStop("Invalid/stale feedback or severe position excursion on " + cfg.name);
        return;
      }
      const double error = std::abs(state.position - commands_[i].position);
      // Existing modest overshoot can recover during park, but may not worsen.
      const double limit = safe_park_active_ ?
        std::max(POSITION_LIMIT_TOLERANCE, park_initial_extra_[i] + 0.05) :
        POSITION_LIMIT_TOLERANCE;
      if (extra > limit || error > 1.57) {
        soft_fault = true;
        reason = cfg.name + " tracking=" + std::to_string(error) +
          " rad (limit=1.57), position excess=" + std::to_string(extra);
      }
      if (!safe_park_active_) {
        park_hold_pos_[i] = state.position;
        park_initial_extra_[i] = extra;
      }
    }
    if (safe_park_requested_) return;
    if (!soft_fault) { soft_fault_since_ = {}; return; }
    if (soft_fault_since_ == std::chrono::steady_clock::time_point{}) soft_fault_since_ = now;
    if (std::chrono::duration<double>(now - soft_fault_since_).count() < 0.1) return;
    if (safe_park_active_) {
      triggerEmergencyStop("Fault during SAFE_PARK: " + reason);
    } else {
      park_request_time_ = now;
      safe_park_requested_ = true;
      std::cerr << "[RobStrideHardwareManager] SAFE_PARK requested: " << reason << std::endl;
    }
  }

  void readIncomingFeedbacks()
  {
    // Read all pending frames on can0
    if (bus_can0_ && bus_can0_->isOpen()) {
      struct can_frame frame;
      for (int n = 0; n < 256 && bus_can0_->receiveFrame(frame); ++n) {
        processFeedbackFrame(frame, "can0", std::chrono::steady_clock::now());
      }
    }

    // Read all pending frames on can1
    if (bus_can1_ && bus_can1_->isOpen()) {
      struct can_frame frame;
      for (int n = 0; n < 256 && bus_can1_->receiveFrame(frame); ++n) {
        processFeedbackFrame(frame, "can1", std::chrono::steady_clock::now());
      }
    }
  }

  void processFeedbackFrame(const struct can_frame & frame, const std::string & bus_name, const std::chrono::steady_clock::time_point & now)
  {
    uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
    if (!(frame.can_id & CAN_EFF_FLAG) || (frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) ||
        (raw_id & 0xFF) != 0xFE) return;
    uint8_t motor_id = static_cast<uint8_t>((raw_id >> 8) & 0xFF);

    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & cfg = joint_configs_[i];
      if (cfg.bus_name == bus_name && cfg.can_id == motor_id) {
        if (((raw_id >> 24) & 0x1F) == 21) {
          uint32_t fault = 0, warning = 0;
          if (frame.can_dlc == 8) {
            for (int byte = 0; byte < 4; ++byte) {
              fault |= static_cast<uint32_t>(frame.data[byte]) << (8 * byte);
              warning |= static_cast<uint32_t>(frame.data[4 + byte]) << (8 * byte);
            }
          }
          std::cerr << "[RobStrideHardwareManager] MOTOR_FAULT " << cfg.name
                    << " bus=" << bus_name << " id=" << static_cast<int>(cfg.can_id)
                    << " fault=0x" << std::hex << fault << " warning=0x" << warning
                    << std::dec << " dlc=" << static_cast<int>(frame.can_dlc) << std::endl;
          if (!startup_clearing_) triggerEmergencyStop("Motor fault frame from " + cfg.name);
          std::lock_guard<std::mutex> lock(state_mutex_);
          states_[i].feedback_valid = false;
          break;
        }
        MotorFeedback fb = parseFeedbackFrame(frame, cfg.motor_params);
        if (fb.valid) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          double position = fb.position - cfg.angle_offset;
          double dt = std::chrono::duration<double>(now - states_[i].last_feedback_time).count();
          const double jump = std::abs(position - states_[i].position);
          const double allowed_jump = cfg.motor_params.v_max * std::max(0.0, dt) + 0.15;
          const bool encoder_jump = states_[i].feedback_valid && dt >= 0 &&
            dt <= feedback_timeout_sec_ && jump > allowed_jump;
          if (fb.error || encoder_jump) {
            std::cerr << "[RobStrideHardwareManager] FEEDBACK_FAULT " << cfg.name
                      << " bus=" << bus_name << " id=" << static_cast<int>(cfg.can_id)
                      << " motor_error_bits=0x" << std::hex << ((raw_id >> 16) & 0x3F)
                      << std::dec << " encoder_jump=" << encoder_jump
                      << " previous_valid=" << states_[i].feedback_valid
                      << " previous_rad=" << states_[i].position
                      << " current_rad=" << position << " delta_rad=" << jump
                      << " allowed_rad=" << allowed_jump << " dt_s=" << dt
                      << " velocity_rad_s=" << fb.velocity
                      << " torque_nm=" << fb.torque << " temperature_c=" << fb.temperature
                      << std::endl;
            states_[i].feedback_valid = false;
            if (!startup_clearing_) triggerEmergencyStop("Invalid encoder or motor fault on " + cfg.name);
            break;
          }
          states_[i].position = position;
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
    return getAllJointStates()[joint_index];
  }

  std::vector<JointStateData> getAllJointStates()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto result = states_;
    auto now = std::chrono::steady_clock::now();
    for (auto & state : result) {
      state.feedback_valid = state.feedback_valid &&
        std::chrono::duration<double>(now - state.last_feedback_time).count() <= feedback_timeout_sec_;
      if (!state.feedback_valid) {
        state.position = state.velocity = state.effort = state.temperature =
          std::numeric_limits<double>::quiet_NaN();
      }
    }
    return result;
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
  void startFaultReset(const std::chrono::steady_clock::time_point & now)
  {
    is_passive_mode_ = true;
    safe_park_requested_ = false;
    safe_park_active_ = false;
    soft_fault_since_ = {};
    for (auto & cmd : commands_) cmd = {};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      for (auto & state : states_) state.feedback_valid = false;
    }
    for (const auto & cfg : joint_configs_) {
      auto * bus = cfg.bus_name == "can0" ? bus_can0_.get() : bus_can1_.get();
      if (!bus || !bus->sendFrame(buildStopMotorFrame(cfg.can_id, 0xFE, true))) {
        std::cerr << "[RobStrideHardwareManager] Fault reset failed: CAN write on " << cfg.name << std::endl;
        disableAllMotors();
        return;
      }
    }
    reset_started_ = now;
    reset_enable_sent_ = false;
    reset_in_progress_ = true;
    std::cerr << "[RobStrideHardwareManager] Fault reset requested; waiting for fresh fault-free feedback from all motors" << std::endl;
  }

  void verifyFaultReset(const std::chrono::steady_clock::time_point & now)
  {
    if (now - reset_started_ > std::chrono::seconds(2)) {
      reset_in_progress_ = false;
      std::cerr << "[RobStrideHardwareManager] Fault reset failed: feedback timeout; motors remain stopped" << std::endl;
      return;
    }
    if (!reset_enable_sent_) {
      if (now - reset_started_ < std::chrono::milliseconds(50)) return;
      for (const auto & cfg : joint_configs_) {
        auto * bus = cfg.bus_name == "can0" ? bus_can0_.get() : bus_can1_.get();
        if (!bus || !bus->sendFrame(buildMitControlFrame(cfg.can_id, 0, 0, 0, 0, 0, cfg.motor_params)) ||
            !bus->sendFrame(buildEnableMotorFrame(cfg.can_id))) {
          triggerEmergencyStop("Fault reset CAN write failed on " + cfg.name);
          return;
        }
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (auto & state : states_) state.feedback_valid = false;
      }
      reset_enabled_at_ = now;
      reset_enable_sent_ = true;
      return;
    }
    if (now - reset_enabled_at_ < std::chrono::milliseconds(100)) return;
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < N_JOINTS; ++i) {
      const auto & state = states_[i];
      const auto & cfg = joint_configs_[i];
      if (!state.feedback_valid || state.last_feedback_time < reset_enabled_at_ ||
          now - state.last_feedback_time > std::chrono::milliseconds(250) ||
          !std::isfinite(state.position) ||
          state.position < cfg.pos_min - POSITION_LIMIT_TOLERANCE ||
          state.position > cfg.pos_max + POSITION_LIMIT_TOLERANCE) return;
    }
    reset_in_progress_ = false;
    last_command_time_ = now;
    emergency_stopped_ = false;
    std::cerr << "[RobStrideHardwareManager] Fault reset complete; motors passive, controller reset required before motion" << std::endl;
  }

  std::vector<JointConfig> joint_configs_;
  std::vector<JointCommand> commands_;
  std::vector<JointStateData> states_;

  std::unique_ptr<RobStrideCanBus> bus_can0_;
  std::unique_ptr<RobStrideCanBus> bus_can1_;

  std::mutex cmd_mutex_;
  std::mutex state_mutex_;

  std::atomic<bool> initialized_;
  std::atomic<bool> emergency_stopped_;
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> reset_in_progress_{false};
  bool startup_clearing_ = false;
  bool reset_enable_sent_ = false;
  std::chrono::steady_clock::time_point reset_started_{};
  std::chrono::steady_clock::time_point reset_enabled_at_{};
  std::atomic<bool> is_passive_mode_;

  std::atomic<bool> safe_park_requested_{false};
  bool safe_park_active_ = false;
  std::array<double, N_JOINTS> park_hold_pos_{};
  std::array<double, N_JOINTS> park_initial_extra_{};
  std::chrono::steady_clock::time_point soft_fault_since_{};
  std::chrono::steady_clock::time_point park_request_time_{};
  std::chrono::steady_clock::time_point safe_park_started_{};
  double watchdog_timeout_sec_;
  const double feedback_timeout_sec_ = 0.25;
  std::chrono::steady_clock::time_point last_command_time_;
};

}  // namespace robstride
