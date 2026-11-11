#include "jaguar_control/ros2_control_system.hpp"
#include "jaguar_control/cybergear_motor_config.hpp"

#include <algorithm>
#include <set>
#include <pluginlib/class_list_macros.hpp>

namespace jaguar_control
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn JaguarSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  try {
    if (info_.joints.size() != JOINT_COUNT || info_.gpios.size() != 1 ||
      info_.gpios[0].name != "jaguar_safety") throw std::invalid_argument("Expected 12 joints and jaguar_safety GPIO");
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      const auto & joint = info_.joints[i];
      std::set<std::string> commands, states;
      for (const auto & iface : joint.command_interfaces) commands.insert(iface.name);
      for (const auto & iface : joint.state_interfaces) states.insert(iface.name);
      if (joint.name != JOINT_NAMES[i] ||
        commands != std::set<std::string>(COMMAND_NAMES.begin(), COMMAND_NAMES.end()) ||
        states != std::set<std::string>(STATE_NAMES.begin(), STATE_NAMES.end())) {
        throw std::invalid_argument("Joint order or interfaces do not match Jaguar impedance contract");
      }
    }
    mock_ = info_.hardware_parameters.at("mock_hardware") == "true";
    clear_faults_ = info_.hardware_parameters.at("startup_clear_faults") == "true";
    torque_limit_ = std::stod(info_.hardware_parameters.at("control_torque_limit_nm"));
    if (!hardware_.setControlTorqueLimit(torque_limit_)) throw std::invalid_argument("Torque cap must be in (0,14] Nm");
    hardware_.setMotorParams(robstride::loadCybergearRS00(info_.hardware_parameters.at("motor_params_file")));
    for (auto & state : states_) state[3] = 25.0;
    update_safety_state();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Jaguar hardware configuration: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JaguarSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn JaguarSystem::on_activate(const rclcpp_lifecycle::State &)
{
  commands_ = {};
  safety_commands_ = {};
  reset_sequence_ = 0.0;
  mock_stopped_ = false;
  if (!mock_ && (!hardware_.initializeBuses() || !hardware_.enableAndConfigureMotors(clear_faults_))) {
    hardware_.shutdown();
    return CallbackReturn::FAILURE;
  }
  active_ = true;
  update_safety_state();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JaguarSystem::stop()
{
  active_ = false;
  commands_ = {};
  hardware_.shutdown();
  return CallbackReturn::SUCCESS;
}
CallbackReturn JaguarSystem::on_deactivate(const rclcpp_lifecycle::State &) {return stop();}
CallbackReturn JaguarSystem::on_cleanup(const rclcpp_lifecycle::State &) {return stop();}
CallbackReturn JaguarSystem::on_shutdown(const rclcpp_lifecycle::State &) {return stop();}
CallbackReturn JaguarSystem::on_error(const rclcpp_lifecycle::State &) {return stop();}

std::vector<hardware_interface::StateInterface> JaguarSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    for (size_t j = 0; j < STATE_COUNT; ++j) result.emplace_back(JOINT_NAMES[i], STATE_NAMES[j], &states_[i][j]);
  }
  for (size_t j = 0; j < SAFETY_STATES.size(); ++j) result.emplace_back("jaguar_safety", SAFETY_STATES[j], &safety_states_[j]);
  return result;
}

std::vector<hardware_interface::CommandInterface> JaguarSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    for (size_t j = 0; j < COMMAND_COUNT; ++j) result.emplace_back(JOINT_NAMES[i], COMMAND_NAMES[j], &commands_[i][j]);
  }
  for (size_t j = 0; j < SAFETY_COMMANDS.size(); ++j) result.emplace_back("jaguar_safety", SAFETY_COMMANDS[j], &safety_commands_[j]);
  return result;
}

void JaguarSystem::update_safety_state()
{
  safety_states_[0] = mock_ ? mock_stopped_ : hardware_.isEmergencyStopped();
  safety_states_[1] = mock_ ? std::all_of(commands_.begin(), commands_.end(), [](const auto & command) {
      return command[2] == 0.0 && command[3] == 0.0 && command[4] == 0.0;
    }) : hardware_.isPassiveMode();
  safety_states_[2] = !mock_ && hardware_.safeParkRequested();
  safety_states_[3] = !mock_ && hardware_.isResetInProgress();
  safety_states_[4] = torque_limit_;
}

return_type JaguarSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!mock_ && active_) {
    hardware_.readIncomingFeedbacks();
    const auto states = hardware_.getAllJointStates();
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      states_[i] = {states[i].position, states[i].velocity, states[i].effort, states[i].temperature};
    }
  }
  update_safety_state();
  return return_type::OK;
}

return_type JaguarSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) return return_type::OK;
  if (safety_commands_[0] != 0.0) {
    if (mock_) mock_stopped_ = true;
    else hardware_.triggerEmergencyStop("CONTROLLER_REQUEST", "source=ros2_control");
  }
  if (safety_commands_[2] != reset_sequence_) {
    reset_sequence_ = safety_commands_[2];
    if (safety_commands_[0] == 0.0) {
      if (mock_) mock_stopped_ = false;
      else hardware_.requestFaultReset();
    }
  }
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    const auto & command = commands_[i];
    motor_commands_[i] = {command[0], command[1], command[2], command[3], command[4]};
  }
  if (!mock_) {
    hardware_.setSafeParkActive(safety_commands_[1] != 0.0);
    hardware_.setJointCommands(motor_commands_);
    hardware_.stepCommunicationCycle();
  } else if (!mock_stopped_) {
    // Offline contract check only: no dynamics, CAN, IMU, or torque simulation.
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      const auto & command = motor_commands_[i];
      if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
        !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
        !std::isfinite(command.effort) || command.kp < 0.0 || command.kd < 0.0) {
        mock_stopped_ = true;
        break;
      }
      if (command.kp != 0.0 || command.kd != 0.0 || command.effort != 0.0) {
        const auto & cfg = hardware_.getJointConfigs()[i];
        states_[i][0] = std::clamp(command.position, cfg.pos_min, cfg.pos_max);
        states_[i][1] = std::clamp(command.velocity, -cfg.max_vel, cfg.max_vel);
        states_[i][2] = std::clamp(command.effort, -torque_limit_, torque_limit_);
      }
    }
  }
  update_safety_state();
  // Keep a latched fault observable and manually resettable through the controller.
  return return_type::OK;
}
}  // namespace jaguar_control

PLUGINLIB_EXPORT_CLASS(jaguar_control::JaguarSystem, hardware_interface::SystemInterface)
