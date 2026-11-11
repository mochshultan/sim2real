#include "jaguar_control/impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace jaguar_control
{
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

CallbackReturn ImpedanceController::on_init()
{
  auto_declare<double>("command_timeout", 0.1);
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ImpedanceController::command_interface_configuration() const
{
  InterfaceConfiguration result{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & joint : JOINT_NAMES) {
    for (const auto & interface : COMMAND_NAMES) result.names.push_back(joint + "/" + interface);
  }
  for (const auto & interface : SAFETY_COMMANDS) result.names.push_back("jaguar_safety/" + interface);
  return result;
}

InterfaceConfiguration ImpedanceController::state_interface_configuration() const
{
  InterfaceConfiguration result{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & interface : SAFETY_STATES) result.names.push_back("jaguar_safety/" + interface);
  for (const auto & joint : JOINT_NAMES) result.names.push_back(joint + "/temperature");
  return result;
}

CallbackReturn ImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  const double timeout = get_node()->get_parameter("command_timeout").as_double();
  if (!std::isfinite(timeout) || timeout <= 0.0) return CallbackReturn::ERROR;
  command_timeout_ns_ = static_cast<int64_t>(timeout * 1e9);
  command_buffer_.writeFromNonRT(Command{});
  command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_commands", rclcpp::QoS(1),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {receive_command(*msg);});
  stop_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/jaguar/emergency_stop", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {if (msg->data) stop_requested_ = true;});
  park_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    "/jaguar/safe_park_active", rclcpp::QoS(1).reliable(),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {park_active_ = msg->data;});
  reset_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/jaguar/reset_fault",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = enabled_ && observed_stopped_ && !observed_resetting_;
      if (response->success) {
        // Drop every pre-reset command; require new commands after recovery.
        accept_after_ns_ = steady_now_ns();
        stop_requested_ = false;
        park_active_ = false;
        observed_resetting_ = true;
        ++reset_sequence_;
      }
      response->message = response->success ?
        "Reset queued. Wait for PASSIVE_ZERO_TORQUE before resetting the policy controller." :
        "Reset rejected: impedance controller must be active, hardware stopped, and no reset pending.";
    });
  status_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(
    get_node()->create_publisher<std_msgs::msg::String>("/jaguar/hardware_status", 10));
  status_pub_->msg_.data.reserve(32);
  park_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    get_node()->create_publisher<std_msgs::msg::Bool>("/jaguar/hardware_safe_park", 10));
  torque_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>>(
    get_node()->create_publisher<std_msgs::msg::Float32>("/jaguar/control_torque_limit_nm", 10));
  temperature_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float32MultiArray>>(
    get_node()->create_publisher<std_msgs::msg::Float32MultiArray>("/jaguar/motor_diagnostics", 10));
  temperature_pub_->msg_.data.resize(JOINT_COUNT);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto command_names = command_interface_configuration().names;
  const auto state_names = state_interface_configuration().names;
  for (size_t i = 0; i < command_names.size(); ++i) {
    auto match = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & interface) {return interface.get_name() == command_names[i];});
    if (match == command_interfaces_.end()) return CallbackReturn::ERROR;
    command_indices_[i] = std::distance(command_interfaces_.begin(), match);
  }
  for (size_t i = 0; i < state_names.size(); ++i) {
    auto match = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & interface) {return interface.get_name() == state_names[i];});
    if (match == state_interfaces_.end()) return CallbackReturn::ERROR;
    state_indices_[i] = std::distance(state_interfaces_.begin(), match);
  }
  // Do not clear a hardware E-stop by cycling the controller lifecycle.
  accept_after_ns_ = steady_now_ns();
  park_active_ = false;
  last_status_ns_ = 0;
  enabled_ = true;
  passive_commands();
  return CallbackReturn::SUCCESS;
}

void ImpedanceController::passive_commands()
{
  for (size_t i = 0; i < JOINT_COUNT * COMMAND_COUNT; ++i) {
    (void)command_interfaces_[command_indices_[i]].set_value(0.0);
  }
}

CallbackReturn ImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  enabled_ = false;
  passive_commands();
  // Deactivation must stop CAN hardware too, rather than leave its last target live.
  (void)command_interfaces_[command_indices_[60]].set_value(1.0);
  (void)command_interfaces_[command_indices_[61]].set_value(0.0);
  return CallbackReturn::SUCCESS;
}

void ImpedanceController::receive_command(const sensor_msgs::msg::JointState & message)
{
  if (!enabled_) return;
  const auto invalid = [this] {
      stop_requested_ = true;
      RCLCPP_ERROR(get_node()->get_logger(), "event=ESTOP code=COMMAND_INVALID source=joint_commands");
    };
  if (message.position.size() != JOINT_COUNT ||
    (!message.name.empty() && message.name.size() != JOINT_COUNT) ||
    (!message.velocity.empty() && message.velocity.size() != JOINT_COUNT) ||
    (message.effort.size() != 24 && message.effort.size() != 36)) {invalid(); return;}
  Command command;
  std::array<bool, JOINT_COUNT> seen{};
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    size_t index = i;
    if (!message.name.empty()) {
      index = JOINT_COUNT;
      for (size_t j = 0; j < JOINT_COUNT; ++j) {
        if (message.name[i] == JOINT_NAMES[j] || message.name[i] == LEGACY_NAMES[j]) {index = j; break;}
      }
    }
    if (index == JOINT_COUNT || seen[index]) {invalid(); return;}
    seen[index] = true;
    auto & values = command.values[index];
    values = {message.position[i], message.velocity.empty() ? 0.0 : message.velocity[i],
      message.effort[i], message.effort[12 + i], message.effort.size() == 36 ? message.effort[24 + i] : 0.0};
    if (!std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);}) ||
      values[2] < 0.0 || values[3] < 0.0) {invalid(); return;}
    command.active |= values[2] != 0.0 || values[3] != 0.0 || values[4] != 0.0;
  }
  command.received_ns = steady_now_ns();
  command_buffer_.writeFromNonRT(command);
}

return_type ImpedanceController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const int64_t now = steady_now_ns();
  observed_stopped_ = state_interfaces_[state_indices_[0]].get_value() != 0.0;
  observed_resetting_ = state_interfaces_[state_indices_[3]].get_value() != 0.0;
  const auto * command = command_buffer_.readFromRT();
  const bool eligible = command->received_ns > accept_after_ns_.load();
  if (eligible && command->active && now - command->received_ns > command_timeout_ns_) {
    if (!stop_requested_.exchange(true)) {
      RCLCPP_ERROR(get_node()->get_logger(), "event=ESTOP code=COMMAND_TIMEOUT source=ros2_control");
    }
  }
  if (eligible && !stop_requested_ && !observed_stopped_ && !observed_resetting_) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      for (size_t j = 0; j < COMMAND_COUNT; ++j) {
        (void)command_interfaces_[command_indices_[i * COMMAND_COUNT + j]].set_value(command->values[i][j]);
      }
    }
  } else {
    passive_commands();
  }
  (void)command_interfaces_[command_indices_[60]].set_value(stop_requested_ ? 1.0 : 0.0);
  (void)command_interfaces_[command_indices_[61]].set_value(park_active_ ? 1.0 : 0.0);
  (void)command_interfaces_[command_indices_[62]].set_value(static_cast<double>(reset_sequence_.load()));
  if (park_pub_->trylock()) {
    park_pub_->msg_.data = state_interfaces_[state_indices_[2]].get_value() != 0.0;
    park_pub_->unlockAndPublish();
  }
  if (now - last_status_ns_ >= 20000000) {publish_status(now);}
  return return_type::OK;
}

void ImpedanceController::publish_status(int64_t now)
{
  last_status_ns_ = now;
  if (status_pub_->trylock()) {
    status_pub_->msg_.data = observed_resetting_ ? "RESETTING_FAULT" :
      observed_stopped_ ? "EMERGENCY_STOPPED" :
      state_interfaces_[state_indices_[1]].get_value() != 0.0 ? "PASSIVE_ZERO_TORQUE" : "ACTIVE_MIT_CONTROL";
    status_pub_->unlockAndPublish();
  }
  if (torque_pub_->trylock()) {
    torque_pub_->msg_.data = static_cast<float>(state_interfaces_[state_indices_[4]].get_value());
    torque_pub_->unlockAndPublish();
  }
  if (temperature_pub_->trylock()) {
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      temperature_pub_->msg_.data[i] = static_cast<float>(state_interfaces_[state_indices_[5 + i]].get_value());
    }
    temperature_pub_->unlockAndPublish();
  }
}
}  // namespace jaguar_control

PLUGINLIB_EXPORT_CLASS(jaguar_control::ImpedanceController, controller_interface::ControllerInterface)
