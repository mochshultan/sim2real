#pragma once

#include <hardware_interface/system_interface.hpp>
#include "jaguar_control/control_interfaces.hpp"
#include "jaguar_control/robstride_hardware_manager.hpp"

namespace jaguar_control
{
// One SystemInterface owns both CAN buses; controller_manager owns the cycle.
class JaguarSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  hardware_interface::CallbackReturn stop();
  void update_safety_state();
  robstride::RobStrideHardwareManager hardware_;
  std::array<std::array<double, COMMAND_COUNT>, JOINT_COUNT> commands_{};
  std::array<std::array<double, STATE_COUNT>, JOINT_COUNT> states_{};
  std::array<double, 3> safety_commands_{};
  std::array<double, 5> safety_states_{};
  std::vector<robstride::JointCommand> motor_commands_{JOINT_COUNT};
  bool mock_ = false;
  bool active_ = false;
  bool mock_stopped_ = false;
  bool clear_faults_ = true;
  double torque_limit_ = 14.0;
  double reset_sequence_ = 0.0;
};
}  // namespace jaguar_control
