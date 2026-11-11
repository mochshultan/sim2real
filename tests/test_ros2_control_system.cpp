#include <cassert>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "jaguar_control/ros2_control_system.hpp"

using namespace jaguar_control;
using hardware_interface::CallbackReturn;

hardware_interface::HardwareInfo info(const std::string & motor_file)
{
  hardware_interface::HardwareInfo result;
  result.name = "JaguarRS00";
  result.type = "system";
  result.hardware_parameters = {{"mock_hardware", "true"}, {"startup_clear_faults", "false"},
    {"control_torque_limit_nm", "3.0"}, {"motor_params_file", motor_file}};
  for (const auto & name : JOINT_NAMES) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    for (const auto & interface : COMMAND_NAMES) {
      hardware_interface::InterfaceInfo iface;
      iface.name = interface;
      joint.command_interfaces.push_back(iface);
    }
    for (const auto & interface : STATE_NAMES) {
      hardware_interface::InterfaceInfo iface;
      iface.name = interface;
      joint.state_interfaces.push_back(iface);
    }
    result.joints.push_back(joint);
  }
  hardware_interface::ComponentInfo gpio;
  gpio.name = "jaguar_safety";
  gpio.type = "gpio";
  for (const auto & interface : SAFETY_COMMANDS) {
    hardware_interface::InterfaceInfo iface;
    iface.name = interface;
    gpio.command_interfaces.push_back(iface);
  }
  for (const auto & interface : SAFETY_STATES) {
    hardware_interface::InterfaceInfo iface;
    iface.name = interface;
    gpio.state_interfaces.push_back(iface);
  }
  result.gpios.push_back(gpio);
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  assert(argc >= 2);
  auto valid = info(argv[1]);
  JaguarSystem system;
  assert(system.on_init(valid) == CallbackReturn::SUCCESS);
  const rclcpp_lifecycle::State lifecycle;
  assert(system.on_configure(lifecycle) == CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  assert(commands.size() == 63 && states.size() == 53);
  assert(system.on_activate(lifecycle) == CallbackReturn::SUCCESS);
  const auto write = [&] {
      system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005));
      system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005));
    };
  write();
  assert(states[49].get_value() == 1.0);  // Passive activation.
  (void)commands[0].set_value(0.2);
  (void)commands[2].set_value(45.0);
  (void)commands[4].set_value(20.0);
  write();
  assert(states[0].get_value() == 0.2);
  assert(states[2].get_value() == 3.0);  // Effort cap, without a fault.
  assert(states[48].get_value() == 0.0);
  (void)commands[60].set_value(1.0);
  write();
  assert(states[48].get_value() == 1.0);
  (void)commands[60].set_value(0.0);
  (void)commands[0].set_value(0.3);
  write();
  assert(states[48].get_value() == 1.0 && states[0].get_value() == 0.2);
  (void)commands[62].set_value(1.0);
  (void)commands[2].set_value(0.0);
  (void)commands[4].set_value(0.0);
  write();
  assert(states[48].get_value() == 0.0 && states[49].get_value() == 1.0);
  assert(system.on_deactivate(lifecycle) == CallbackReturn::SUCCESS);
  assert(system.on_activate(lifecycle) == CallbackReturn::SUCCESS);
  write();
  assert(states[49].get_value() == 1.0);
  system.on_shutdown(lifecycle);
  for (const auto invalid : {"missing_joint", "wrong_name", "missing_gain", "bad_cap"}) {
    auto broken = valid;
    if (std::string(invalid) == "missing_joint") broken.joints.pop_back();
    if (std::string(invalid) == "wrong_name") broken.joints[0].name = "wrong";
    if (std::string(invalid) == "missing_gain") broken.joints[0].command_interfaces.pop_back();
    if (std::string(invalid) == "bad_cap") broken.hardware_parameters["control_torque_limit_nm"] = "14.1";
    JaguarSystem rejected;
    assert(rejected.on_init(broken) == CallbackReturn::ERROR);
  }
  rclcpp::shutdown();
}
