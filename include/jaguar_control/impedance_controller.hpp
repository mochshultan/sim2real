#pragma once

#include <atomic>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "jaguar_control/control_interfaces.hpp"

namespace jaguar_control
{
class ImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Command
  {
    std::array<std::array<double, COMMAND_COUNT>, JOINT_COUNT> values{};
    int64_t received_ns = 0;
    bool active = false;
  };
  void receive_command(const sensor_msgs::msg::JointState & message);
  void passive_commands();
  void publish_status(int64_t now);
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> park_active_{false};
  std::atomic<bool> observed_stopped_{false};
  std::atomic<bool> observed_resetting_{false};
  std::atomic<bool> enabled_{false};
  std::atomic<uint64_t> reset_sequence_{0};
  std::atomic<int64_t> accept_after_ns_{0};
  int64_t command_timeout_ns_ = 100000000;
  int64_t last_status_ns_ = 0;
  std::array<size_t, JOINT_COUNT * COMMAND_COUNT + 3> command_indices_{};
  std::array<size_t, 5 + JOINT_COUNT> state_indices_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_, park_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>> status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> park_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float32>> torque_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float32MultiArray>> temperature_pub_;
};
}  // namespace jaguar_control
