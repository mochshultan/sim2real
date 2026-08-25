#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <atomic>

#include "jaguar_control/robstride_hardware_manager.hpp"

namespace robstride
{

class RobStrideCanNode : public rclcpp::Node
{
public:
  RobStrideCanNode()
  : Node("robstride_can_hardware"),
    running_(true),
    hw_manager_(),
    loop_hz_(200),
    default_kp_(25.0),
    default_kd_(1.5)
  {
    RCLCPP_INFO(this->get_logger(), "=================================================");
    RCLCPP_INFO(this->get_logger(), " Starting RobStride RS00 Hard Real-Time CAN Node ");
    RCLCPP_INFO(this->get_logger(), "=================================================");

    // Declare ROS parameters
    this->declare_parameter<int>("rate_hz", 200);
    this->declare_parameter<double>("default_kp", 25.0);
    this->declare_parameter<double>("default_kd", 1.5);
    this->declare_parameter<int>("rt_priority", 80);

    loop_hz_ = this->get_parameter("rate_hz").as_int();
    default_kp_ = this->get_parameter("default_kp").as_double();
    default_kd_ = this->get_parameter("default_kd").as_double();
    rt_priority_ = this->get_parameter("rt_priority").as_int();

    buildNameMapping();

    // Publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    diag_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/jaguar/motor_diagnostics", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/jaguar/hardware_status", 10);

    // Subscribers
    joint_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_commands", 10,
      std::bind(&RobStrideCanNode::onJointCommand, this, std::placeholders::_1)
    );

    mit_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/mit_controller/commands", 10,
      std::bind(&RobStrideCanNode::onMitCommand, this, std::placeholders::_1)
    );

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/jaguar/emergency_stop", 10,
      std::bind(&RobStrideCanNode::onEmergencyStop, this, std::placeholders::_1)
    );

    // Initialize CAN Hardware
    if (!hw_manager_.initializeBuses()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN buses! Exiting...");
      rclcpp::shutdown();
      return;
    }

    if (!hw_manager_.enableAndConfigureMotors()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure RobStride motors!");
    }

    // Diagnostics Timer (1 Hz)
    diag_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RobStrideCanNode::publishDiagnostics, this)
    );

    // Start Dedicated Real-Time Thread
    rt_thread_ = std::thread(&RobStrideCanNode::realtimeControlLoop, this);

    RCLCPP_INFO(this->get_logger(), "RobStride RS00 Driver Running deterministically at %d Hz.", loop_hz_);
  }

  ~RobStrideCanNode() override
  {
    stop();
  }

  void stop()
  {
    if (running_) {
      running_ = false;
      if (rt_thread_.joinable()) {
        rt_thread_.join();
      }
      hw_manager_.shutdown();
      RCLCPP_INFO(this->get_logger(), "RobStride CAN Hardware Node safely stopped.");
    }
  }

private:
  void buildNameMapping()
  {
    // ROS CAN order: BL (0..2), BR (3..5), FL (6..8), FR (9..11)
    name_to_index_ = {
      // ROS CAN hardware naming
      {"BL_collar_joint", 0}, {"BL_hip_joint", 1}, {"BL_knee_joint", 2},
      {"BR_collar_joint", 3}, {"BR_hip_joint", 4}, {"BR_knee_joint", 5},
      {"FL_collar_joint", 6}, {"FL_hip_joint", 7}, {"FL_knee_joint", 8},
      {"FR_collar_joint", 9}, {"FR_hip_joint", 10}, {"FR_knee_joint", 11},
      // Isaac Lab naming
      {"Bl_roll_joint", 0}, {"Bl_hip_pitch_joint", 1}, {"Bl_knee_joint", 2},
      {"Br_roll_joint", 3}, {"Br_hip_pitch_joint", 4}, {"Br_knee_joint", 5},
      {"Fl_roll_joint", 6}, {"Fl_hip_pitch_joint", 7}, {"Fl_knee_joint", 8},
      {"Fr_roll_joint", 9}, {"Fr_hip_pitch_joint", 10}, {"Fr_knee_joint", 11}
    };
  }

  void setRealtimePriority()
  {
    struct sched_param param;
    param.sched_priority = rt_priority_;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
      RCLCPP_WARN(this->get_logger(), "Could not set SCHED_FIFO realtime priority (requires root/CAP_SYS_NICE). Running with standard POSIX scheduling.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Hard Real-Time SCHED_FIFO priority set to %d.", rt_priority_);
    }
  }

  void onJointCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool has_custom_gains = (msg->effort.size() >= 24);
    if (msg->name.size() > 0) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = name_to_index_.find(msg->name[i]);
        if (it != name_to_index_.end()) {
          size_t idx = it->second;
          JointCommand cmd;
          cmd.position = (msg->position.size() > i) ? msg->position[i] : 0.0;
          cmd.velocity = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
          if (has_custom_gains) {
            cmd.kp = msg->effort[i];
            cmd.kd = msg->effort[12 + i];
            cmd.effort = (msg->effort.size() >= 36) ? msg->effort[24 + i] : 0.0;
          } else {
            cmd.effort = (msg->effort.size() > i) ? msg->effort[i] : 0.0;
            cmd.kp = default_kp_;
            cmd.kd = default_kd_;
          }
          hw_manager_.setJointCommand(idx, cmd);
        }
      }
    } else if (msg->position.size() == N_JOINTS) {
      for (size_t i = 0; i < N_JOINTS; ++i) {
        JointCommand cmd;
        cmd.position = msg->position[i];
        cmd.velocity = (msg->velocity.size() == N_JOINTS) ? msg->velocity[i] : 0.0;
        if (has_custom_gains) {
          cmd.kp = msg->effort[i];
          cmd.kd = msg->effort[12 + i];
          cmd.effort = (msg->effort.size() >= 36) ? msg->effort[24 + i] : 0.0;
        } else {
          cmd.effort = (msg->effort.size() == N_JOINTS) ? msg->effort[i] : 0.0;
          cmd.kp = default_kp_;
          cmd.kd = default_kd_;
        }
        hw_manager_.setJointCommand(i, cmd);
      }
    }
  }

  void onMitCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Format: 60 elements -> [12 pos, 12 vel, 12 kp, 12 kd, 12 effort]
    // Or 36 elements -> [12 pos, 12 vel, 12 effort] with default Kp/Kd
    if (msg->data.size() == 60) {
      for (size_t i = 0; i < N_JOINTS; ++i) {
        JointCommand cmd;
        cmd.position = msg->data[i];
        cmd.velocity = msg->data[12 + i];
        cmd.kp       = msg->data[24 + i];
        cmd.kd       = msg->data[36 + i];
        cmd.effort   = msg->data[48 + i];
        hw_manager_.setJointCommand(i, cmd);
      }
    } else if (msg->data.size() == 36) {
      for (size_t i = 0; i < N_JOINTS; ++i) {
        JointCommand cmd;
        cmd.position = msg->data[i];
        cmd.velocity = msg->data[12 + i];
        cmd.effort   = msg->data[24 + i];
        cmd.kp = default_kp_;
        cmd.kd = default_kd_;
        hw_manager_.setJointCommand(i, cmd);
      }
    }
  }

  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      hw_manager_.triggerEmergencyStop("Manual E-Stop Topic Triggered");
    }
  }

  void realtimeControlLoop()
  {
    setRealtimePriority();

    struct timespec next_period;
    clock_gettime(CLOCK_MONOTONIC, &next_period);

    const long period_ns = static_cast<long>(1e9 / loop_hz_);

    sensor_msgs::msg::JointState js_msg;
    js_msg.name.resize(N_JOINTS);
    js_msg.position.resize(N_JOINTS);
    js_msg.velocity.resize(N_JOINTS);
    js_msg.effort.resize(N_JOINTS);

    const auto & configs = hw_manager_.getJointConfigs();
    for (size_t i = 0; i < N_JOINTS; ++i) {
      js_msg.name[i] = configs[i].name;
    }

    while (running_ && rclcpp::ok()) {
      // 1. Step CAN communication
      hw_manager_.stepCommunicationCycle();

      // 2. Publish Joint States
      auto states = hw_manager_.getAllJointStates();
      js_msg.header.stamp = this->get_clock()->now();
      for (size_t i = 0; i < N_JOINTS; ++i) {
        js_msg.position[i] = states[i].position;
        js_msg.velocity[i] = states[i].velocity;
        js_msg.effort[i]   = states[i].effort;
      }
      joint_state_pub_->publish(js_msg);

      // 3. Deterministic Sleep & Deadline Overrun Watchdog
      next_period.tv_nsec += period_ns;
      while (next_period.tv_nsec >= 1000000000) {
        next_period.tv_nsec -= 1000000000;
        next_period.tv_sec += 1;
      }

      struct timespec now_ts;
      clock_gettime(CLOCK_MONOTONIC, &now_ts);
      long diff_ns = (now_ts.tv_sec - next_period.tv_sec) * 1000000000L + (now_ts.tv_nsec - next_period.tv_nsec);

      if (diff_ns > 0) {
        // Deadline missed! Cycle took longer than period_ns (5 ms for 200 Hz)
        overrun_count_++;
        next_period = now_ts;  // Catch up to current time to avoid burst catch-ups
      } else {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, nullptr);
      }
    }
  }

  void publishDiagnostics()
  {
    auto states = hw_manager_.getAllJointStates();
    std_msgs::msg::Float32MultiArray diag_msg;
    diag_msg.data.resize(N_JOINTS);
    for (size_t i = 0; i < N_JOINTS; ++i) {
      diag_msg.data[i] = static_cast<float>(states[i].temperature);
    }
    diag_pub_->publish(diag_msg);

    if (overrun_count_ > 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "⚠️ [CAN Real-Time Overrun] Loop missed 200 Hz deadline %lu times! Check CPU load.", overrun_count_.load());
    }

    std_msgs::msg::String status_msg;
    if (hw_manager_.isEmergencyStopped()) {
      status_msg.data = "EMERGENCY_STOPPED";
    } else if (hw_manager_.isPassiveMode()) {
      status_msg.data = "PASSIVE_ZERO_TORQUE";
    } else {
      status_msg.data = "ACTIVE_MIT_CONTROL";
    }
    status_pub_->publish(status_msg);
  }

  std::atomic<bool> running_;
  std::atomic<uint64_t> overrun_count_{0};
  RobStrideHardwareManager hw_manager_;
  int loop_hz_;
  double default_kp_;
  double default_kd_;
  int rt_priority_;

  std::unordered_map<std::string, size_t> name_to_index_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mit_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  rclcpp::TimerBase::SharedPtr diag_timer_;
  std::thread rt_thread_;
};

}  // namespace robstride

static std::shared_ptr<robstride::RobStrideCanNode> g_node = nullptr;

void sigint_handler(int sig)
{
  (void)sig;
  if (g_node) {
    g_node->stop();
  }
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, sigint_handler);
  signal(SIGTERM, sigint_handler);

  g_node = std::make_shared<robstride::RobStrideCanNode>();
  rclcpp::spin(g_node);
  g_node->stop();
  rclcpp::shutdown();
  return 0;
}
