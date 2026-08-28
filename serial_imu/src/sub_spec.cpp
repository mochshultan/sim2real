#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <iomanip>
#include <iostream>
#include <memory>

namespace serial_imu
{

class IMUSubscriber : public rclcpp::Node
{
public:
  IMUSubscriber()
  : Node("imu_sub")
  {
    this->declare_parameter<std::string>("topic_name", "/Imu_data");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      topic_name, 10,
      std::bind(&IMUSubscriber::topicCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "IMU Subscriber Node listening on '%s'.", topic_name.c_str());
  }

private:
  void topicCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::cout << "header:\n"
              << "  stamp:\n"
              << "    secs: " << msg->header.stamp.sec << "\n"
              << "    nanosecs: " << msg->header.stamp.nanosec << "\n"
              << "  frame_id: " << msg->header.frame_id << "\n"
              << "orientation:\n"
              << "  x: " << std::fixed << std::setprecision(6) << msg->orientation.x << "\n"
              << "  y: " << std::fixed << std::setprecision(6) << msg->orientation.y << "\n"
              << "  z: " << std::fixed << std::setprecision(6) << msg->orientation.z << "\n"
              << "  w: " << std::fixed << std::setprecision(6) << msg->orientation.w << "\n"
              << "angular_velocity:\n"
              << "  x: " << std::fixed << std::setprecision(6) << msg->angular_velocity.x << "\n"
              << "  y: " << std::fixed << std::setprecision(6) << msg->angular_velocity.y << "\n"
              << "  z: " << std::fixed << std::setprecision(6) << msg->angular_velocity.z << "\n"
              << "linear_acceleration:\n"
              << "  x: " << std::fixed << std::setprecision(6) << msg->linear_acceleration.x << "\n"
              << "  y: " << std::fixed << std::setprecision(6) << msg->linear_acceleration.y << "\n"
              << "  z: " << std::fixed << std::setprecision(6) << msg->linear_acceleration.z << "\n"
              << "---" << std::endl;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

}  // namespace serial_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<serial_imu::IMUSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
