#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstring>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif
#include "ch_serial.h"
#ifdef __cplusplus
}
#endif

namespace serial_imu
{

constexpr speed_t DEFAULT_BAUD = B115200;
constexpr double GRA_ACC = 9.80665;
constexpr double DEG_TO_RAD = 0.017453292519943295;
constexpr size_t BUF_SIZE = 1024;

class IMUPublisher : public rclcpp::Node
{
public:
  IMUPublisher()
  : Node("IMU_publisher"),
    fd_(-1)
  {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("frame_id", "imu_link");
    this->declare_parameter<std::string>("topic_name", "/Imu_data");

    port_name_ = this->get_parameter("port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();

    std::memset(&raw_, 0, sizeof(raw_));
    std::memset(buf_, 0, sizeof(buf_));

    fd_ = openSerialPort(port_name_);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port '%s': %s", port_name_.c_str(), std::strerror(errno));
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port '%s' opened successfully.", port_name_.c_str());
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name_, 20);

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(2ms, std::bind(&IMUPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "IMU Publisher Node initialized. Publishing to topic '%s' with frame '%s'.",
                topic_name_.c_str(), frame_id_.c_str());
  }

  ~IMUPublisher() override
  {
    closeSerialPort();
  }

private:
  int openSerialPort(const std::string & port)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      return -1;
    }

    struct termios options;
    std::memset(&options, 0, sizeof(options));

    if (tcgetattr(fd, &options) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed on '%s': %s", port.c_str(), std::strerror(errno));
      close(fd);
      return -1;
    }

    options.c_cflag = DEFAULT_BAUD | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed on '%s': %s", port.c_str(), std::strerror(errno));
      close(fd);
      return -1;
    }

    return fd;
  }

  void closeSerialPort()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
      RCLCPP_INFO(this->get_logger(), "Serial port closed.");
    }
  }

  void timerCallback()
  {
    if (fd_ < 0) {
      static auto last_retry = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_retry).count() >= 1) {
        last_retry = now;
        fd_ = openSerialPort(port_name_);
        if (fd_ >= 0) {
          RCLCPP_INFO(this->get_logger(), "Reconnected to serial port '%s'.", port_name_.c_str());
        }
      }
      return;
    }

    ssize_t n = read(fd_, buf_, sizeof(buf_));
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      closeSerialPort();
      std::memset(&raw_, 0, sizeof(raw_));
      return;
    }
    if (n <= 0) {
      return;
    }

    sensor_msgs::msg::Imu imu_data;

    for (ssize_t i = 0; i < n; ++i) {
      int rev = ch_serial_input(&raw_, buf_[i]);

      if (raw_.nitem_code > 0 && raw_.item_code[raw_.nitem_code - 1] != KItemGWSOL) {
        if (rev == 1 && raw_.nimu > 0) {
          const auto & imu_node = raw_.imu[raw_.nimu - 1];
          bool has_acc = false, has_gyr = false, has_quat = false;
          for (int j = 0; j < raw_.nitem_code; ++j) {
            const auto item = raw_.item_code[j];
            has_acc |= item == kItemAccRaw || item == KItemIMUSOL;
            has_gyr |= item == kItemGyrRaw || item == KItemIMUSOL;
            has_quat |= item == kItemRotationQuat || item == KItemIMUSOL;
          }
          double norm = 0;
          bool finite = true;
          for (float q : imu_node.quat) { finite &= std::isfinite(q); norm += q * q; }
          for (float a : imu_node.acc) finite &= std::isfinite(a);
          for (float g : imu_node.gyr) finite &= std::isfinite(g);
          if (!has_acc || !has_gyr || !has_quat || !finite || std::abs(norm - 1.0) > 0.1) continue;

          imu_data.orientation.w = imu_node.quat[0];
          imu_data.orientation.x = imu_node.quat[1];
          imu_data.orientation.y = imu_node.quat[2];
          imu_data.orientation.z = imu_node.quat[3];

          imu_data.angular_velocity.x = imu_node.gyr[0] * DEG_TO_RAD;
          imu_data.angular_velocity.y = imu_node.gyr[1] * DEG_TO_RAD;
          imu_data.angular_velocity.z = imu_node.gyr[2] * DEG_TO_RAD;

          imu_data.linear_acceleration.x = imu_node.acc[0] * GRA_ACC;
          imu_data.linear_acceleration.y = imu_node.acc[1] * GRA_ACC;
          imu_data.linear_acceleration.z = imu_node.acc[2] * GRA_ACC;

          imu_data.header.stamp = this->now();
          imu_data.header.frame_id = frame_id_;
          imu_pub_->publish(imu_data);
        }
      }
    }

    std::memset(buf_, 0, sizeof(buf_));
  }

  int fd_;
  std::string port_name_;
  std::string frame_id_;
  std::string topic_name_;
  uint8_t buf_[BUF_SIZE];
  raw_t raw_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace serial_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<serial_imu::IMUPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
