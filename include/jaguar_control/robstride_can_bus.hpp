#pragma once

#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <iostream>

namespace robstride
{

class RobStrideCanBus
{
public:
  explicit RobStrideCanBus(const std::string & interface_name)
  : interface_name_(interface_name), socket_fd_(-1), is_open_(false)
  {
  }

  ~RobStrideCanBus()
  {
    closeBus();
  }

  bool openBus()
  {
    if (is_open_) {
      return true;
    }

    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      std::cerr << "[RobStrideCanBus] Failed to open socket on " << interface_name_ << ": " << strerror(errno) << std::endl;
      return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      std::cerr << "[RobStrideCanBus] Failed to get interface index for " << interface_name_ << ": " << strerror(errno) << std::endl;
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // Enable non-blocking I/O for deterministic 200 Hz real-time operation
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    }

    // Disable loopback
    int loopback = 0;
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      std::cerr << "[RobStrideCanBus] Failed to bind socket on " << interface_name_ << ": " << strerror(errno) << std::endl;
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    is_open_ = true;
    return true;
  }

  void closeBus()
  {
    if (is_open_ && socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
      is_open_ = false;
    }
  }

  bool sendFrame(const struct can_frame & frame)
  {
    if (!is_open_ || socket_fd_ < 0) {
      return false;
    }

    ssize_t bytes_written = write(socket_fd_, &frame, sizeof(struct can_frame));
    return (bytes_written == static_cast<ssize_t>(sizeof(struct can_frame)));
  }

  bool receiveFrame(struct can_frame & frame)
  {
    if (!is_open_ || socket_fd_ < 0) {
      return false;
    }

    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(struct can_frame));
    if (bytes_read <= 0) {
      return false;
    }
    return (bytes_read == static_cast<ssize_t>(sizeof(struct can_frame)));
  }

  bool isOpen() const { return is_open_; }
  const std::string & getInterfaceName() const { return interface_name_; }

private:
  std::string interface_name_;
  int socket_fd_;
  bool is_open_;
};

}  // namespace robstride
