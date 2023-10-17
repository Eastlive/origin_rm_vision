// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef VIRTUAL_SERIAL_DRIVER__VIRTUAL_SERIAL_DRIVER_HPP_
#define VIRTUAL_SERIAL_DRIVER__VIRTUAL_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "std_msgs/msg/char.hpp"

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"

namespace virtual_serial_driver
{
#define M_PI 3.14159265358979323846

class VirtualSerialDriver : public rclcpp::Node
{
public:
  explicit VirtualSerialDriver(const rclcpp::NodeOptions & options);

  ~VirtualSerialDriver() {}

private:
  inline double deg2rad(float deg) {return deg / 180.0 * M_PI;}

  inline double rad2deg(float rad) {return rad / M_PI * 180.0;}

  void receiveData();

  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  //time offset
  double timestamp_offset_ = 0;
  //- 发布状态
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr enemy_color_pub_;

  //- 订阅
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  //发送弹速
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bullet_speed_pub_;

  //Publisher latency
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

  //- 测试1 通信timer发布消息
  rclcpp::TimerBase::SharedPtr mTimer;
  void pubMsgCallback();

};
}  // namespace virtual_serial_driver

#endif  // VIRTUAL_SERIAL_DRIVER__VIRTUAL_SERIAL_DRIVER_HPP_
