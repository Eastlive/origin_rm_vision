// Copyright (c) 2023 Chen Tingxu
// Licensed under the MIT License.

#include "virtual_serial_driver/virtual_serial_driver.hpp"

#include "rclcpp/rclcpp.hpp"

namespace virtual_serial_driver
{
VirtualSerialDriver::VirtualSerialDriver(const rclcpp::NodeOptions & options)
: Node("virtual_serial_driver", options)
{
  RCLCPP_INFO(this->get_logger(), "Start virtual serial driver!");

  enemy_color_pub_ = this->create_publisher<std_msgs::msg::Char>("/color", 10);

  bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/bullet_speed", 10);

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&VirtualSerialDriver::timerCallback, this));
}

void VirtualSerialDriver::timerCallback()
{
  RCLCPP_INFO(this->get_logger(), "Start Timer Callback!");

  std_msgs::msg::Char color;
  color.data = 'R';
  enemy_color_pub_->publish(color);

  std_msgs::msg::Float64 bullet_speed;
  bullet_speed.data = 0.0;
  bullet_speed_pub_->publish(bullet_speed);

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = this->now();
  joint_state.name.push_back("pitch_joint");
  joint_state.name.push_back("yaw_joint");
  joint_state.position.push_back(0.1);
  joint_state.position.push_back(0.1);
  joint_state_pub_->publish(joint_state);
}

} // virtual_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(virtual_serial_driver::VirtualSerialDriver)
