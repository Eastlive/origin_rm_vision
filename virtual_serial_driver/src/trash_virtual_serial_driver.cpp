// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "virtual_serial_driver/virtual_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

#include "tf2/LinearMath/Quaternion.h"

// C++ system
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace virtual_serial_driver
{
VirtualSerialDriver::VirtualSerialDriver(const rclcpp::NodeOptions & options)
: Node("virtual_serial_driver", options)                                     // node
{
  printf("aaaaaaaa");

  RCLCPP_INFO(get_logger(), "Start VirtualSerialDriver!");

  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

  enemy_color_pub_ = this->create_publisher<std_msgs::msg::Char>("/color", 10);

  bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/bullet_speed", 10);

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  mTimer =
    this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&VirtualSerialDriver::pubMsgCallback, this));

  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&VirtualSerialDriver::sendData, this, std::placeholders::_1));

  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
}

//测试3
void VirtualSerialDriver::pubMsgCallback()
{
  std_msgs::msg::Float64 bullet_speed;
  bullet_speed.data = (double)0.1;
  bullet_speed_pub_->publish(bullet_speed);

  sensor_msgs::msg::JointState joint_state;
  timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
  joint_state.header.stamp =
    this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  joint_state.name.push_back("pitch_joint");
  joint_state.name.push_back("yaw_joint");
  joint_state.position.push_back(deg2rad(0.1));
  joint_state.position.push_back(deg2rad(0.1));
  joint_state_pub_->publish(joint_state);

  std_msgs::msg::Char color;
  color.data = 82;
  enemy_color_pub_->publish(color);
}

void VirtualSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Recieve target data");
}

}  // namespace virtual_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(virtual_serial_driver::VirtualSerialDriver)
