// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>

#include "std_msgs/msg/char.hpp"

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_serial_driver
{
#define M_PI 3.14159265358979323846

class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:

  // 角度转弧度
  inline double deg2rad(float deg) {return deg / 180.0 * M_PI;}

  // 弧度转角度
  inline double rad2deg(float rad) {return rad / M_PI * 180.0;}

  // 获取参数
  void getParams();

  // 接收数据
  void receiveData();

  // 发送数据，该函数是tracker的回调函数，接收到目标装甲板消息后，将消息发送给下位机
  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  // 重启串口
  void reopenPort();

  // io互斥锁
  // 用来保护io_context_的互斥锁
  // 该互斥锁仅在serial_driver中使用
  // 功能是保证串口通信不会同时进行读写
  // 因为串口通信使用的是同一块内存，所以需要保证读写不会同时进行
  std::unique_ptr<IoContext> owned_ctx_;

  // 串口名称
  std::string device_name_;

  // serial driver配置参数和serialDriver
  
  // 用来配置串口的参数
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  // 创建ROS2标准的串口驱动类型，用来操作串口
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  //time offset
  double timestamp_offset_ = 0;
  // 云台角度发布器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  // 敌方颜色发布器
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr enemy_color_pub_;

  // 订阅目标装甲板消息
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  // 发送弹速
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bullet_speed_pub_;

  // 发送延迟
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;

  // Debug 模式开关
  bool debug_mode_ = false;
  // Debug 通信timer发布消息
  rclcpp::TimerBase::SharedPtr debug_mTimer_;
  void debugMsgCallback();

  // 串口通信线程
  // 为了异步地接收数据，这样主线程可以继续执行其他任务而不会被 I/O 操作（如从串口读取数据）所阻塞
  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
