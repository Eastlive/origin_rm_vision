// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

#include "rm_serial_driver/rm_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

#include "tf2/LinearMath/Quaternion.h"

// C++ system
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),                                     // node
  owned_ctx_{new IoContext(2)},                                          // 初始化IoContext
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}  //- 初始化serialDriver
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  // 获取参数，such as : 波特率、奇偶校验、停止位、流控等等
  getParams();  //- params: name baud flowctrl parity stopbits

  //
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);

  // 创建敌方颜色发布器，发布敌方颜色，名称是/color，类型是std_msgs::msg::Char，队列长度是10
  enemy_color_pub_ = this->create_publisher<std_msgs::msg::Char>("/color", 10);

  // 创建子弹速度发布器，发布子弹速度，名称是/bullet_speed，类型是std_msgs::msg::Float64，队列长度是10
  bullet_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("/bullet_speed", 10);

  // 创建云台状态发布器，发布云台状态，名称是/joint_states，类型是sensor_msgs::msg::JointState，QoS是表示只保留最新的1个消息
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  // 串口驱动初始化及打开
  try {
    // 初始化串口，serial_driver_为标准的ROS2串口驱动类型
    serial_driver_->init_port(device_name_, *device_config_);
    // 判断串口是否打开
    if (!serial_driver_->port()->is_open()) {
      // 如果串口没有打开，就打开串口
      serial_driver_->port()->open();
      // 并且创建一个线程，用来接收数据
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      // 如果初始化失败，就抛出异常
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // 创建目标订阅器，订阅目标，名称是/tracker/target，用来接收ArmorTracker发布的装甲板信息
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  if(debug_mode_)
  {
    // 创建定时器，每10ms发布一次数据
    debug_mTimer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&RMSerialDriver::debugMsgCallback,this));
  }
  // 创建延迟发布器，发布延迟，名称是/latency，类型是std_msgs::msg::Float64，用来发布延迟信息
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
}

RMSerialDriver::~RMSerialDriver()
{
  // 关闭接收线程
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  // 关闭串口
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  // 等待所有任务完成
  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

//- 接受data数据
void RMSerialDriver::receiveData()
{
  // header用于接收针头，每次接收一个字节，判断是否为针头
  // header(1)中的数字1表示header向量的初始大小为1，即它初始化为包含一个字节的空间
  std::vector<uint8_t> header(1);
  // 对比std::vector<uint8_t> data(sizeof(ReceivePacket));和以下代码
  // std::vector<uint8_t> data(sizeof(ReceivePacket));：这直接初始化了一个向量，其大小为sizeof(ReceivePacket)，并将所有元素初始化为0。
  // 以下代码：这只是初始化了一个空向量，其大小为0，然后调用reserve()函数，将向量的容量设置为sizeof(ReceivePacket)，此时向量的大小仍为0。
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket) - 1);

  // 如果串口打开，就一直循环，接收数据
  while (rclcpp::ok()) {
    try {
      // 接收数据
      serial_driver_->port()->receive(header);

      // 判断数据头
      if (header[0] == 0x5A) {  //frame header
        // 接收数据，数据长度为sizeof(ReceivePacket) - 1
        data.resize(sizeof(ReceivePacket) - 1);
        // 接收数据
        serial_driver_->port()->receive(data);

        // 将数据头和数据合并
        data.insert(data.begin(), header[0]);
        // 将数据转换为ReceivePacket类型
        ReceivePacket packet = fromVector(data);

        // 判断CRC校验
        bool crc_ok =
          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        // 如果CRC校验通过
        if (crc_ok) {
          // 创建子弹速度消息
          std_msgs::msg::Float64 bullet_speed;
          // 将数据转换为double类型
          bullet_speed.data = (double)packet.bullet_speed;
          // 发布子弹速度
          bullet_speed_pub_->publish(bullet_speed);

          //发布pitch和yaw joint的位置
          sensor_msgs::msg::JointState joint_state;
          // 
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          // 接收到云台状态后，发布云台状态
          joint_state.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");
          joint_state.name.push_back("shoot_yaw_joint");
          joint_state.position.push_back(deg2rad(packet.pitch));
          joint_state.position.push_back(deg2rad(packet.yaw));
          joint_state.position.push_back(deg2rad(packet.pitch));
          //RCLCPP_INFO(this->get_logger(),"send color :  %c",packet.enemy_color);
          joint_state_pub_->publish(joint_state);

          // 发布敌方颜色
          std_msgs::msg::Char color;
          color.data = packet.enemy_color;
          enemy_color_pub_->publish(color);
        } else {
          // 如果CRC校验失败，打印错误信息
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        // 如果数据头不是0x5A，则反馈信息
        RCLCPP_WARN(get_logger(), "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      // 如果接收失败，就重新打开串口
      RCLCPP_ERROR(get_logger(), "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

// 发送数据函数，该函数作为接收到target信息的回调函数，接收到装甲板信息后，就会调用该函数
void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  // 发送数据
  // 当你在函数内部使用static变量时，该变量在函数首次调用时进行初始化。
  // 在之后的调用中，它将保持其上一次赋予的值，不会再次初始化。
  // static变量的使用在函数中常常用作计数器或其他需要在函数调用之间保持状态的目的。
  static uint32_t packet_id = 0;
  try {
    // 创建发送数据包
    SendPacket packet;
    // 针头
    packet.header = 0xA5;
    // 数据包ID，每发送一次数据，ID就会加1，这样就可以判断是否为新的数据包
    packet.packat_id = packet_id++;
    // 是否允许开火
    packet.suggest_fire = msg->fire_permit;
    // 偏航角
    packet.offset_yaw = msg->offset_yaw;
    // 俯仰角
    packet.offset_pitch = msg->offset_pitch;
    // CRC校验
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    // 将数据包转换为vector
    std::vector<uint8_t> data = toVector(packet);
    // 发送数据
    serial_driver_->port()->send(data);

    // 创建延迟消息
    // 延迟消息的作用是监控代码性能
    std_msgs::msg::Float64 latency;
    // 计算延迟
    // 延迟是从接收到装甲板信息，到发送数据的时间差
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    // 在终端打印延迟的信息
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    // 发布延迟
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    // 如果发送失败，就重新打开串口
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

//- 配置串口参数，such as : 波特率、奇偶校验、停止位、流控等等
void RMSerialDriver::getParams()
{
  // 获取debug模式，如果是true，就打印debug信息
  debug_mode_ = this->declare_parameter("debug_mode", false);
  RCLCPP_INFO(get_logger(), "Debug mode: %s", debug_mode_ ? "true" : "false");

  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  // 波特率
  uint32_t baud_rate{};
  // 流控
  auto fc = FlowControl::NONE;
  // 奇偶校验
  auto pt = Parity::NONE;
  // 停止位
  auto sb = StopBits::ONE;

  // 获取参数
  try {
    // 获取设备名称
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    // 如果获取失败，抛出异常
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    // 获取波特率
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    // 如果获取失败，抛出异常
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    // 获取流控
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    // 根据字符串，判断流控类型
    if (fc_string == "none") {
      // 如果是none，就是无流控
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      // 如果是hardware，就是硬件流控
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      // 如果是software，就是软件流控
      fc = FlowControl::SOFTWARE;
    } else {
      // 如果都不是，抛出异常
      throw std::invalid_argument{
              "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    // 如果获取失败，抛出异常
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    // 获取奇偶校验
    const auto pt_string = declare_parameter<std::string>("parity", "");

    // 根据字符串，判断奇偶校验类型
    if (pt_string == "none") {
      // 如果是none，就是无奇偶校验
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      // 如果是odd，就是奇校验
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      // 如果是even，就是偶校验
      pt = Parity::EVEN;
    } else {
      // 如果都不是，抛出异常
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    // 如果获取失败，抛出异常
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    // 获取停止位
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    // 根据字符串，判断停止位类型
    if (sb_string == "1" || sb_string == "1.0") {
      // 如果是1，就是1个停止位
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      // 如果是1.5，就是1.5个停止位
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      // 如果是2，就是2个停止位
      sb = StopBits::TWO;
    } else {
      // 如果都不是，抛出异常
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    // 如果获取失败，抛出异常
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  // 设备配置参数 = 波特率 + 流控 + 奇偶校验 + 停止位，这里使用了智能指针
  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

// 重启串口
void RMSerialDriver::reopenPort()
{
  // 打印警告信息
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  // 重启串口
  try {
    // 如果串口打开，就关闭串口
    if (serial_driver_->port()->is_open()) {
      // 关闭串口
      serial_driver_->port()->close();
    }
    // 重新打开串口
    serial_driver_->port()->open();
    // 打印成功信息
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    // 如果重启失败，就打印错误信息
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    // 判断ROS2是否还在运行
    if (rclcpp::ok()) {
      // 如果ROS2还在运行，就等待1s后，重新打开串口
      rclcpp::sleep_for(std::chrono::seconds(1));
      // 重新打开串口
      reopenPort();
    }
  }
}

// debug模式下，每10ms发布一次数据
void RMSerialDriver::debugMsgCallback()
{
  static uint32_t packet_id = 0;
  try {
    SendPacket packet;
    packet.header = 0xA5;
    packet.packat_id = packet_id++;
    packet.suggest_fire = 0;
    packet.offset_yaw = 0.1;
    packet.offset_pitch = 1.1;
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
