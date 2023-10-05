// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/detector.hpp"
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include <std_msgs/msg/char.hpp>
namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
  // 装甲板检测节点的构造函数
  // 初始化信息有
  // 1. 节点名称
  // 2. 装甲板检测器
  // 3. 装甲板检测结果发布器
  // 4. 装甲板可视化标记发布器
  // 5. 颜色信息
  // 6. Debug信息
  // 7. 相机参数
  //  7.1 相机内参
  //  7.2 相机畸变参数
  //  7.3 PnP解算器
  // 8. 图像订阅器
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  /**
   * @brief 创建debug发布器，发布信息为灯条、装甲板、二值化图像、数字图像、结果图像
   */
  void createDebugPublishers();
  /**
   * @brief 销毁debug发布器
   */
  void destroyDebugPublishers();

  void publishMarkers();

  // Armor Detector
  // 装甲板检测器
  std::unique_ptr<Detector> detector_;

  // Detected armors publisher
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  // 可视化标记
  visualization_msgs::msg::Marker armor_marker_; // 用来表示装甲板的3D框
  visualization_msgs::msg::Marker text_marker_; // 用于显示分类结果的文本标记
  visualization_msgs::msg::MarkerArray marker_array_; // 标记数组
  // 可视化标记发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;


  //target detector color set
  rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr color_sub_;
  int count_red_{0};
  int count_blue_{0};
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
