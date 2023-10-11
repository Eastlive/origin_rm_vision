// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
/// @brief 装甲板检测节点的构造函数
/// @param options 节点选项
/// @details 初始化信息有
/// 1. 节点名称，为armor_detector
/// 2. 装甲板检测器
/// 3. 装甲板检测结果发布器
/// 4. 装甲板可视化标记发布器
/// 5. 颜色信息
/// 6. Debug信息
/// 7. 相机参数
///  7.1 相机内参
///  7.2 相机畸变参数
///  7.3 PnP解算器
/// 8. 图像订阅器
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)
{
  // 日志输出，表示节点启动
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // Detector
  // 初始化装甲板检测器
  detector_ = initDetector();

  // Armors Publisher
  // 创建ros2的发布器，发布装甲板信息，话题名为/detector/armors
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors"; // 命名空间
  armor_marker_.action = visualization_msgs::msg::Marker::ADD; // 动作
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE; // 类型
  armor_marker_.scale.x = 0.05; // 尺寸
  armor_marker_.scale.z = 0.125; // 尺寸
  armor_marker_.color.a = 1.0; // 颜色
  armor_marker_.color.g = 0.5; // 颜色
  armor_marker_.color.b = 1.0; // 颜色
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1); // 生命周期

  text_marker_.ns = "classification"; // 命名空间
  text_marker_.action = visualization_msgs::msg::Marker::ADD; // 动作
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // 类型
  text_marker_.scale.z = 0.1; // 尺寸
  text_marker_.color.a = 1.0; // 颜色
  text_marker_.color.r = 1.0; // 颜色r
  text_marker_.color.g = 1.0; // 颜色g
  text_marker_.color.b = 1.0; // 颜色b
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1); // 生命周期

  // 创建ros2的发布器，发布可视化标记信息，话题名为/detector/marker
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // 创建ros2的订阅器，订阅颜色信息，并在回调函数中更新颜色信息，话题名为/color
  // 在回调函数中，如果连续5次接收到的颜色信息都是红色，则将detect_color设置为0，即红色
  // 如果连续5次接收到的颜色信息都是蓝色，则将detect_color设置为1，即蓝色
  color_sub_ = this->create_subscription<std_msgs::msg::Char>(
    "/color", 10,
    [this](std_msgs::msg::Char::ConstSharedPtr color)
    {
      if (color->data == 'B') {
        count_red_ = 0;
        count_blue_++;
        if (count_blue_ >= 5) {
          this->detector_->detect_color = 1;
          //RCLCPP_INFO(this->get_logger(),"change to aim blue");
        }
      } else if (color->data == 'R') {
        count_blue_ = 0;
        count_red_++;
        if (count_red_ >= 5) {
          this->detector_->detect_color = 0;
          //RCLCPP_INFO(this->get_logger(),"change to aim red");
        }
      }
    }
  );
  // Debug Publishers
  // 获取debug参数，如果debug为true，则创建debug发布器，否则不创建
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    // 创建debug发布器
    createDebugPublishers();

    // Debug param change moniter
    // 创建ros2的参数事件处理器，用于监视debug参数的变化
    debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    debug_cb_handle_ =
      debug_param_sub_->add_parameter_callback(
      "debug", [this](const rclcpp::Parameter & p) {
        debug_ = p.as_bool();
        debug_ ? createDebugPublishers() : destroyDebugPublishers();
      });
  }
  // 相机参数订阅器
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_camera/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]); // 设置相机中心点
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info); // 保存相机参数
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d); // 使用该信息初始化PnP解算器
      cam_info_sub_.reset(); // 重置此订阅器，因为只需要读取一次相机信息
    });

  // 创建一个订阅器来接收原始图像，并将其传递给imageCallback函数处理
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/front_camera/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

}

/// @brief 图像回调函数，用于检测图像中的装甲板并估计它们的位姿
/// @param img_msg 图像消息
void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // 调用detectArmors函数对图像中的装甲板进行检测，返回一个包含检测到的装甲板的列表
  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr) { // 检查是否初始化了PnP求解器
    // 设置消息头部信息
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    // 清空装甲板信息和标记数组，以确保它们为当前图像消息新填充
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    // 重置标记id
    armor_marker_.id = 0;
    text_marker_.id = 0;

    // 临时变量，用于存储装甲板信息
    auto_aim_interfaces::msg::Armor armor_msg;
    // 遍历检测到的装甲板
    for (const auto & armor : armors) { // 对于每一个检测到的装甲板
      // 平移向量和旋转向量
      cv::Mat rvec, tvec;
      // 使用PnP求解器计算平移向量和旋转向量
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);
      if (success) { // 如果求解成功
        // Fill basic info
        // 填充基本信息
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)]; // 装甲板类型
        armor_msg.number = armor.number; // 装甲板编号

        // Fill pose
        // 填充位姿信息，计算装甲板的3D位姿
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1); 
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix
        // 旋转向量转换为旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion
        // 将3x3的旋转矩阵转换为四元数形式
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        // 旋转矩阵转换为四元数
        tf2_rotation_matrix.getRotation(tf2_q);
        // 四元数转换为消息类型
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center
        // 计算装甲板中心到图像中心的距离
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // Fill the markers
        // 填充用于可视化的标记信息
        armor_marker_.id++; // 标记id
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23; // 标记尺寸
        armor_marker_.pose = armor_msg.pose; // 标记位姿
        text_marker_.id++;// 标记id
        text_marker_.pose.position = armor_msg.pose.position; // 标记位姿
        text_marker_.pose.position.y -= 0.1; // 标记位姿
        text_marker_.text = armor.classfication_result; // 标记文本
        armors_msg_.armors.emplace_back(armor_msg); // 装甲板信息
        marker_array_.markers.emplace_back(armor_marker_); // 标记数组
        marker_array_.markers.emplace_back(text_marker_); // 标记数组
      } else {
        // 如果PnP失败，发出警告
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // Publishing detected armors
    // 发布信息
    // 发布检测到的装甲板信息
    armors_pub_->publish(armors_msg_);

    // Publishing marker
    //调用函数发布装甲板的可视化标记
    publishMarkers();
  }
}

/// @brief 初始化装甲板检测器
/// @return 装甲板检测器 
std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  // 参数描述与范围
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1); // 整数范围
  param_desc.integer_range[0].step = 1; // 步长
  param_desc.integer_range[0].from_value = 0; // 起始值
  param_desc.integer_range[0].to_value = 255; // 终止值
  // 获取二值化阈值参数，是一个整数，范围为0-255
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE"; // 参数描述，0-红色，1-蓝色
  param_desc.integer_range[0].from_value = 0; // 起始值
  param_desc.integer_range[0].to_value = 1; // 终止值
  // 获取颜色参数，是一个整数，范围为0-1，0表示红色，1表示蓝色
  auto detect_color = declare_parameter("detect_color", RED, param_desc);

  // 灯条参数声明
  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1), // 灯条最小比例
    .max_ratio = declare_parameter("light.max_ratio", 0.4), // 灯条最大比例
    .max_angle = declare_parameter("light.max_angle", 40.0)}; // 灯条最大角度

  // 装甲板参数声明
  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.8), // 装甲板最小灯条比例
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8), // 装甲板最小小灯条中心距离
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2), // 装甲板最大小灯条中心距离
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2), // 装甲板最小大灯条中心距离
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5), // 装甲板最大大灯条中心距离
    .max_angle = declare_parameter("armor.max_angle", 35.0)}; // 装甲板最大角度

  // 创建装甲板检测器
  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  // 初始化分类器
  // 获取模型路径和标签路径
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/mlp.onnx"; // 模型路径
  auto label_path = pkg_path + "/model/label.txt"; // 标签路径
  // 获取分类器阈值和忽略类别
  double threshold = this->declare_parameter("classifier_threshold", 0.7);  // 分类器阈值
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"}); // 忽略类别
  // 创建分类器
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  // 返回装甲板检测器
  return detector;
}

/// @brief 传入的图像消息中检测装甲板
/// @param img_msg 图像消息
/// @return 检测到的装甲板列表
std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  // 将ROS的图像消息转换为OpenCV的cv::Mat图像格式
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // Update params
  // 更新参数
  // 从ROS节点的参数中获取并更新detector_的binary_thres属性
  detector_->binary_thres = get_parameter("binary_thres").as_int(); // 二值化阈值
  //detector_->detect_color = get_parameter("detect_color").as_int();
  // 从ROS节点的参数中获取并更新detector_的classifier子对象的threshold属性
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double(); // 分类器置信度阈值

  // 装甲板检测，并获取检测到的装甲板列表
  auto armors = detector_->detect(img);

  // 计算从图像消息的时间戳到现在的延迟，并将其记录到日志中
  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info
  // 发布debug信息
  if (debug_) { // 如果开启了debug调试模式
    // 发布二值化图像
    binary_img_pub_.publish(
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate
    // 按x坐标对灯条和装甲板数据进行排序
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) {return l1.center_x < l2.center_x;});
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) {return a1.center_x < a2.center_x;});

    // 发布灯条调试信息和装甲板调试信息
    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    // 如果检测到了装甲板，获取所有装甲板上的数字的图像，并发布它
    if (!armors.empty()) {
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }

    // 将检测结果绘制到图像上
    detector_->drawResults(img);
    // Draw camera center
    // 绘制相机中心
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    // 绘制延迟
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    // 发布带有检测结果的图像
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  // 返回检测到的装甲板列表
  return armors;
}

/// @brief 创建debug发布器，发布信息为灯条、装甲板、二值化图像、数字图像、结果图像
void ArmorDetectorNode::createDebugPublishers()
{
  // 创建ros2的发布器，发布灯条信息，话题名为/detector/debug_lights
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  // 创建ros2的发布器，发布装甲板信息，话题名为/detector/debug_armors
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

  // 创建ros2的发布器，发布二值化图像，话题名为/detector/binary_img
  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  // 创建ros2的发布器，发布数字图像，话题名为/detector/number_img
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  // 创建ros2的发布器，发布结果图像，话题名为/detector/result_img
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

/// @brief 销毁debug发布器
void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

/// @brief 发布可视化标记
void ArmorDetectorNode::publishMarkers()
{
  // 发布可视化标记
  using Marker = visualization_msgs::msg::Marker;
  // 如果检测到了装甲板，将动作设置为ADD，否则设置为DELETE
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  // 将装甲板框的信息添加到标记数组中
  marker_array_.markers.emplace_back(armor_marker_);
  // 发布标记数组
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
