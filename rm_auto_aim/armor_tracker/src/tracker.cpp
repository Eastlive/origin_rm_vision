// Copyright 2022 Chen Jun

#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>

namespace rm_auto_aim
{
// Tracker类的构造函数
// 初始化内容
// 1. 追踪状态为LOST
// 2. 追踪目标id为空
// 3. 测量量为4维
// 4. 目标状态量为9维
// 5. 最大匹配距离
// 6. 最大匹配偏航角差
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST),
  tracked_id(std::string("")),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(max_match_yaw_diff)
{
}

// 初始化追踪器，在追踪器为LOST状态时调用
void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  // 如果装甲板信息为空，不进行初始化
  if (armors_msg->armors.empty()) {
    return;
  }

  // 简单选择最接近图像中心的装甲板
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  // 初始化卡尔曼滤波器
  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  // 更新追踪目标id
  // 装甲板tracked_id为string类型
  tracked_id = tracked_armor.number;
  // 更新追踪状态
  tracker_state = DETECTING;

  // 更新目标装甲板数量，用于整车估计
  updateArmorsNum(tracked_armor);
}

// 更新追踪器，当追踪器不为LOST状态时调用
void Tracker::update(const Armors::SharedPtr & armors_msg)
{
  ///////////////////////////////
  //////////1.EKF预测////////////
  //////////////////////////////

  // 保存EKF预测结果
  Eigen::VectorXd ekf_prediction = ekf.predict();
  // 输出调试信息，表明进行EKF预测
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

  ///////////////////////////////
  //////2.匹配装甲板逻辑部分////////
  ///////////////////////////////

  // 该变量用于标记是否找到匹配的装甲板，初始值为false，即没有找到匹配的装甲板
  bool matched = false;
  // 如果没有找到匹配的装甲板，则使用EKF预测作为默认目标状态
  // target_state的状态会在后续的匹配装甲板逻辑中更新
  target_state = ekf_prediction;
  // 匹配装甲板
  // 如果装甲板信息不为空
  if (!armors_msg->armors.empty()) {
    // 查找与追踪目标id相同的装甲板
    Armor same_id_armor;
    int same_id_armors_count = 0;
    // 该变量用于存储预测的装甲板位置，此位置由EKF的预测值得到，该变量为3维向量
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    // 该变量用于存储最小距离差，初始值为最大值
    double min_position_diff = DBL_MAX;
    // 该变量用于存储偏航角差，初始值为最大值
    double yaw_diff = DBL_MAX;
    // 遍历装甲板信息
    for (const auto & armor : armors_msg->armors) {
      // Only consider armors with the same id
      // 如果装甲板id与追踪目标id相同
      if (armor.number == tracked_id) {
        // 将该装甲板存下，用于后续处理
        same_id_armor = armor;
        // 装甲板计数器加一
        same_id_armors_count++;
        // Calculate the difference between the predicted position and the current armor position
        // 计算预测的装甲板位置与当前装甲板位置的距离差
        auto p = armor.pose.position;
        // 将当前遍历到的装甲板位置存储为3维向量
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        // 计算预测的装甲板位置与当前检测到的装甲板位置的距离差
        double position_diff = (predicted_position - position_vec).norm();
        // 如果距离差小于最小距离差
        if (position_diff < min_position_diff) {
          // Find the closest armor
          // 更新最小距离差
          min_position_diff = position_diff;
          // 更新偏航角差
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          // 更新追踪装甲板
          tracked_armor = armor;
        }
      }
    }

    // Store tracker info
    info_position_diff = min_position_diff;
    info_yaw_diff = yaw_diff;

    // Check if the distance and yaw difference of closest armor are within the threshold
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      // Matched armor found
      matched = true;
      auto p = tracked_armor.pose.position;
      // Update EKF
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf.update(measurement);
      RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      // Matched armor not found, but there is only one armor with the same id
      // and yaw has jumped, take this case as the target is spinning and armor jumped
      handleArmorJump(same_id_armor);
    } else {
      // No matched armor found
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
    }
  }

  // Prevent radius from spreading
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf.setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf.setState(target_state);
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}

// 初始化扩展卡尔曼滤波器
void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz = 0, another_r = r;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

  ekf.setState(target_state);
}

// 更新目标装甲板数量，用于整车估计
void Tracker::updateArmorsNum(const Armor & armor)
{
  if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) {
    // 当目标为大装甲板，且追踪目标为3、4、5号装甲板时，目标拥有两块装甲板，判定为平衡步兵
    tracked_armors_num = ArmorsNum::BALANCE_2;
  } else if (tracked_id == "outpost") {
    // 当追踪目标为前哨站时，目标拥有三块装甲板
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else {
    // 其他情况，目标拥有四块装甲板
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);
  target_state(6) = yaw;
  updateArmorsNum(current_armor);
  // Only 4 armors has 2 radius and height
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    dz = target_state(4) - current_armor.pose.position.z;
    target_state(4) = current_armor.pose.position.z;
    std::swap(target_state(8), another_r);
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // If position difference is larger than max_match_distance_,
  // take this case as the ekf diverged, reset the state
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // za
    target_state(5) = 0;                   // vza
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  ekf.setState(target_state);
}

// 此函数用于从四元数方向中获取yaw，并确保yaw的连续性
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  // 将四元数转换为tf2中的四元数
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  // 将tf2中的四元数转换为yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  // 保证yaw的连续性
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  // 返回yaw
  return yaw;
}

// 此函数通过状态向量计算装甲板位置
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  // 对于给定的观测状态，计算预测的装甲板位置
  // x为9维向量，该步骤用来将x中的数据提取出来
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  // 计算装甲板位置
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  // 返回一个3维向量，表示装甲板位置
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace rm_auto_aim
