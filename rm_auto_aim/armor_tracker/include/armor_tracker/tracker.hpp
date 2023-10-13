// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

// 用于标记目标单位拥有的装甲板的数量
// NORMAL_4: 4个装甲板
// BALANCE_2: 2个装甲板，用于平衡车
// OUTPOST_3: 3个装甲板，用于前哨站
enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker
{
public:
  /**
   * @brief Tracker类的构造函数
   * @param max_match_distance 最大匹配距离
   * @param max_match_yaw_diff 最大匹配偏航角差
   */
  Tracker(double max_match_distance, double max_match_yaw_diff);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  /**
   * @brief 初始化追踪器，在追踪器为LOST状态时调用
   * @param armors_msg 所有检测到的装甲板信息，当装甲板信息为空时，不进行初始化
   */
  void init(const Armors::SharedPtr & armors_msg);

  /**
   * @brief 更新追踪器，在追踪器不为LOST状态时调用
   * @param armors_msg 所有检测到的装甲板信息，当装甲板信息为空时，不进行更新
   */
  void update(const Armors::SharedPtr & armors_msg);

  // 扩展卡尔曼滤波器
  ExtendedKalmanFilter ekf;

  // 用于标记追踪器的追踪阈值，超过该阈值则切换为跟踪状态
  int tracking_thres;
  // 用于标记追踪器的丢失阈值，超过该阈值则切换为丢失状态
  int lost_thres;

  // 用于标记追踪状态
  // LOST: 丢失目标
  // DETECTING: 正在检测目标
  // TRACKING: 正在追踪目标
  // TEMP_LOST: 暂时丢失目标
  // tracker_state为State类型
  enum State
  {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  // 用于标记目标装甲板的id信息
  // 该变量在整个tracker中生效
  std::string tracked_id;
  // 用于存储追踪的装甲板信息
  Armor tracked_armor;
  // 用于标记目标单位拥有的装甲板的数量
  ArmorsNum tracked_armors_num;

  // 用于标记预测的装甲板于最近的检测到的装甲板的距离差
  double info_position_diff;
  // 用于标记预测的装甲板于最近的检测到的装甲板的偏航角差
  double info_yaw_diff;

  // 用于存储EKF观测量，用于更新步
  Eigen::VectorXd measurement;

  // 用于存储EKF状态量，用于记录预测步
  Eigen::VectorXd target_state;

  // To store another pair of armors message
  // 用于存储另一组装甲板信息
  // 两组装甲板的高度差
  double dz;
  // 另一组装甲板的半径
  double another_r;

private:
  /**
   * @brief 初始化卡尔曼滤波器，在init函数中调用
   * @param a 装甲板信息，该装甲板为最接近图像中心的装甲板
   */
  void initEKF(const Armor & a);

  /**
   * @brief 更新目标装甲板数量，在init函数和handleArmorJump函数中调用
   * @param a 装甲板信息
   */
  void updateArmorsNum(const Armor & a);

  /**
   * @brief 处理装甲板跳变，在update函数中调用
   * @param a 装甲板信息
   */
  void handleArmorJump(const Armor & a);

  /**
   * @brief 此函数用于将四元数转换为偏航角
   * @param q 四元数
   * @return yaw值，偏航角
   */
  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  /**
   * @brief 此函数通过状态向量，计算装甲板的位置
   * @param x 状态向量，为9维向量
   * @return 装甲板的位置，为3维向量
   */
  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  // 最大匹配距离参数
  double max_match_distance_;
  // 最大匹配偏航角差参数
  double max_match_yaw_diff_;

  // 用于标记目标单位被检测的次数，超过阈值则切换为TRACKING状态
  int detect_count_;
  // 用于标记目标单位丢失的次数，超过阈值则切换为LOST状态
  int lost_count_;

  // 用于确保yaw的连续性，标记上一次的偏航角
  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
