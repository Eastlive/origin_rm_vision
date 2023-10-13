// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "std_msgs/msg/float64.hpp"
#include "armor_tracker/trajectory_slover.hpp"
namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  // 构造函数
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  inline double deg2rad(double deg) {return deg / 180.0 * M_PI;}
  inline double rad2deg(double rad) {return (rad * 180.0) / M_PI;}

  /**
   * @brief 计算相一个3D点相对于坐标原点的偏航和俯仰角
   * @param point_cam 相对于相机坐标系的3D点
   * @return 2D向量，偏航角yaw和俯仰角pitch
   */
  Eigen::Vector2d calcYawAndPitch(const Eigen::Vector3d & point_cam)
  {
    Eigen::Vector2d offset_angle;
    auto offset_yaw = atan2(point_cam[1], point_cam[0]);
    auto offset_pitch =
      -(atan2(point_cam[2], sqrt(point_cam[0] * point_cam[0] + point_cam[1] * point_cam[1])));
    offset_angle << offset_yaw, offset_pitch;
    return offset_angle;
  }

  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  /**
   * @brief 根据下位机的子弹速度，设置弹道解算器的子弹速度
   * @param bulletspeed 下位机的子弹速度
   */
  void setBulletSpeed(const std_msgs::msg::Float64::SharedPtr bulletspeed);

  /**
   * @brief 设置延迟
   * @param latency 订阅器接收延迟
   */
  void setLatancy(const std_msgs::msg::Float64::SharedPtr latency);

  // Maximum allowable armor distance in the XOY plane
  // 最大击打距离
  double max_armor_distance_;

  // The time when the last message was received
  // 上一次接收到消息的时间
  rclcpp::Time last_time_;
  // 上一次时间到现在的时间差
  double dt_;
  double latency_ = 0.0;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker pred_armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  //gimbal yaw
  double gimbal_yaw;

  //static offset
  double static_offset_yaw_;
  double static_offset_pitch_;

  //angle thres
  double yaw_angle_thres;
  double fire_permit_thres;
  double fire_latency;

  //speed thres
  double min_speed;

  //car_center_angle
  // 记录机器人转向目标的yaw角度之差
  double car_center_diff;

  //trajectory_slover
  std::shared_ptr<TrajectorySlover> trajectory_slover_;

  //bullet speed
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bullet_speed_sub_;

  //Subscriber latency
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr latency_sub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
