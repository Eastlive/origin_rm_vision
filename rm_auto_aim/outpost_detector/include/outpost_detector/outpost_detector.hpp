#ifndef OUTPOST_DETECTOR__OUTPOST_DETECTOR_NODE_HPP_
#define OUTPOST_DETECTOR__OUTPOST_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/armors.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

namespace rm_auto_aim
{

class OutpostDetectorNode : public rclcpp::Node
{
public:
  OutpostDetectorNode(const rclcpp::NodeOptions & options);

private:
  // Detected armors publisher
  bool publish_armors_[3];
  auto_aim_interfaces::msg::Armor armor_msg_[3];
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker direction_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  void publishMarkers();

  double outpost_x_;
  double outpost_y_;
  double outpost_z_;

  double orientation[4];

  void rotate(geometry_msgs::msg::Pose & pose, double angle);
  void revolve(geometry_msgs::msg::Pose & pose, double offset);

  const double radius = 0.2765;
  const double angle_speed = 0.8 * M_PI;
  double now_angle_;

  double last_time_;
  double dt_;
};

}  // namespace rm_auto_aim

#endif  // OUTPOST_DETECTOR__OUTPOST_DETECTOR_NODE_HPP_