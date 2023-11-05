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
  auto_aim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker direction_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  void publishMarkers();

  double roll;
  double pitch;
  double yaw;
};

}  // namespace rm_auto_aim

#endif  // OUTPOST_DETECTOR__OUTPOST_DETECTOR_NODE_HPP_