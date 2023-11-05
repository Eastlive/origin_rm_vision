#include "outpost_detector/outpost_detector.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry> 

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

namespace rm_auto_aim
{

OutpostDetectorNode::OutpostDetectorNode(const rclcpp::NodeOptions& options) : Node("outpost_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Outpost detector node started.");

  roll = declare_parameter("outpost_roll", 0.0);
  pitch = declare_parameter("outpost_pitch", 0.0);
  yaw = declare_parameter("outpost_yaw", 0.0);

  roll = roll / 180.0 * M_PI;
  pitch = pitch / 180.0 * M_PI;
  yaw = yaw / 180.0 * M_PI;

  RCLCPP_INFO(this->get_logger(), "outpost_roll: %f", roll);
  RCLCPP_INFO(this->get_logger(), "outpost_pitch: %f", pitch);
  RCLCPP_INFO(this->get_logger(), "outpost_yaw: %f", yaw);

  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("/detector/armors", 10);

  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.05;
  armor_marker_.scale.y = 0.235;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 0.0;
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  direction_marker_.ns = "direction";
  direction_marker_.action = visualization_msgs::msg::Marker::ADD;
  direction_marker_.type = visualization_msgs::msg::Marker::ARROW;
  direction_marker_.scale.x = 0.1;
  direction_marker_.scale.y = 0.01;
  direction_marker_.scale.z = 0.01;
  direction_marker_.color.a = 1.0;
  direction_marker_.color.r = 1.0;
  direction_marker_.color.g = 0.5;
  direction_marker_.color.b = 0.0;
  direction_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/markers", 10);

  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&OutpostDetectorNode::timerCallback, this));
}

void OutpostDetectorNode::timerCallback()
{
  // RCLCPP_INFO(this->get_logger(), "Start Timer Callback!");

  armors_msg_.header.frame_id = "camera_optical_link";
  armors_msg_.header.stamp = this->now();

  armors_msg_.armors.resize(2);
  armors_msg_.armors[0].number = "outpost";
  armors_msg_.armors[0].type = "small";
  armors_msg_.armors[0].distance_to_image_center = 1.0;
  armors_msg_.armors[0].pose.position.x = 0.0;
  armors_msg_.armors[0].pose.position.y = 0.0;
  armors_msg_.armors[0].pose.position.z = 1.0;
  roll = get_parameter("outpost_roll").as_double() / 180.0 * M_PI;
  pitch = get_parameter("outpost_pitch").as_double() / 180.0 * M_PI;
  yaw = get_parameter("outpost_yaw").as_double() / 180.0 * M_PI;
  RCLCPP_INFO(this->get_logger(), "outpost_roll: %f", roll);
  RCLCPP_INFO(this->get_logger(), "outpost_pitch: %f", pitch);
  RCLCPP_INFO(this->get_logger(), "outpost_yaw: %f", yaw);
  Eigen::Quaterniond quaternion = eulerToQuaternion(roll, pitch, yaw);
  armors_msg_.armors[0].pose.orientation.x = quaternion.x();
  armors_msg_.armors[0].pose.orientation.y = quaternion.y();
  armors_msg_.armors[0].pose.orientation.z = quaternion.z();
  armors_msg_.armors[0].pose.orientation.w = quaternion.w();

  armors_pub_->publish(armors_msg_);

  publishMarkers();
}

void OutpostDetectorNode::publishMarkers()
{
  marker_array_.markers.clear();

  armor_marker_.header = armors_msg_.header;
  armor_marker_.id = 0;
  armor_marker_.pose = armors_msg_.armors[0].pose;

  marker_array_.markers.emplace_back(armor_marker_);

  direction_marker_.header = armors_msg_.header;
  direction_marker_.id = 0;
  direction_marker_.pose = armors_msg_.armors[0].pose;

  marker_array_.markers.emplace_back(direction_marker_);

  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::OutpostDetectorNode)