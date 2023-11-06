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

  x = declare_parameter("outpost_x", 0.0);
  y = declare_parameter("outpost_y", 0.0);
  z = declare_parameter("outpost_z", 2.5);

  orientation[0] = declare_parameter("orientation_x", -0.6335811);
  orientation[1] = declare_parameter("orientation_y", 0.6335811);
  orientation[2] = declare_parameter("orientation_z", 0.0);
  orientation[3] = declare_parameter("orientation_w", 0.4440158);

  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

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

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

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

  armors_msg_.armors.resize(3);
  armors_msg_.armors[0].number = "outpost";
  armors_msg_.armors[0].type = "small";
  armors_msg_.armors[0].pose.position.x = get_parameter("outpost_x").as_double();
  armors_msg_.armors[0].pose.position.y = get_parameter("outpost_y").as_double();
  armors_msg_.armors[0].pose.position.z = get_parameter("outpost_z").as_double();
  armors_msg_.armors[0].distance_to_image_center = 600;
  armors_msg_.armors[0].pose.orientation.x = orientation[0];
  armors_msg_.armors[0].pose.orientation.y = orientation[1];
  armors_msg_.armors[0].pose.orientation.z = orientation[2];
  armors_msg_.armors[0].pose.orientation.w = orientation[3];

  armors_msg_.armors[1].number = "outpost";
  armors_msg_.armors[1].type = "small";
  armors_msg_.armors[1].pose.position.x = get_parameter("outpost_x").as_double();
  armors_msg_.armors[1].pose.position.y = get_parameter("outpost_y").as_double();
  armors_msg_.armors[1].pose.position.z = get_parameter("outpost_z").as_double();
  armors_msg_.armors[1].distance_to_image_center = 200;
  armors_msg_.armors[1].pose.orientation.x = 0.5;
  armors_msg_.armors[1].pose.orientation.y = -0.5;
  armors_msg_.armors[1].pose.orientation.z = 0.5;
  armors_msg_.armors[1].pose.orientation.w = 0.5;
  armors_pub_->publish(armors_msg_);

  armors_msg_.armors[2].number = "outpost";
  armors_msg_.armors[2].type = "small";
  armors_msg_.armors[2].pose.position.x = get_parameter("outpost_x").as_double();
  armors_msg_.armors[2].pose.position.y = get_parameter("outpost_y").as_double();
  armors_msg_.armors[2].pose.position.z = get_parameter("outpost_z").as_double();
  armors_msg_.armors[2].distance_to_image_center = 1000;
  armors_msg_.armors[2].pose.orientation.x = -0.130526192220051;
  armors_msg_.armors[2].pose.orientation.y = 0.0;
  armors_msg_.armors[2].pose.orientation.z = 0.0;
  armors_msg_.armors[2].pose.orientation.w = 0.99144486137381;

  publishMarkers();
}

void OutpostDetectorNode::publishMarkers()
{
  marker_array_.markers.clear();

  armor_marker_.header = armors_msg_.header;
  armor_marker_.id = 1;
  armor_marker_.pose = armors_msg_.armors[0].pose;

  marker_array_.markers.emplace_back(armor_marker_);

  armor_marker_.id = 2;
  armor_marker_.pose = armors_msg_.armors[1].pose;
  armor_marker_.color.r = 1.0;
  armor_marker_.color.g = 0.0;
  armor_marker_.color.b = 0.0;
  marker_array_.markers.emplace_back(armor_marker_);

  armor_marker_.id = 3;
  armor_marker_.pose = armors_msg_.armors[2].pose;
  armor_marker_.color.r = 0.0;
  armor_marker_.color.g = 1.0;
  armor_marker_.color.b = 0.0;
  marker_array_.markers.emplace_back(armor_marker_);

  direction_marker_.header = armors_msg_.header;
  direction_marker_.id = 1;
  direction_marker_.pose = armors_msg_.armors[0].pose;

  marker_array_.markers.emplace_back(direction_marker_);

  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::OutpostDetectorNode)