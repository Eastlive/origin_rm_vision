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

OutpostDetectorNode::OutpostDetectorNode(const rclcpp::NodeOptions& options)
 : Node("outpost_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Outpost detector node started.");

  outpost_x_ = declare_parameter("outpost_x", 0.0);
  outpost_y_ = declare_parameter("outpost_y", 0.0);
  outpost_z_ = declare_parameter("outpost_z", 2.5);

  orientation[0] = declare_parameter("orientation_x", 0.56098552679693100);
  orientation[1] = declare_parameter("orientation_y", -0.56098552679693100);
  orientation[2] = declare_parameter("orientation_z", 0.43045933457687941);
  orientation[3] = declare_parameter("orientation_w", 0.43045933457687941);

  tf2::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
  tf2::Quaternion q_120(0, sin(M_PI / 3), 0, cos(M_PI / 3));
  armor_msg_[0].number = "outpost";
  armor_msg_[0].type = "small";
  armor_msg_[0].distance_to_image_center = sqrt(outpost_x_ * outpost_x_ + outpost_y_ * outpost_y_ + outpost_z_ * outpost_z_) * 200;
  armor_msg_[0].pose.position.x = outpost_x_;
  armor_msg_[0].pose.position.y = outpost_y_;
  armor_msg_[0].pose.position.z = outpost_z_ - radius;
  armor_msg_[0].pose.orientation.x = q.x();
  armor_msg_[0].pose.orientation.y = q.y();
  armor_msg_[0].pose.orientation.z = q.z();
  armor_msg_[0].pose.orientation.w = q.w();


  q = q_120 * q;
  armor_msg_[1].number = "outpost";
  armor_msg_[1].type = "small";
  armor_msg_[1].distance_to_image_center = sqrt(outpost_x_ * outpost_x_ + outpost_y_ * outpost_y_ + outpost_z_ * outpost_z_) * 200;
  armor_msg_[1].pose.position.x = outpost_x_ - radius * sin(2 * M_PI / 3);
  armor_msg_[1].pose.position.y = outpost_y_;
  armor_msg_[1].pose.position.z = outpost_z_ - radius * cos(2 * M_PI / 3);
  armor_msg_[1].pose.orientation.x = q.x();
  armor_msg_[1].pose.orientation.y = q.y();
  armor_msg_[1].pose.orientation.z = q.z();
  armor_msg_[1].pose.orientation.w = q.w();

  q = q_120 * q;
  armor_msg_[2].number = "outpost";
  armor_msg_[2].type = "small";
  armor_msg_[2].distance_to_image_center = sqrt(outpost_x_ * outpost_x_ + outpost_y_ * outpost_y_ + outpost_z_ * outpost_z_) * 200;
  armor_msg_[2].pose.position.x = outpost_x_ - radius * sin(4 * M_PI / 3);
  armor_msg_[2].pose.position.y = outpost_y_;
  armor_msg_[2].pose.position.z = outpost_z_ - radius * cos(4 * M_PI / 3);
  armor_msg_[2].pose.orientation.x = q.x();
  armor_msg_[2].pose.orientation.y = q.y();
  armor_msg_[2].pose.orientation.z = q.z();
  armor_msg_[2].pose.orientation.w = q.w();

  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.045;
  armor_marker_.scale.y = 0.135;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  armor_marker_.color.g = 0.0;
  armor_marker_.color.b = 0.0;
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

  now_angle_ = 0;

  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&OutpostDetectorNode::timerCallback, this));
}

void OutpostDetectorNode::timerCallback()
{
  armors_msg_.header.frame_id = "camera_optical_link";
  armors_msg_.header.stamp = this->now();

  dt_ = this->now().seconds() - last_time_;
  last_time_ = this->now().seconds();
  now_angle_ += angle_speed * dt_;
  // RCLCPP_INFO(this->get_logger(), "now_angle_: %lf", now_angle_);

  rotate(armor_msg_[0].pose, angle_speed * dt_);
  revolve(armor_msg_[0].pose, 0);
  rotate(armor_msg_[1].pose, angle_speed * dt_);
  revolve(armor_msg_[1].pose, 2 * M_PI / 3);
  rotate(armor_msg_[2].pose, angle_speed * dt_);
  revolve(armor_msg_[2].pose, 4 * M_PI / 3);

  for (int i = 0; i < 3; i++)
  {
    armor_msg_[i].distance_to_image_center
      = sqrt(armor_msg_[i].pose.position.x * armor_msg_[i].pose.position.x
      + armor_msg_[i].pose.position.y * armor_msg_[i].pose.position.y
      + armor_msg_[i].pose.position.z * armor_msg_[i].pose.position.z
      ) * 200;
  }

  armors_msg_.armors.clear();

  for (int i = 0; i < 3; i++)
  {
    if(armor_msg_[i].pose.position.z < outpost_z_ - radius * sin(M_PI / 10))
    {
      armors_msg_.armors.emplace_back(armor_msg_[i]);
    }
  }

  armors_pub_->publish(armors_msg_);

  publishMarkers();
}

void OutpostDetectorNode::rotate(geometry_msgs::msg::Pose & pose, double angle)
{
  // 四元数相乘
  tf2::Quaternion q(0, sin(angle / 2), 0, cos(angle / 2));
  tf2::Quaternion q_pose(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  q_pose = q * q_pose;
  pose.orientation.x = q_pose.x();
  pose.orientation.y = q_pose.y();
  pose.orientation.z = q_pose.z();
  pose.orientation.w = q_pose.w();
}

void OutpostDetectorNode::revolve(geometry_msgs::msg::Pose & pose, double offset)
{
  pose.position.x = outpost_x_ - radius * sin(now_angle_ + offset);
  pose.position.y = outpost_y_;
  pose.position.z = outpost_z_ - radius * cos(now_angle_ + offset);
}

void OutpostDetectorNode::publishMarkers()
{
  marker_array_.markers.clear();

  armor_marker_.header = armors_msg_.header;

  for (int i = 0; i < 3; i++)
  {
    armor_marker_.id = i;
    armor_marker_.pose = armor_msg_[i].pose;
    if (armor_msg_[i].pose.position.z < outpost_z_ - radius * sin(M_PI / 10))
    {
      armor_marker_.color.r = 1.0;
      armor_marker_.color.g = 0.0;
      armor_marker_.color.b = 0.0;
      armor_marker_.color.a = 1.0;
    }
    else
    {
      armor_marker_.color.r = 0.0;
      armor_marker_.color.g = 1.0;
      armor_marker_.color.b = 0.0;
      armor_marker_.color.a = 0.5;
    }
    marker_array_.markers.emplace_back(armor_marker_);
  }

  // direction_marker_.header = armors_msg_.header;
  // direction_marker_.id = 1;
  // direction_marker_.pose = armor_msg_[0].pose;

  // marker_array_.markers.emplace_back(direction_marker_);

  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::OutpostDetectorNode)