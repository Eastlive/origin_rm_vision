// Copyright 2022 Chen Jun
#include "armor_tracker/tracker_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
      Eigen::VectorXd x_new = x;
      x_new(0) += x(1) * dt_;
      x_new(2) += x(3) * dt_;
      x_new(4) += x(5) * dt_;
      x_new(6) += x(7) * dt_;
      return x_new;
    };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
      Eigen::MatrixXd f(9, 9);
      // clang-format off
      f << 1, dt_, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, dt_, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, dt_, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, dt_, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;
      // clang-format on
      return f;
    };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
      Eigen::VectorXd z(4);
      double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
      z(0) = xc - r * cos(yaw); // xa
      z(1) = yc - r * sin(yaw); // ya
      z(2) = x(4);             // za
      z(3) = x(6);             // yaw
      return z;
    };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
      Eigen::MatrixXd h(4, 9);
      double yaw = x(6), r = x(8);
      // clang-format off
      //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
      h << 1, 0, 0, 0, 0, 0, r * sin(yaw), 0, -cos(yaw),
        0, 0, 1, 0, 0, 0, -r * cos(yaw), 0, -sin(yaw),
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0;
      // clang-format on
      return h;
    };
  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 20.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 800.0);
  auto u_q = [this]() {
      Eigen::MatrixXd q(9, 9);
      double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
      double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
      double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
      double q_r = pow(t, 4) / 4 * r;
      // clang-format off
      //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
      q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0,
        q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0,
        0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0,
        0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0,
        0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0,
        0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0,
        0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0,
        0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0,
        0, 0, 0, 0, 0, 0, 0, 0, q_r;
      // clang-format on
      return q;
    };
  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
      Eigen::DiagonalMatrix<double, 4> r;
      double x = r_xyz_factor;
      r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
      return r;
    };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  pred_armor_marker_.ns = "pred_armors";
  pred_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  pred_armor_marker_.scale.x = 0.03;
  pred_armor_marker_.scale.z = 0.125;
  pred_armor_marker_.color.a = 1.0;
  pred_armor_marker_.color.g = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  //trajectory slover param
  int max_iter = this->declare_parameter("trajectory.max_iter", 10);
  float stop_error = this->declare_parameter("trajectory.stop_error", 0.001);
  int R_K_iter = this->declare_parameter("trajectory.R_K_iter", 50);
  double init_speed = this->declare_parameter("trajectory.init_bullet_speed", 26.5);
  bool is_hero = this->declare_parameter("trajectory.is_hero", false);
  static_offset_yaw_ = this->declare_parameter("trajectory.static_offset.yaw", 0.0);
  static_offset_pitch_ = this->declare_parameter("trajectory.static_offset.pitch", 0.0);

  //Get fire angle thres
  yaw_angle_thres = this->declare_parameter("yaw_angle_thres", 25.0);
  fire_permit_thres = this->declare_parameter("fire_permit_thres", 1.5);
  fire_latency = this->declare_parameter("fire_latency", 0.02);

  trajectory_slover_ =
    std::make_shared<TrajectorySlover>(max_iter, stop_error, R_K_iter, init_speed, is_hero);

  bullet_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/bullet_speed", 10,
    std::bind(&ArmorTrackerNode::setBulletSpeed, this, std::placeholders::_1));

  latency_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/latency", 10, std::bind(&ArmorTrackerNode::setLatancy, this, std::placeholders::_1));
}

void ArmorTrackerNode::setBulletSpeed(const std_msgs::msg::Float64::SharedPtr bulletspeed)
{

  if (bulletspeed->data != 0) {
    auto diff = bulletspeed->data - trajectory_slover_->getBulletSpeed();
    if (diff > 0.2 || diff < -0.2) {
      trajectory_slover_->setBulletSpeed(bulletspeed->data);
      RCLCPP_INFO(
        this->get_logger(), "set bullet speed: %.3f", trajectory_slover_->getBulletSpeed());
    }
  }
}

void ArmorTrackerNode::setLatancy(const std_msgs::msg::Float64::SharedPtr latency)
{
  if (latency->data >= 0) {
    latency_ = latency->data;
  }
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{

  // Print armors
  RCLCPP_INFO(this->get_logger(), "Armor number: %d", static_cast<int>(armors_msg->armors.size()));
  for (auto & armor : armors_msg->armors) {
    RCLCPP_INFO(
      this->get_logger(), "armor x: %f, y: %f, z: %f", armor.pose.position.x,
      armor.pose.position.y, armor.pose.position.z);
  }

  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
        Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
        max_armor_distance_;
      }),
    armors_msg->armors.end());

  //Get gimbal status
  //Tranform gimbal state from world coordinate
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf2_buffer_->lookupTransform("pitch_link", target_frame_, tf2::TimePointZero);
    auto orientation_gimbal = transform.transform.rotation;
    tf2::Quaternion q_;
    tf2::fromMsg(orientation_gimbal, q_);
    gimbal_yaw = -tf2::impl::getYaw(q_);
    //RCLCPP_INFO(get_logger(), "gimbal_yaw : %lf", gimbal_yaw);
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return;
  }
  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST)
    {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;
    }
  }

  last_time_ = time;

  if (target_msg.tracking == true) {

    //save target states
    Eigen::Vector3d now_car_pos = Eigen::Vector3d(
      target_msg.position.x,
      target_msg.position.y,
      target_msg.position.z);
    Eigen::Vector3d now_car_vec = Eigen::Vector3d(
      target_msg.velocity.x,
      target_msg.velocity.y,
      target_msg.velocity.z);
    double armor_yaw = target_msg.yaw;
    double car_w = target_msg.v_yaw;

    RCLCPP_INFO(get_logger(), "distance : %lf", (now_car_pos.norm() - target_msg.radius_1));
    RCLCPP_INFO(get_logger(), "speed : %lf", trajectory_slover_->getBulletSpeed());
    //save the pred target
    double pred_dt =
      fire_latency + latency_ / 1000 + (now_car_pos.norm() - target_msg.radius_1) /
      trajectory_slover_->getBulletSpeed();
    RCLCPP_INFO(get_logger(), "latency : %lf", latency_ / 1000.0);
    Eigen::Vector3d pred_car_pos = now_car_pos + now_car_vec * pred_dt;
    double pred_yaw = armor_yaw + car_w * pred_dt;
    auto car_center_diff = calcYawAndPitch(pred_car_pos);
    double r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double dz = target_msg.dz;

    Eigen::Vector3d armor_target_min_dis, armor_target, pred_armor_pos;
    double min_dis_yaw;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    double min_distance_armor = DBL_MAX;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = pred_yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        pred_armor_pos[2] = pred_car_pos[2] + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        pred_armor_pos[2] = pred_car_pos[2];
      }
      pred_armor_pos[0] = pred_car_pos[0] - r * cos(tmp_yaw);
      pred_armor_pos[1] = pred_car_pos[1] - r * sin(tmp_yaw);

      //get armor yaw
      double a_yaw = tmp_yaw;
      if (a_yaw >= M_PI) {
        a_yaw -= 2.0 * M_PI;
      }

      //get minimum distance target
      double armor_distance = pred_armor_pos.norm();
      if (armor_distance < min_distance_armor) {
        min_distance_armor = armor_distance;
        armor_target_min_dis = pred_armor_pos;
        //min_distance_target_num = i;
        min_dis_yaw = a_yaw;
      }
    }

    //tracked permit
    armor_target = armor_target_min_dis;
    auto a2c_yaw_diff = fabs(min_dis_yaw - car_center_diff[0]);
    bool tracked_permit = 0;
    if (rad2deg(a2c_yaw_diff) < yaw_angle_thres) {
      tracked_permit = 1;
    }

    //Transform world frame to pitch frame
    geometry_msgs::msg::Vector3Stamped point_target, armor_target_tf;
    point_target.vector.x = armor_target(0);
    point_target.vector.y = armor_target(1);
    point_target.vector.z = armor_target(2);
    tf2::doTransform(point_target, armor_target_tf, transform);
    Eigen::Vector3d armor_target_pitch_link;
    armor_target_pitch_link(0) = armor_target_tf.vector.x;
    armor_target_pitch_link(1) = armor_target_tf.vector.y;
    armor_target_pitch_link(2) = armor_target_tf.vector.z;

    //Get offset
    Eigen::Vector2d angel_diff = calcYawAndPitch(armor_target_pitch_link);
    auto trajectory_pitch = (-trajectory_slover_->calcPitchCompensate(armor_target));
    auto trajectory_view = trajectory_slover_->getTrajectoryWorld();

    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());
    RCLCPP_INFO(this->get_logger(), "SHHANGYGANG TRAJECTORY SIZE : %ld", trajectory_view.size());

    //Set fire permit
    int8_t fire_permit = 0;
    if (fabs(angel_diff[0]) < fire_permit_thres && tracked_permit) {
      fire_permit = 1;
    }

    target_msg.fire_permit = fire_permit;
    target_msg.offset_yaw = rad2deg((double)angel_diff(0)) + static_offset_yaw_;
    target_msg.offset_pitch = rad2deg((double)angel_diff(1)) + trajectory_pitch +
      static_offset_pitch_;

    RCLCPP_INFO(this->get_logger(), "trajectory_pitch : %lf", trajectory_pitch);
    RCLCPP_INFO(
      this->get_logger(), "offset_pitch : %lf , offset_yaw : %lf", target_msg.offset_pitch,
      target_msg.offset_yaw);

    //RCLCPP_INFO(this->get_logger(), "yaw_angle_thres : %lf", yaw_angle_thres);
    //RCLCPP_INFO(this->get_logger(), "fire_permit_thres : %lf", fire_permit_thres);
    //RCLCPP_INFO(this->get_logger(), "fire_latency : %lf", fire_latency);
    RCLCPP_INFO(this->get_logger(), "fire_permit : %d", fire_permit);

    if (!(isnan(target_msg.offset_yaw) || isnan(target_msg.offset_pitch))) {
      target_pub_->publish(target_msg);
    }

    publishMarkers(target_msg, trajectory_view);
  }
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg, const std::vector<Eigen::Vector3d> & trajectory_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;
  pred_armor_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;
    double car_w = target_msg.v_yaw;

    Eigen::Vector3d now_car_pos = Eigen::Vector3d(
      target_msg.position.x,
      target_msg.position.y,
      target_msg.position.z);
    Eigen::Vector3d now_car_vec = Eigen::Vector3d(
      target_msg.velocity.x,
      target_msg.velocity.y,
      target_msg.velocity.z);

    double pred_dt =
      fire_latency + latency_ / 1000 + (now_car_pos.norm() - target_msg.radius_1) /
      trajectory_slover_->getBulletSpeed();
    double pred_yaw = yaw + car_w * pred_dt;
    Eigen::Vector3d pred_car_pos = now_car_pos + now_car_vec * pred_dt;


    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
    //pred_armor visualization

    pred_armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    pred_armor_marker_.points.clear();
    pred_armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    is_current_pair = true;
    Eigen::Vector3d pred_p_a;
    geometry_msgs::msg::Point target_p_a;
    double min_distance_armor = DBL_MAX;
    r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = pred_yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        pred_p_a[2] = pred_car_pos[2] + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        pred_p_a[2] = pred_car_pos[2];
      }
      pred_p_a[0] = pred_car_pos[0] - r * cos(tmp_yaw);
      pred_p_a[1] = pred_car_pos[1] - r * sin(tmp_yaw);

      //get armor yaw
      double a_yaw = tmp_yaw;
      if (a_yaw >= M_PI) {
        a_yaw -= 2.0 * M_PI;
      }

      //get minimum distance target
      double armor_distance = pred_p_a.norm();
      if (armor_distance < min_distance_armor) {
        min_distance_armor = armor_distance;
        target_p_a.x = pred_p_a[0];
        target_p_a.y = pred_p_a[1];
        target_p_a.z = pred_p_a[2];
        //min_distance_target_num = i;
        pred_armor_marker_.id = i;
        pred_armor_marker_.pose.position = target_p_a;
        tf2::Quaternion q;
        q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
        pred_armor_marker_.pose.orientation = tf2::toMsg(q);
      }
    }
    marker_array.markers.emplace_back(pred_armor_marker_);

    //trajectory visualization
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.points.reserve(trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    RCLCPP_INFO(this->get_logger(), "trajectory size : %ld", trajectory_msg.size());
    for (const auto & point : trajectory_msg) {
      geometry_msgs::msg::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      trajectory_marker_.points.emplace_back(p);
    }
    marker_array.markers.emplace_back(trajectory_marker_);
    
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    pred_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_array.markers.emplace_back(armor_marker_);
  marker_array.markers.emplace_back(pred_armor_marker_);
  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
