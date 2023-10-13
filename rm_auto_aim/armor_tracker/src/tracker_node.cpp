// Copyright 2022 Chen Jun
#include "armor_tracker/tracker_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
/// @brief 跟踪器节点的构造函数
/// @param options 节点选项
/// @details 初始化信息有：
/// 1. 节点名称，为"armor_tracker"
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  // 获取参数，XOY平面内最大允许的装甲版距离
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  // 获取参数，最大匹配距离
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  // 获取参数，最大匹配角度差
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  // 创建Tracker对象
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  // 获取参数，追踪阈值
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  // 获取参数，丢失阈值
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  // 创建Kalman滤波器的F矩阵
  auto f = [this](const Eigen::VectorXd & x) {
      Eigen::VectorXd x_new = x;
      x_new(0) += x(1) * dt_;
      x_new(2) += x(3) * dt_;
      x_new(4) += x(5) * dt_;
      x_new(6) += x(7) * dt_;
      return x_new;
    };
  // J_f - Jacobian of process function
  // 创建Kalman滤波器的F矩阵的雅克比矩阵
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
  // 创建Kalman滤波器的H矩阵
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
  // 创建Kalman滤波器的H矩阵的雅克比矩阵
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
  // 创建Kalman滤波器的Q矩阵
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
  // 创建Kalman滤波器的R矩阵
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
      Eigen::DiagonalMatrix<double, 4> r;
      double x = r_xyz_factor;
      r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
      return r;
    };
  // P - error estimate covariance matrix
  // 创建Kalman滤波器的P矩阵
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  // 创建Kalman滤波器
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Subscriber with tf2 message_filter
  // 订阅tf2消息
  // tf2 relevant
  // tf2的relevant
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
  // 创建发布器，发布TrackerInfo消息
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  // 创建发布器，发布Target消息
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
  armor_marker_.ns = "pred_armors";
  pred_armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  pred_armor_marker_.scale.x = 0.03;
  pred_armor_marker_.scale.z = 0.125;
  pred_armor_marker_.color.a = 1.0;
  pred_armor_marker_.color.g = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  //trajectory slover param
  // 弹道解算器参数
  int max_iter = this->declare_parameter("trajectory.max_iter", 10);
  float stop_error = this->declare_parameter("trajectory.stop_error", 0.001);
  int R_K_iter = this->declare_parameter("trajectory.R_K_iter", 50);
  double init_speed = this->declare_parameter("trajectory.init_bullet_speed", 26.5);
  bool is_hero = this->declare_parameter("trajectory.is_hero", false);
  static_offset_yaw_ = this->declare_parameter("trajectory.static_offset.yaw", 0.0);
  static_offset_pitch_ = this->declare_parameter("trajectory.static_offset.pitch", 0.0);

  //Get fire angle thres
  // 获取参数，射击角度阈值
  yaw_angle_thres = this->declare_parameter("yaw_angle_thres", 25.0);
  fire_permit_thres = this->declare_parameter("fire_permit_thres", 1.5);
  fire_latency = this->declare_parameter("fire_latenc3y", 0.02);
  min_speed = this->declare_parameter("min_speed", 20.0);

  // 创建弹道解算器
  trajectory_slover_ =
    std::make_shared<TrajectorySlover>(max_iter, stop_error, R_K_iter, init_speed, is_hero);

  // 创建订阅器，订阅子弹速度
  // 回调函数为setBulletSpeed
  bullet_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/bullet_speed", 10,
    std::bind(&ArmorTrackerNode::setBulletSpeed, this, std::placeholders::_1));

  // 创建订阅器，订阅延迟
  latency_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/latency", 10, std::bind(&ArmorTrackerNode::setLatancy, this, std::placeholders::_1));
}

/// @brief 根据下位机的子弹速度，设置弹道解算器的子弹速度
/// @param bulletspeed 订阅器接收子弹速度
void ArmorTrackerNode::setBulletSpeed(const std_msgs::msg::Float64::SharedPtr bulletspeed)
{
  // 接收到子弹数据
  if (bulletspeed->data != 0) {
    // 子弹速度与弹道解算器的子弹速度相差0.2
    auto diff = bulletspeed->data - trajectory_slover_->getBulletSpeed();
    if ((diff > 0.2 || diff < -0.2) && bulletspeed->data > min_speed) {
      // 设置弹道解算器的子弹速度
      trajectory_slover_->setBulletSpeed(bulletspeed->data);
      RCLCPP_INFO(
        this->get_logger(), "set bullet speed: %.3f", trajectory_slover_->getBulletSpeed());
    }
  }
}

/// @brief 设置延迟
/// @param latency 订阅器接收延迟
void ArmorTrackerNode::setLatancy(const std_msgs::msg::Float64::SharedPtr latency)
{
  if (latency->data >= 0) {
    latency_ = latency->data;
  }
}

/// @brief 进行跟踪
void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{

  //////////////////
  // 1. 坐标变换   //
  //////////////////

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
  // 过滤异常装甲板：高度大于1.2m，距离大于最大击打距离
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
  geometry_msgs::msg::TransformStamped transform_pitch;
  try {
    transform_pitch = tf2_buffer_->lookupTransform(
      "shoot_pitch_link", target_frame_,
      tf2::TimePointZero);
    // auto orientation_gimbal = transform_pitch.transform.rotation;
    // tf2::Quaternion q_;
    // tf2::fromMsg(orientation_gimbal, q_);
    // gimbal_yaw = -tf2::impl::getYaw(q_);
    //RCLCPP_INFO(get_logger(), "gimbal_yaw : %lf", gimbal_yaw);
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return;
  }

  //Tranform gimbal state from world coordinate
  geometry_msgs::msg::TransformStamped transform_yaw;
  try {
    transform_yaw =
      tf2_buffer_->lookupTransform("shoot_yaw_link", target_frame_, tf2::TimePointZero);

  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return;
  }

  //////////////////
  // 2. 开始跟踪   //
  //////////////////

  // Init message
  // 初始化消息
  auto_aim_interfaces::msg::TrackerInfo info_msg; // 用来发布跟踪信息
  auto_aim_interfaces::msg::Target target_msg;    // 用来储存检测器计算出来的目标信息
  rclcpp::Time time = armors_msg->header.stamp;   // 获取时间戳，该时间戳为图像消息的时间戳
  target_msg.header.stamp = time;                 // Target消息的时间戳
  target_msg.header.frame_id = target_frame_;     // Target消息的坐标系

  // Update tracker
  // 更新跟踪器
  // 如果跟踪器状态为LOST，则初始化跟踪器  
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else { // 如果跟踪器状态为DETECTING、TRACKING或TEMP_LOST，则更新跟踪器
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_); // 将丢失阈值时间转化为次数
    tracker_->update(armors_msg); // 更新跟踪器

    // Publish Info
    // 发布TrackerInfo消息
    info_msg.position_diff = tracker_->info_position_diff; // 位置差
    info_msg.yaw_diff = tracker_->info_yaw_diff;          // 偏航角差
    info_msg.position.x = tracker_->measurement(0);      // 装甲板x坐标
    info_msg.position.y = tracker_->measurement(1);     // 装甲板y坐标
    info_msg.position.z = tracker_->measurement(2);    // 装甲板z坐标
    info_msg.yaw = tracker_->measurement(3);          // 装甲板偏航角
    info_pub_->publish(info_msg);                    // 发布TrackerInfo消息

    // 根据跟踪器更新后的状态，进行分析
    if (tracker_->tracker_state == Tracker::DETECTING) {
      // 如果还在检测中，则不进行任何操作
      target_msg.tracking = false;
    } else if ( // 如果跟踪器状态为TRACKING或TEMP_LOST
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST)
    {
      // 设置为正在跟踪
      target_msg.tracking = true;
      // Fill target message
      // 填充跟踪信息
      const auto & state = tracker_->target_state; // 跟踪器的状态向量
      target_msg.id = tracker_->tracked_id; // 跟踪器所跟踪的目标装甲板id
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num); // 跟踪器所跟踪的目标机器人的装甲板数量，原先为enum，这里做强制类型转化
      target_msg.position.x = state(0); // 跟踪器所跟踪的目标机器人的x坐标
      target_msg.velocity.x = state(1); // 跟踪器所跟踪的目标机器人的x速度
      target_msg.position.y = state(2); // 跟踪器所跟踪的目标机器人的y坐标
      target_msg.velocity.y = state(3); // 跟踪器所跟踪的目标机器人的y速度
      target_msg.position.z = state(4); // 跟踪器所跟踪的目标装甲板的z坐标
      target_msg.velocity.z = state(5); // 跟踪器所跟踪的目标装甲板的z速度
      target_msg.yaw = state(6);        // 跟踪器所跟踪的目标装甲板的偏航角
      target_msg.v_yaw = state(7);      // 跟踪器所跟踪的目标装甲板的偏航角速度
      target_msg.radius_1 = state(8);   // 跟踪器所跟踪的目标装甲板的半径1
      target_msg.radius_2 = tracker_->another_r; // 跟踪器所跟踪的目标装甲板的半径2
      target_msg.dz = tracker_->dz;     // 跟踪器所跟踪的目标装甲板的高度差
    }
  }

  // 记录时间戳用于下一次计算
  last_time_ = time;

  /////////////////////
  // 3. 跟踪信息处理   //
  /////////////////////

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

    //save the pred target
    double pred_dt =
      fire_latency + latency_ / 1000 + (now_car_pos.norm() - target_msg.radius_1) /
      trajectory_slover_->getBulletSpeed();
    Eigen::Vector3d pred_car_pos = now_car_pos + now_car_vec * pred_dt;
    double pred_yaw = armor_yaw + car_w * pred_dt;
    car_center_diff = calcYawAndPitch(pred_car_pos)[0];
    double r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double dz = target_msg.dz;

    Eigen::Vector3d armor_target_min_dis, armor_target, pred_armor_pos, armor_target_min_yaw_diff;
    //double min_dis_yaw;
    double min_yaw_diff = DBL_MAX;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    //double min_distance_armor = DBL_MAX;
    //double min_yaw_diff_armor = DBL_MAX;
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
      // double armor_distance = pred_armor_pos.norm();
      // if(armor_distance < min_distance_armor)
      // {
      //   min_distance_armor = armor_distance;
      //   armor_target_min_dis = pred_armor_pos;
      //   //min_distance_target_num = i;
      //   min_dis_yaw = a_yaw;
      // }
      auto tmp_yaw_diff = fabs(a_yaw - car_center_diff);
      if (tmp_yaw_diff < min_yaw_diff) {
        min_yaw_diff = tmp_yaw_diff;
        armor_target_min_yaw_diff = pred_armor_pos;
        //min_distance_target_num = i;
      }
    }

    //Tracked permit
    armor_target = armor_target_min_yaw_diff;
    bool tracked_permit = 0;
    if (rad2deg(min_yaw_diff) < yaw_angle_thres) {
      tracked_permit = 1;
    }

    //Transform world frame to pitch frame
    geometry_msgs::msg::Vector3Stamped point_target, armor_target_tf;
    point_target.vector.x = armor_target(0);
    point_target.vector.y = armor_target(1);
    point_target.vector.z = armor_target(2);

    tf2::doTransform(point_target, armor_target_tf, transform_pitch);
    Eigen::Vector3d armor_target_pitch_link;
    armor_target_pitch_link(0) = armor_target_tf.vector.x;
    armor_target_pitch_link(1) = armor_target_tf.vector.y;
    armor_target_pitch_link(2) = armor_target_tf.vector.z;

    Eigen::Vector3d armor_target_yaw_link;
    tf2::doTransform(point_target, armor_target_tf, transform_yaw);
    armor_target_yaw_link(0) = armor_target_tf.vector.x;
    armor_target_yaw_link(1) = armor_target_tf.vector.y;
    armor_target_yaw_link(2) = armor_target_tf.vector.z;

    //Get offset
    Eigen::Vector2d angel_diff = calcYawAndPitch(armor_target_pitch_link);
    auto trajectory_pitch = (-trajectory_slover_->calcPitchCompensate(armor_target));


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
    RCLCPP_INFO(this->get_logger(), "fire_permit : %d", fire_permit);
    RCLCPP_INFO(get_logger(), "distance : %lf", (now_car_pos.norm() - target_msg.radius_1));
    RCLCPP_INFO(get_logger(), "speed : %lf", trajectory_slover_->getBulletSpeed());
    RCLCPP_INFO(get_logger(), "latency : %lf", latency_ / 1000.0);
    //RCLCPP_INFO(this->get_logger(), "tracked_permit : %d", tracked_permit);
    // RCLCPP_INFO(this->get_logger(), "car_center_diff : %lf", rad2deg(car_center_diff));
    // RCLCPP_INFO(this->get_logger(), "min_yaw_diff : %lf", rad2deg(min_yaw_diff));
    //RCLCPP_INFO(this->get_logger(), "yaw_angle_thres : %lf", yaw_angle_thres);
    //RCLCPP_INFO(this->get_logger(), "fire_permit_thres : %lf", fire_permit_thres);
    //RCLCPP_INFO(this->get_logger(), "fire_latency : %lf", fire_latency);

    if (!(isnan(target_msg.offset_yaw) || isnan(target_msg.offset_pitch))) {
      target_pub_->publish(target_msg);
    }

    publishMarkers(target_msg);
  }
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;
  pred_armor_marker_.header = target_msg.header;

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
      fire_latency + latency_ / 1000.0 + (now_car_pos.norm() - target_msg.radius_1) /
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
    armor_marker_.points.clear();
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
    double min_yaw_diff = DBL_MAX;
    r = 0;
    tf2::Quaternion q;
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

      //Get armor yaw
      double a_yaw = tmp_yaw;
      if (a_yaw >= M_PI) {
        a_yaw -= 2.0 * M_PI;
      }

      //Get min yaw armor
      auto tmp_yaw_diff = fabs(a_yaw - car_center_diff);
      if (tmp_yaw_diff < min_yaw_diff) {
        min_yaw_diff = tmp_yaw_diff;
        target_p_a.x = pred_p_a[0];
        target_p_a.y = pred_p_a[1];
        target_p_a.z = pred_p_a[2];
        q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);

      }
    }
    pred_armor_marker_.pose.position = target_p_a;
    pred_armor_marker_.pose.orientation = tf2::toMsg(q);
    marker_array.markers.emplace_back(pred_armor_marker_);
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    pred_armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.emplace_back(armor_marker_);
    marker_array.markers.emplace_back(pred_armor_marker_);
  }

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
