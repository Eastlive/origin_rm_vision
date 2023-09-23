#include "armor_tracker/trajectory_slover.hpp"

namespace rm_auto_aim
{
/**
     * @brief constructor ,init param from yaml
     * @param coord_path
     * @param name
     */
TrajectorySlover::TrajectorySlover(
  int max_iter, float stop_error, int R_K_iter, double bullet_speed, bool is_hero)
{
  max_iter_ = max_iter;
  stop_error_ = stop_error;
  R_K_iter_ = R_K_iter;
  bullet_speed_ = bullet_speed;
  if (is_hero) {
    k = K_BIG;
  } else {
    k = K_SMALL;
  }
}

/**
     * #brief trajectory solver get pitch offset
     * @param point_world
     * @return pitch offset
     * @attention 用到的知识：高中斜抛运动+空气阻力。求解方法:四阶龙格库塔法 + 迭代法
     *             https://zhuanlan.zhihu.com/p/34261490
     */
double TrajectorySlover::calcPitchCompensate(Eigen::Vector3d & point_world)
{
  //TODO:根据陀螺仪安装位置调整距离求解方式
  //降维，坐标系Y轴以垂直向上为正方向
  auto dist_vertical = point_world[2];
  auto vertical_tmp = dist_vertical;
  auto dist_horizonal = sqrt(point_world.squaredNorm() - dist_vertical * dist_vertical);
  //auto dist_horizonal = sqrt(point_world[0] * point_world[0] + point_world[1] * point_world[1]);
  // auto dist_vertical = xyz[2];
  // auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
  auto pitch = atan(dist_vertical / dist_horizonal) * 180.0 / PI;
  // std::cout << "begin trajectory slove : " << std::endl;
  // std::cout << "origin pitch : " << pitch << std::endl;  //此pitch应与相机坐标系算出的offset差不多
  // std::cout << "dist_horizonal: " << dist_horizonal << std::endl;
  // std::cout << "dist_vertical: " << dist_vertical << std::endl;
  // std::cout << "point_world.squaredNorm(): " << point_world.squaredNorm() << std::endl;
  // std::cout << "pitch: " << pitch << std::endl;
  auto pitch_new = pitch;
  //开始使用龙格库塔法求解弹道补偿
  for (int i = 0; i < max_iter_; i++) { //迭代次数
    //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
    //初始化
    auto x = 0.0;
    auto y = 0.0;
    auto p = tan(pitch_new / 180 * PI);  // p = dy / dx
    auto v = bullet_speed_;
    auto u = v / sqrt(1 + pow(p, 2));
    auto delta_x = dist_horizonal / R_K_iter_;
    for (int j = 0; j < R_K_iter_; j++) {
      auto k1_u = -k * u * sqrt(1 + pow(p, 2));
      auto k1_p = -g / pow(u, 2);
      auto k1_u_sum = u + k1_u * (delta_x / 2);
      auto k1_p_sum = p + k1_p * (delta_x / 2);

      auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
      auto k2_p = -g / pow(k1_u_sum, 2);
      auto k2_u_sum = u + k2_u * (delta_x / 2);
      auto k2_p_sum = p + k2_p * (delta_x / 2);

      auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
      auto k3_p = -g / pow(k2_u_sum, 2);
      auto k3_u_sum = u + k3_u * (delta_x / 2);
      auto k3_p_sum = p + k3_p * (delta_x / 2);

      auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
      auto k4_p = -g / pow(k3_u_sum, 2);

      u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
      p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

      x += delta_x;
      y += p * delta_x;
    }
    //评估迭代结果,若小于迭代精度需求则停止迭代
    auto error = dist_vertical - y;
    if (abs(error) <= stop_error_) {
      break;
    } else {
      vertical_tmp += error;
      // xyz_tmp[1] -= error;
      pitch_new = atan(vertical_tmp / dist_horizonal) * 180.0 / PI;
      // std::cout << "iter_i: " << i << " error: " << error << " pitch_new: " << pitch_new
      //           << std::endl;
    }
  }
  // std::cout <<"offset:" << pitch_new-pitch << std::endl <<std::endl ;
  return pitch_new - pitch;
}

}  // namespace rm_auto_aim
