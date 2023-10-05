// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class PnPSolver
{
public:
  /**
   * @brief PnP解算构造函数
   * @param camera_matrix 3x3相机内参矩阵
   * @param distortion_coefficients 镜头畸变系数
   */
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  // 获取3D坐标
  /**
   * @brief 获取3D坐标
   * @param armor 装甲板信息
   * @param rvec 物体相对于相机的旋转
   * @param tvec 物体相对于相机的平移
   * @return 是否解算成功
   */
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  /**
   * @brief 计算装甲板中心点到图像中心点的距离
   * @param image_point 装甲板中心点
   * @return 装甲板中心点到图像中心点的距离
   */
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  // 3x3相机内参矩阵
  cv::Mat camera_matrix_;
  // 镜头畸变系数
  cv::Mat dist_coeffs_;

  // Unit: mm
  // 定义标准装甲板尺寸
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // Four vertices of armor in 3d
  // 定义了两种类型的装甲板（小和大）在3D空间中的角点坐标，并将这
  // 些点添加到small_armor_points_和large_armor_points_向量
  // 中。注意点的坐标是按照从左下角开始的顺时针顺序定义的，并且使用
  // 的坐标系是：x向前、y向左、z向上
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
