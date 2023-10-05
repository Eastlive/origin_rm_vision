// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
// PnP解算构造函数
// 初始化：
// 1. camera_matrix 3x3相机内参矩阵
// 2. dist_coeffs 镜头畸变系数
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  // 使用constexpr定义了两种装甲板的半高度和半宽度，并将其从毫米
  // 转换为米。这些尺寸用于定义3D世界中的装甲板角点
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  // 定义了两种类型的装甲板（小和大）在3D空间中的角点坐标，并将这
  // 些点添加到small_armor_points_和large_armor_points_向量
  // 中。注意点的坐标是按照从左下角开始的顺时针顺序定义的，并且使用
  // 的坐标系是：x向前、y向左、z向上
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

// 获取3D坐标
bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // Fill in image points
  // 
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // Solve pnp
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
  // 使用solvePnP函数解算装甲板的旋转向量和平移向量
  // 该函数的参数依次为：
  // 1. 3D世界中的点
  // 2. 2D图像中的点
  // 3. 相机内参矩阵
  // 4. 镜头畸变系数
  // 5. 输出的旋转向量
  // 6. 输出的平移向量
  // 7. 指示是否使用初始估计的标志（此处为false，表示不使用）
  // 8. 使用的算法（此处为cv::SOLVEPNP_IPPE）
  // 该函数返回一个布尔值，表示求解是否成功
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

// 计算装甲板中心点到图像中心点的距离
float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  // 从相机矩阵中提取中心坐标cx和cy
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  // 使用cv::norm计算image_point和中心点之间的距离
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
