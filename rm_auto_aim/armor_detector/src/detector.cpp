// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
/// @brief 检测器的构造函数
/// @param bin_thres 二值化阈值
/// @param color 装甲板颜色
/// @param l 灯条参数
/// @param a 装甲板参数
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
}

/// @brief 对图像进行装甲板检测
/// @param input 输入图像
/// @return 装甲板容器
std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  // 预处理图像
  binary_img = preprocessImage(input);
  // 寻找灯条
  lights_ = findLights(input, binary_img);
  // 匹配灯条，寻找装甲板
  armors_ = matchLights(lights_);

  // 如果装甲板不为空，提取数字并分类
  if (!armors_.empty()) {
    // 获取数字分类器
    classifier->extractNumbers(input, armors_);
    // 分类数字
    classifier->classify(armors_);
  }

  // 返回装甲板容器
  return armors_;
}

/// @brief 对图像进行预处理
/// @param rgb_img 输入图像
/// @return 二值化图像
cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  // 将图像转换为灰度图像
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  // 对图像进行二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  // 返回二值化图像
  return binary_img;
}

/// @brief 寻找符合要求的灯条
/// @param rbg_img 原始图像
/// @param binary_img 二值化图像
/// @return 灯条容器
std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  // 用来存储轮廓
  vector<vector<cv::Point>> contours;
  // 用来存储轮廓的层次结构
  vector<cv::Vec4i> hierarchy;
  // 寻找轮廓
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 创建灯条容器
  vector<Light> lights;
  // 清空调试信息
  this->debug_lights.data.clear();

  // 遍历轮廓，寻找灯条
  for (const auto & contour : contours) {
    // 轮廓点数小于 5 个，不是灯条
    if (contour.size() < 5) {continue;}

    // 寻找轮廓的最小外接矩形
    auto r_rect = cv::minAreaRect(contour);
    // 将旋转矩形转化为灯条
    auto light = Light(r_rect);

    // 判断是否为灯条
    if (isLight(light)) {
      // 如果是灯条，计算灯条的颜色
      auto rect = light.boundingRect();
      // 防止越界，检查矩形是否在图像内
      if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols &&
          0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= rbg_img.rows)
      {
        // 如果矩形在图像内，计算矩形内红色和蓝色像素的和
        int sum_r = 0, sum_b = 0;
        // 获取矩形内的 ROI ，ROI即 Region of Interest，感兴趣区域，用来提取矩形内的像素
        auto roi = rbg_img(rect);
        // 遍历 ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            // 判断像素是否在轮廓内，如果在轮廓内，计算红色和蓝色像素的和
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // 累加红色和蓝色像素的和
              sum_r += roi.at<cv::Vec3b>(i, j)[0];
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // 红色像素的和大于蓝色像素的和，灯条为红色
        light.color = sum_r > sum_b ? RED : BLUE;
        // 将灯条存入灯条容器
        lights.emplace_back(light);
      }
    }
  }

  // 返回灯条容器
  return lights;
}

/// @brief 判断灯条是否符合要求
/// @param light 灯条
/// @return 是否为灯条
bool Detector::isLight(const Light & light)
{
  // 灯条的短边长 / 长边长
  float ratio = light.width / light.length;
  // 判断灯条的短边长 / 长边长是否在范围内
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;
  // 判断灯条的倾斜角度是否在范围内
  bool angle_ok = light.tilt_angle < l.max_angle;
  // 判断是否为灯条
  bool is_light = ratio_ok && angle_ok;

  // 填充调试信息
  auto_aim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  // 返回是否为灯条
  return is_light;
}

/// @brief 匹配灯条，寻找装甲板
/// @param lights 灯条容器
/// @return 装甲板容器
std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  // 创建装甲板容器
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // 遍历所有灯条的配对
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      // 判断两个灯条的颜色是否相同，如果不相同，跳过
      if (light_1->color != detect_color || light_2->color != detect_color) {continue;}

      // 判断两个灯条中间是否有其他灯条，如果有，跳过
      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      // 判断两个灯条是否符合装甲板的要求
      auto type = isArmor(*light_1, *light_2);
      // 如果符合装甲板的要求，将装甲板存入装甲板容器
      if (type != ArmorType::INVALID) {
        // 创建装甲板，将灯条存为装甲板
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  // 返回装甲板容器
  return armors;
}

/// @brief 判断两个灯条中间是否有其他灯条
/// @param light_1 灯条1
/// @param light_2 灯条2
/// @param lights 灯条容器
/// @return 是否有其他灯条
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  // 计算两个灯条的中心点连线的矩形
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  // 遍历所有灯条，判断是否在矩形内
  for (const auto & test_light : lights) {
    // 如果是选定的两个灯条，跳过
    if (test_light.center == light_1.center || test_light.center == light_2.center) {continue;}

    // 判断灯条的顶点、底点、中心点是否在矩形内
    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center))
    {
      // 如果在矩形内，返回 true
      return true;
    }
  }

  // 如果不在矩形内，返回 false
  return false;
}

/// @brief 判断两个灯条是否符合装甲板的要求
/// @param light_1 灯条1
/// @param light_2 灯条2
/// @return 装甲板类型
ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // 计算两个灯条的长度比 (短边长 / 长边长)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length :
    light_2.length / light_1.length;
  // 判断长度比是否在范围内
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // 计算两个灯条的平均长度
  float avg_light_length = (light_1.length + light_2.length) / 2;
  // 计算两个灯条中心的距离
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  // 判断中心距离是否在范围内
  bool center_distance_ok = (a.min_small_center_distance <= center_distance && // 判断是否为小装甲板
    center_distance < a.max_small_center_distance) || // 判断是否为小装甲板
    (a.min_large_center_distance <= center_distance && // 判断是否为大装甲板
    center_distance < a.max_large_center_distance); // 判断是否为大装甲板

  // 计算两个灯条中心连线的角度
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  // 判断角度是否在范围内
  bool angle_ok = angle < a.max_angle;
  
  // 判断是否为装甲板
  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // 判断装甲板类型
  ArmorType type;
  if (is_armor) {
    // 判断是否为大装甲板
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    // 不是装甲板
    type = ArmorType::INVALID;
  }

  // 填充调试信息
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)]; // 装甲板类型
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2; // 装甲板中心点的x坐标
  armor_data.light_ratio = light_length_ratio; // 两个灯条的长度比
  armor_data.center_distance = center_distance; // 两个灯条中心的距离
  armor_data.angle = angle; // 两个灯条中心连线的角度
  this->debug_armors.data.emplace_back(armor_data); // 将调试信息存入调试信息容器

  // 返回装甲板类型
  return type;
}

/// @brief 获取所有数字图片
/// @return 所有数字图片
cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    // 如果装甲板为空，返回空的数字图片
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    // 创建数字图片容器
    std::vector<cv::Mat> number_imgs;
    // 预留空间
    number_imgs.reserve(armors_.size());
    // 遍历所有装甲板，将数字图片存入数字图片容器
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    // 创建所有数字图片
    cv::Mat all_num_img;
    // 将所有数字图片拼接在一起
    cv::vconcat(number_imgs, all_num_img);
    // 返回所有数字图片
    return all_num_img;
  }
}

/// @brief Debug，将识别到的数字绘制在图像上
/// @param img 输入图像
/// @return 绘制了数字的图像
void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  // 绘制灯条图案
  for (const auto & light : lights_) {
    // 绘制灯条的顶点
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    // 绘制灯条的底点
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    // 判断灯条的颜色，绘制不同颜色的灯条
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    // 绘制灯条
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  // 绘制装甲板图案
  for (const auto & armor : armors_) {
    // 绘制的装甲板图像为X的图案
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  // 显示数字和置信度
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace rm_auto_aim
