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
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
}

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

// 判断两个灯条中间是否有其他灯条
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
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  // 返回装甲板类型
  return type;
}

cv::Mat Detector::getAllNumbersImage()
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}
void Detector::drawResults(cv::Mat & img)
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace rm_auto_aim
