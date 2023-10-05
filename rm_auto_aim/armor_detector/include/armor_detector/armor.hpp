// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

// 用来表示装甲板的类型
enum class ArmorType { SMALL, LARGE, INVALID };
// 装甲板类型的字符串表示
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::RotatedRect
{
  Light() = default;
  /**
   * @brief 创建一个灯条
   * @param box 灯条的旋转矩形
   */
  explicit Light(cv::RotatedRect box) // explicit 防止隐式转换
  : cv::RotatedRect(box)
  {
    // 将旋转矩形的四个顶点转换为灯条的两个顶点
    cv::Point2f p[4];
    box.points(p);
    // 按照 y 坐标从小到大排序，排序规则为 lambda 表达式，即按照 y 坐标从小到大排序
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) {return a.y < b.y;});
    // 计算灯条的顶点和底点
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    // 计算灯条的长度和宽度
    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    // 计算灯条的倾斜角
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  // 灯条的颜色
  int color;
  // 灯条的顶点、底点坐标
  cv::Point2f top, bottom;
  // 灯条的长度
  double length;
  // 灯条的宽度
  double width;
  // 灯条的倾斜角
  float tilt_angle;
};

struct Armor
{
  Armor() = default;
  /**
   * @brief 根据两个灯条创建一个装甲板
   * @param l1 灯条1
   * @param l2 灯条2
   */
  Armor(const Light & l1, const Light & l2)
  {
    // 判断左右灯条
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    // 计算装甲板的中心点
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light; //灯条对
  cv::Point2f center; //装甲板中心点
  ArmorType type; //装甲板类型

  // Number part
  cv::Mat number_img; //装甲板数字图像
  std::string number; //装甲板数字
  float confidence; //装甲板数字置信度
  std::string classfication_result; //装甲板数字分类结果
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
