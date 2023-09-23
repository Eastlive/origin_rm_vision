// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
class Detector
{
public:
  /**
   * @brief 灯条参数
   */
  struct LightParams
  {
    // 灯条的宽高比（灯条的宽度 / 灯条的高度）
    double min_ratio; // 灯条的最小宽高比
    double max_ratio; // 灯条的最大宽高比
    //垂直角度
    double max_angle;
  };

  /**
   * @brief 装甲板参数
   */
  struct ArmorParams
  {
    // 灯条长度比
    double min_light_ratio;
    // 灯条中心距离
    double min_small_center_distance; // 小装甲板两个灯条中心的最小距离
    double max_small_center_distance; // 小装甲板两个灯条中心的最大距离
    double min_large_center_distance; // 大装甲板两个灯条中心的最小距离
    double max_large_center_distance; // 大装甲板两个灯条中心的最大距离
    // 水平角度
    double max_angle;
  };

  /**
   * @brief 创建一个装甲板检测器（构造函数）
   * @param bin_thres 二值化阈值
   * @param color 装甲板颜色
   * @param l 灯条参数
   * @param a 装甲板参数
   */
  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  /**
   * @brief 对图像进行装甲板检测
   * @param input 输入图像
   * @return 装甲板容器
   */
  std::vector<Armor> detect(const cv::Mat & input);

  /**
   * @brief 对图像进行预处理
   * @param input 输入图像
   * @return 二值化图像
   */
  cv::Mat preprocessImage(const cv::Mat & input);

  /**
   * @brief 寻找符合基本条件的灯条，包括灯条的筛选、颜色的判断
   * @param rbg_img 原始图像
   * @param binary_img 二值化图像
   * @return 已找到的灯条容器
   */
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);

  /**
   * @brief 对灯条进行匹配，判断是否符合装甲板的要求
   * @param lights 灯条容器
   * @return 装甲板容器
   */ 
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // 用来Debug的函数
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  // 用来存放二值化阈值
  int binary_thres;
  // 用来存放装甲板颜色
  int detect_color;
  // 用来存放灯条参数，l代表light
  LightParams l;
  // 用来存放装甲板参数，a代表armor（糟糕的命名）
  ArmorParams a;

  // 数字分类器，用来识别数字
  std::unique_ptr<NumberClassifier> classifier;

  // Debug信息，用于调试
  cv::Mat binary_img;
  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
  /**
   * @brief 判断两个灯条是否符合装甲板的要求
   * @param possible_light 可能的灯条
   * @return 是否符合灯条要求
   */
  bool isLight(const Light & possible_light);
  
  /**
   * @brief 判断两个灯条中间是否有其他灯条
   * @param light_1 灯条1
   * @param light_2 灯条2
   * @param lights 灯条容器
   * @return 是否符合灯条要求
   */
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  
  /**
   * @brief 判断两个灯条是否符合装甲板的要求
   * @param light_1 灯条1
   * @param light_2 灯条2
   * @return 装甲板类型
   */
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
