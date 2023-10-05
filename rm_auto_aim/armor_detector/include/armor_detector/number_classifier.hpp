// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  /**
   * @brief 创建一个数字分类器（构造函数）
   * @param model_path 网络模型路径
   * @param label_path 标签路径
   * @param threshold 分类阈值，删除置信度低于阈值的装甲板
   * @param ignore_classes 忽略的类别，默认为“negative”，即非装甲板样本（负样本）
   */
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  /**
   * @brief 获取装甲板数字图像，直接将获取到的图像存入armor的number_img中
   * @param src 原始图像
   * @param armors 装甲板容器
   */
  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  /**
   * @brief 对装甲板数字进行分类
   * @param armors 装甲板容器
   */
  void classify(std::vector<Armor> & armors);

  // 装甲板数字分类器的阈值，删除置信度低于阈值的装甲板
  double threshold;

private:
  // 神经网络
  cv::dnn::Net net_;
  // 类别名称
  std::vector<std::string> class_names_;
  // 忽略的类别，默认为“negative”，即非装甲板样本（负样本）
  std::vector<std::string> ignore_classes_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
