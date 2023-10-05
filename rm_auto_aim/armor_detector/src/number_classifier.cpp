// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"

namespace rm_auto_aim
{
// 数字分类器的构造函数
// 初始化：
// 1. 网络模型路径
// 2. 标签路径
// 3. 分类阈值，删除置信度低于阈值的装甲板
// 4. 忽略的类别，默认为“negative”，即非装甲板样本（负样本）
// 5. 神经网络
// 6. 类别名称
NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

// 获取装甲板数字
void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // Light length in image
  // 灯条标准长度
  const int light_length = 12; // 灯条标准长度
  // Image size after warp
  // 装甲板标准尺寸
  const int warp_height = 28; // 装甲板标准高度
  const int small_armor_width = 32; // 小装甲板标准宽度
  const int large_armor_width = 54; // 大装甲板标准宽度
  // Number ROI size
  // 数字图像标准尺寸
  const cv::Size roi_size(20, 28); // 数字图像标准尺寸

  // 遍历装甲板容器获取数字图像
  for (auto & armor : armors) {
    // Warp perspective transform
    // 透视变换

    // 从装甲板容器中获取灯条的四个顶点
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    // 获取灯条的顶部y坐标，这里的值是7
    const int top_light_y = (warp_height - light_length) / 2 - 1;
    // 获取灯条的底部y坐标，这里的值是19
    const int bottom_light_y = top_light_y + light_length;
    // 根据装甲板的类型，获取装甲板的宽度，小装甲板宽度为32，大装甲板宽度为54
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    // 标准装甲板图像灯条的四个顶点
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    // 用来存放数字图像
    cv::Mat number_image;
    // 获取透视变换矩阵，该矩阵为原始灯条对应到标准装甲板图像的灯条
    // lights_vertices：原始灯条的四边形
    // target_vertices：变换后灯条的矩形
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    // 透视变换
    // src：原始图像
    // number_image：变换后的图像
    // rotation_matrix：透视变换矩阵
    // cv::Size(warp_width, warp_height)：变换后的图像尺寸
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    // 获取ROI，即截取数字图像
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    // 转化为灰度图像
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    // 二值化
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 保存数字图像
    armor.number_img = number_image;
  }
}

// 对装甲板数字进行分类
void NumberClassifier::classify(std::vector<Armor> & armors)
{
  // 遍历装甲板容器
  for (auto & armor : armors) {
    // 获取图像副本
    cv::Mat image = armor.number_img.clone();

    // Normalize
    // 归一化，将图像像素值归一化到0~1之间
    image = image / 255.0;

    // Create blob from image
    // 将图像转化为blob，blob是一种用于神经网络的数据结构
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    // 将blob设置为神经网络的输入
    net_.setInput(blob);
    // Forward pass the image blob through the model
    // 神经网络前向传播，得到模型输出
    cv::Mat outputs = net_.forward();

    // Do softmax
    // 对输出进行softmax，得到每个类别的概率

    // 这行代码找到模型原始输出中的最大值。Softmax计算过程中常常从每个
    // 元素中减去最大值以增加数值稳定性（防止指数运算结果过大）
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    // 定义一个新的矩阵来存储Softmax的结果
    cv::Mat softmax_prob;
    // 对每个输出值减去最大值后取指数，结果存储在softmax_prob中
    cv::exp(outputs - max_prob, softmax_prob);
    // 计算softmax_prob所有元素的总和。这是Softmax的分母
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    // 对softmax_prob进行归一化，使其所有元素的总和为1
    softmax_prob /= sum;

    // 获取置信度和预测的类别索引

    // 函数找到矩阵中的最小和最大值及其位置。在此处，我们只关心最大值（置信度）
    // 和其位置（预测的类别索引）
    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    // 从class_id_point中提取类别索引
    int label_id = class_id_point.x;

    // 将置信度和分类结果存储到装甲板对象中
    armor.confidence = confidence; // 存储置信度
    // 使用类别索引从class_names_数组中获取预测的类别，并存储到armor对象中
    armor.number = class_names_[label_id];

    // 创建分类结果字符串
    // 使用std::stringstream来构造一个包含分类结果和置信度的字符串
    std::stringstream result_ss;
    // 将分类结果、置信度和“%”添加到流中
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    // 将构造的字符串存储到armor对象中
    armor.classfication_result = result_ss.str();
  }

  // 删除不符合条件的装甲板
  armors.erase(
    std::remove_if( // 使用std::remove_if来删除不符合条件的装甲板
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        // 条件1: 置信度低于阈值threshold
        if (armor.confidence < threshold) {
          return true;
        }

        // 条件2: 类别为ignore_classes_忽略的类别
        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }

        // 条件3: 分类结果与装甲板类型不匹配
        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) { // 大装甲板且为前哨站、工程或哨兵
          mismatch_armor_type =
          armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) { // 小装甲板且为英雄或基地
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
}

}  // namespace rm_auto_aim
