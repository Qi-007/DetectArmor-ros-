#include "detect.h"

// 图像预处理的阈值
double threshold::minus_image_threshold = 120;
double threshold::original_image_threshold = 180;

// 筛选灯条的阈值
float threshold::minArea = 100.0f;   //最小面积
float threshold::maxArea = 1500.0f;   // 最大面积
double threshold::minRatio = 1.2;   //最小宽高比  
double threshold::maxRatio = 8.0;   //最大宽高比

// 匹配灯条的阈值
float threshold::max_angle_diff = 10.0f;        // 筛选平行角度的阈值
float threshold::max_len_diff_ratio = 0.25f;   // 灯条长度差的最大比率
float threshold::max_dist = 500.0f;           // 最大距离阈值

// 绘制装甲板阈值
double threshold::min_armorArea = 500.0;    // 绘制装甲板的最小面积