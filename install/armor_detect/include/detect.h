#ifndef DETECT_H
#define DETECT_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>  // 包含 sort 函数

//计算两点之间的欧几里得距离
inline float calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2){
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

class threshold{
public:
    // 原图像二值化阈值 
    static double original_image_threshold; 
    // 通道相减图像二值化阈值
    static double minus_image_threshold;

    // 筛选灯条的阈值
    static float minArea;   //最小面积
    static float maxArea;   // 最大面积
    static double minRatio;   //最小宽高比  
    static double maxRatio;   //最大宽高比

    // 匹配灯条的阈值
    static float max_angle_diff;        // 筛选平行角度的阈值
    static float max_len_diff_ratio;   // 灯条长度差的最大比率
    static float max_dist;           // 最大距离阈值 

    // 绘制装甲板阈值
    static double min_armorArea;    // 绘制装甲板的最小面积
};

// 创建灯条类
class LightDescriptor{
public:
    float width, length, angle, area;    // 宽度，长度，角度，面积
    cv::Point2f center, point[4];    //灯条的质心，4个顶点
public:
    LightDescriptor() {};    //简单声明灯条
    LightDescriptor(const cv::RotatedRect& light){
        width = light.size.width;
        length = light.size.height;
        angle = light.angle;
        area = light.size.area();
        center = light.center;
        light.points(point);
    }
};

//图像的预处理
class imageDispose{
public:
    // 通道相减图像的预处理
    static void minus_dispose(const cv::Mat& m_frame, cv::Mat& m_dst);
    // 原图像的处理
    static void image_dispose(const cv::Mat& m_frame, cv::Mat& m_binaryImage);

    cv::Mat imageGaussion(const cv::Mat& m_frame);   //使用高斯函数平滑图像，减少噪声
    cv::Mat stressBlue(const cv::Mat& m_frame);   //筛选出蓝色
    cv::Mat stressRed(const cv::Mat& m_frame);    //筛选出红色
    cv::Mat imageThreshold(const cv::Mat& m_frame, const double& thresh);    //对彩色/灰色图像进行二值化处理
    cv::Mat imageDilate(const cv::Mat& m_binaryImage);    //对二值化图像进行膨胀
    cv::Mat imageClose(const cv::Mat& binary);       // 对二值化图像进行闭运算处理
    cv::Mat imageOpen(const cv::Mat& binary);       // 对二值化图像进行开运算处理
};

//检测灯条
class findLightBar {
public:
    std::vector<LightDescriptor> Lights(const std::vector<std::vector<cv::Point>>& CONTOURS_MATCH_I2);
};

//匹配灯条
class matchingLightBar{
public:
    std::vector<std::pair<LightDescriptor, LightDescriptor>> matchLight(const std::vector<LightDescriptor>& lights);
};

// 识别装甲板
class findArmor{
public:
    std::vector<std::pair<LightDescriptor, LightDescriptor>> find_Armor(std::vector<std::pair<LightDescriptor, LightDescriptor>>& m_matchingLightBar);
};

// 匹配装甲板
class matchingArmor{
public:
    //绘制装甲板
    cv::Mat matchingArmors(const std::vector<std::pair<LightDescriptor, LightDescriptor>>& m_lights, cv::Mat& m_frame, std::vector<cv::Point2f>& m_armor);
};

#endif

