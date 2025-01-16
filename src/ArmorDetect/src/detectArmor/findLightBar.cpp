#include "detect.h"

bool isVerticalLike(float angle) {
    return angle <= 15 || angle >= 165;
}

//按轮廓的宽高比.面积大小筛选轮廓
std::vector<LightDescriptor> findLightBar:: Lights(const std::vector<std::vector<cv::Point>>& contours){
    //存储筛选过的灯条
    std::vector<LightDescriptor> m_light;

    // 遍历轮廓，拟合椭圆并计算外接矩形
    // 因为灯条是椭圆型的，所以用椭圆去拟合轮廓，再直接获取旋转外接矩形
    for (size_t i = 0; i < contours.size(); ++i) {
        if (contours[i].size() < 5) continue; // 拟合椭圆需要至少5个点
        // 获取椭圆的外接矩形
        cv::RotatedRect Light_Rec = fitEllipse(contours[i]);

        // // 通过倾斜角度筛选灯条
        if (!isVerticalLike(Light_Rec.angle)) {
            continue;
        }

        // 计算宽高比
        float aspectRatio = static_cast<float> (Light_Rec.size.height) / (Light_Rec.size.width); 
        if(aspectRatio < threshold::minRatio || aspectRatio > threshold::maxRatio || Light_Rec.size.area() < threshold:: minArea ||
            Light_Rec.size.area() > threshold::maxArea){
                continue;
            }         
            m_light.push_back(Light_Rec);
        
   }
    return m_light;
}