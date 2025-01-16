#include "detect.h"

class find_armor_condition{
public:
    float max_y_diff_ratio = 0.8;
    float min_x_diff_ratio = 1.2;
    float max_aspect_ratio = 3.3;
    float min_aspect_ratio = 1.0;
};

std::vector<std::pair<LightDescriptor, LightDescriptor>> findArmor::find_Armor(std::vector<std::pair<LightDescriptor, LightDescriptor>>& m_matchingLightBar){
    std::vector<std::pair<LightDescriptor, LightDescriptor>> armor;

    for(const auto& matchedLights : m_matchingLightBar){
        const auto& l1 = matchedLights.first;
        const auto& l2 = matchedLights.second;
        // 将点存入容器
        std::vector<LightDescriptor> lights = {l1, l2};

        // 按 x 值从小到大排序
        sort(lights.begin(), lights.end(), [](const LightDescriptor& l1, const LightDescriptor& l2) {
            return l1.center.x < l2.center.x; // 根据 x 值升序排序
        });

        LightDescriptor leftLight = lights[0];
        LightDescriptor rightLight = lights[1];

        // 左右灯条相距距离
        float dis = calculateDistance(leftLight.center, rightLight.center);
        // float dis = pow(pow((leftLight.center.x - rightLight.center.x), 2) + pow((leftLight.center.y - rightLight.center.y), 2), 0.5);

        // 左右灯条长度的平均值
        float meanLen = (leftLight.length + rightLight.length) / 2;

        // 左右灯条的高度差
        float yDiff = fabs(leftLight.center.y - rightLight.center.y);

        // 高度差比率
        float yDiff_ratio = yDiff / meanLen;

        // 左右灯条的宽度差
        float xDiff = fabs(leftLight.center.x - rightLight.center.x);

        // 宽度差比率
        float xDiff_ratio = xDiff / meanLen;

        // 相距距离与灯条长度比值
        float ratio = dis / meanLen;

        find_armor_condition con;

        // 筛选
        if(
            yDiff_ratio > con.max_y_diff_ratio || 
            xDiff_ratio < con.min_x_diff_ratio ||
            ratio > con.max_aspect_ratio ||
            ratio < con.min_aspect_ratio
        ){
            // cout << "meanLen: " << meanLen << "   yDiff: " << yDiff << "   yDiff_ratio: " << yDiff_ratio<< endl;
            // cout << "meanLen: " << meanLen << "   xDiff: " << xDiff << "   xDiff_ratio: " << xDiff_ratio<< endl;
            // cout << "meanLen: " << meanLen << "   dis: " << dis << "   ratio: " << ratio << endl;
            // cout << "xDiff: " << xDiff_ratio << "   yDiff: " << yDiff_ratio << "   ratio:" << ratio << "   meanLen: " << meanLen << endl; 
            continue;
        }
        // cout << "xDiff: " << xDiff_ratio << "   yDiff: " << yDiff_ratio << "   ratio:" << ratio << "   meanLen: " << meanLen << endl; 
        armor.emplace_back(matchedLights);
    }
    return armor;
}