#include "detect.h"

// 角度差值并进行标准化
float angle_normalization(float& angle){
    if(angle >= 90){
        angle = fabs(angle - 180);
    }
    return angle;
}

//两两匹配灯条
std::vector<std::pair<LightDescriptor, LightDescriptor>> matchingLightBar:: matchLight(const std::vector<LightDescriptor>& lights){
    // 按距离进行排序的匹配灯条
    std::vector<std::pair<LightDescriptor, LightDescriptor>> matched;

    // 对灯条的所有可能组合按照距离排序
    std::vector<std::tuple<float, size_t, size_t>> distances; // (距离, index1, index2)
    for (size_t i = 0; i < lights.size(); i++) {
        for (size_t j = i + 1; j < lights.size(); j++) {
            float dist = calculateDistance(lights[i].center, lights[j].center);
            distances.emplace_back(dist, i, j);
        }
    }

    // 按距离从小到大排序
    sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return std::get<0>(a) < std::get<0>(b);
    });

    // 检查每对灯条是否符合条件
    for (const auto& [dist, i, j] : distances) {
        if (dist > threshold::max_dist) break; // 跳过超过最大距离的灯条

        LightDescriptor leftLight = lights[i];
        LightDescriptor rightLight = lights[j];

        // 角度值归一化
        angle_normalization(rightLight.angle);
        angle_normalization(leftLight.angle);

        // 角差
        float angleDiff = fabs(leftLight.angle - rightLight.angle);

        // 长度差比率
        float lenDiff_ratio = fabs(leftLight.length - rightLight.length) / std::max(leftLight.length, rightLight.length);

        // 跳过不符合条件的灯条
        if (angleDiff > threshold::max_angle_diff || lenDiff_ratio > threshold::max_len_diff_ratio) {
            continue;
        }

        matched.emplace_back(leftLight, rightLight);
    }

    return matched;
}
