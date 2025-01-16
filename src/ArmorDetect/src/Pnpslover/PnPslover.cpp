#include "PnPslover.h"
#include <iostream>


namespace armor_auto_aim {
ArmorPnpSlover::ArmorPnpSlover(const std::array<double, 9>& intrinsic_matrix,
                               const std::vector<double>& distortion_vector) 
                               : m_intrinsic_matrix(cv::Mat(3, 3, CV_64F, const_cast<double*>(intrinsic_matrix.data())).clone()),       // 内参矩阵
                               m_distortion_vector(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_vector.data())).clone()) {      // 畸变系数
    // m
    constexpr double small_armro_half_x = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;     // / 2.0 取装甲板尺寸的一半
    constexpr double small_armro_half_y = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;    // /1000.0 将单位从毫米转化为米
    constexpr double large_armro_half_x = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;     // 用来表示装甲板的三维坐标点
    constexpr double large_armro_half_y = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;
    // std::cout << "dv: ";
    // for (int i = 0; i < m_distortion_vector.rows; ++i) {
    //     for (int j = 0; j < m_distortion_vector.cols; ++j) {
    //         std::cout << m_distortion_vector.ptr<double>(i)[j] << " ";      // .ptr<double>(i) 获取第i行的指针
    //     }
    // }
    // std::cout << std::endl;
    // std::cout << "im: ";
    // for (int i = 0; i < m_intrinsic_matrix.rows; ++i) {
    //     for (int j = 0; j < m_intrinsic_matrix.cols; ++j) {
    //         std::cout << m_intrinsic_matrix.ptr<double>(i)[j] << " ";
    //     }
    // }
    // std::cout << std::endl;
    // 像素坐标系为左上角逆时针
 
    // 世界坐标系和相机坐标系都设为   z向前   x向右  y向下 

    // 小型装甲板的三维点，左上角逆时针顺序
    m_small_armor_point3d.push_back(cv::Point3f(-0, -small_armro_half_x,  small_armro_half_y));  // 左上
    m_small_armor_point3d.push_back(cv::Point3f(-0, -small_armro_half_x, -small_armro_half_y));   // 左下
    m_small_armor_point3d.push_back(cv::Point3f(-0,  small_armro_half_x, -small_armro_half_y));    // 右下
    m_small_armor_point3d.push_back(cv::Point3f(-0,  small_armro_half_x,  small_armro_half_y));   // 右上

    // m_small_armor_point3d.push_back(cv::Point3f(0, -small_armro_half_x, -small_armro_half_y));  // 左上
    // m_small_armor_point3d.push_back(cv::Point3f(0, -small_armro_half_x,  small_armro_half_y));   // 左下
    // m_small_armor_point3d.push_back(cv::Point3f(0,  small_armro_half_x,  small_armro_half_y));    // 右下
    // m_small_armor_point3d.push_back(cv::Point3f(0,  small_armro_half_x, -small_armro_half_y));   // 右上

    // m_small_armor_point3d.push_back(cv::Point3f(-small_armro_half_x, -small_armro_half_y, 0));  // 左上
    // m_small_armor_point3d.push_back(cv::Point3f(-small_armro_half_x,  small_armro_half_y, 0));   // 左下
    // m_small_armor_point3d.push_back(cv::Point3f( small_armro_half_x,  small_armro_half_y, 0));    // 右下
    // m_small_armor_point3d.push_back(cv::Point3f( small_armro_half_x, -small_armro_half_y, 0));   // 右上

    // m_small_armor_point3d.push_back(cv::Point3f(0, -small_armro_half_x, -small_armro_half_y));  // 左上
    // m_small_armor_point3d.push_back(cv::Point3f(0, -small_armro_half_x, small_armro_half_y));   // 左下
    // m_small_armor_point3d.push_back(cv::Point3f(0, small_armro_half_x, small_armro_half_y));    // 右下
    // m_small_armor_point3d.push_back(cv::Point3f(0, small_armro_half_x, -small_armro_half_y));   // 右上

    // m_small_armor_point3d.push_back(cv::Point3f(-small_armro_half_x,  small_armro_half_y, 0));  // 左上
    // m_small_armor_point3d.push_back(cv::Point3f(-small_armro_half_x, -small_armro_half_y, 0));   // 左下
    // m_small_armor_point3d.push_back(cv::Point3f( small_armro_half_x, -small_armro_half_y, 0));    // 右下
    // m_small_armor_point3d.push_back(cv::Point3f( small_armro_half_x,  small_armro_half_y, 0));   // 右上

    // 大型装甲板的三维点
    m_large_armor_point3d.push_back(cv::Point3f(0, -large_armro_half_x, -large_armro_half_y)); 
    m_large_armor_point3d.push_back(cv::Point3f(0, -large_armro_half_x, large_armro_half_y)); 
    m_large_armor_point3d.push_back(cv::Point3f(0, large_armro_half_x, large_armro_half_y)); 
    m_large_armor_point3d.push_back(cv::Point3f(0, large_armro_half_x, large_armro_half_y)); 
}

// rvec（朝向.旋转矩阵）  tvec（位置.平移向量）
// cv:: solvePnP(世界坐标系下的控制点的坐标， 图像坐标系下对应的控制点坐标， 相机的内参矩阵， 相机的畸变系数， 输出的旋转向量， 输出的平移向量， PnP解法)
// 返回 true 表示成功估算了摄像机的姿态  返回 false 表示失败
bool ArmorPnpSlover::pnpSlove(const std::vector<cv::Point2f>& armor, cv::Mat& rvec, cv::Mat& tvec) {
    try {
        // if (armor.ifsmall) {
        //     std::cout << "small armor pnp slover" << std::endl;
        //     return cv::solvePnP(m_small_armor_point3d, armor.points, m_intrinsic_matrix, m_distortion_vector,
        //                         rvec, tvec, false, cv::SOLVEPNP_IPPE);
        // } else {
        //     std::cout << "large armor pnp slover" << std::endl;
        //     return cv::solvePnP(m_large_armor_point3d, armor.points, m_intrinsic_matrix, m_distortion_vector,
        //                         rvec, tvec, false, cv::SOLVEPNP_IPPE);
        // }
            // std::cout << "small armor pnp slover" << std::endl;
            // std:: cout << m_small_armor_point3d.size() << std:: endl;
            if(m_small_armor_point3d.size() == armor.size()) {
                m_small_armor_point3d.size() == armor.size(); 
                // std::cout << "rvec = " << rvec.at<double>(0,0) << " ," << rvec.at<double>(1,0) << " ," << rvec.at<double>(2,0) << std::endl;
                // std::cout << "tvec = " << tvec.at<double>(0,0) << " ," << tvec.at<double>(1,0) << " ," << tvec.at<double>(2,0) << std::endl;
            }
            return cv::solvePnP(m_small_armor_point3d, armor, m_intrinsic_matrix, m_distortion_vector,
                                rvec, tvec, false, cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception& e) {      // 处理异常代码
        // std::cout << "Intrinsic Matrix:" << std::endl << m_intrinsic_matrix << std::endl;       // 查看内参矩阵和畸变系数是否错误
        // std::cout << "Distortion Coefficients:" << std::endl << m_distortion_vector << std::endl;

        for (int i = 0; i < 4; ++i) {
            auto item = armor[i];
            RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "%d: %f, %f;", i, item.x, item.y);    // 通过 ROS 2 的 RCLCPP_WARN 宏，在循环中输出 armor.points 中的每个点的索引及其二维坐标值（x, y）
        }
        RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "-----");     // 使用 RCLCPP_WARN 打印 ROS 2 的日志，方便在运行时查看和分析异常原因
        return false;
    }
}

// 光心（相机的图像中心）
float ArmorPnpSlover::computeArmorToCenter(const cv::Point2f &center) {
    float cx = m_intrinsic_matrix.at<double>(0, 2);     // 光心的x坐标
    float cy = m_intrinsic_matrix.at<double>(1, 2);     // 光心的y坐标
    return cv::norm(center - cv::Point2f(cx, cy));      // 返回center与光心的距离

}

// geometry_msgs::msg::Quaternion ArmorPnpSlover::orientationFromRvec(const cv::Mat &r_vec)  {
//     // 将旋转向量转换为旋转矩阵
//     cv::Mat rotation_matrix;
//     cv::Rodrigues(r_vec, rotation_matrix);      // 用Rodrigues公式将rvec（旋转矢量）转化为旋转矩阵
//     // 将 OpenCV 的旋转矩阵转换为 tf2 的 Matrix3x3
//     // transform2处理和变换不同坐标系之间的位置和方向
//     // tf2::Matrix3x是 tf2 库中的一个矩阵类型，用于表示 3x3 的旋转矩阵
//     tf2::Matrix3x3 tf2_rotation_matrix(
//         rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
//         rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
//         rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
//     );
//     // 获取当前帧的四元数
//     tf2::Quaternion tf2_q;
//     tf2_rotation_matrix.getRotation(tf2_q);
//     std::cout << "Original Quaternion: "
//             << "x: " << tf2_q.x() << ", "
//             << "y: " << tf2_q.y() << ", "
//             << "z: " << tf2_q.z() << ", "
//             << "w: " << tf2_q.w() << std::endl;
//     // 如果是第一帧，直接设置 last_q 为当前四元数
//     if (is_first_frame) {
//         last_q = tf2_q;
//     }
//     // 进行四元数插值 (Slerp)
//     double t = 0.05;  // 插值因子
//     tf2::Quaternion interpolated_q = last_q.slerp(tf2_q, t);
//     // 更新 last_q 为当前插值后的四元数，供下一帧使用
//     last_q = interpolated_q;
//     // 将插值后的四元数转换为欧拉角
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(interpolated_q).getRPY(roll, pitch, yaw);
//     // 将欧拉角转换为度数
//     roll = roll * 180.0 / M_PI;
//     pitch = pitch * 180.0 / M_PI;
//     yaw = yaw * 180.0 / M_PI;
//     if (is_first_frame) {
//         last_pitch = pitch;
//         last_roll = roll;
//         last_yaw = yaw;
//         is_first_frame = false;
//     }
//     std::cout << "Interpolated roll  : " << roll <<  " ";
//     std::cout << "Interpolated pitch : " << pitch << " ";
//     std::cout << "Interpolated yaw   : " << yaw << std::endl;
//     // 使用角度差更新法，避免角度突变
//     double delta_roll = angles::shortest_angular_distance(last_roll * M_PI / 180.0, roll * M_PI / 180.0) * 180.0 / M_PI;
//     double delta_pitch = angles::shortest_angular_distance(last_pitch * M_PI / 180.0, pitch * M_PI / 180.0) * 180.0 / M_PI;
//     double delta_yaw = angles::shortest_angular_distance(last_yaw * M_PI / 180.0, yaw * M_PI / 180.0) * 180.0 / M_PI;
//     roll = last_roll + delta_roll;
//     pitch = last_pitch + delta_pitch;
//     yaw = last_yaw + delta_yaw;
//     // 更新上一帧的角度
//     last_roll = roll;
//     last_pitch = pitch;
//     last_yaw = yaw;
//     // 将角度归一化到 -180° 到 180°
//     roll = angles::normalize_angle(roll * M_PI / 180.0) * 180.0 / M_PI;
//     pitch = angles::normalize_angle(pitch * M_PI / 180.0) * 180.0 / M_PI;
//     yaw = angles::normalize_angle(yaw * M_PI / 180.0) * 180.0 / M_PI;
//     // 输出插值后的 roll、pitch、yaw 角度
//     // 将插值后的四元数转换为 ROS 消息格式返回
//     return tf2::toMsg(tf2_q);
// }
} // namespace armor_auto_aim


void armor_auto_aim::ArmorPnpSlover::drawPoints(const std::vector<cv::Point2f>& m_armor, cv::Mat& m_frame){
    // 遍历所有点，绘制点对应的索引编号
    for(size_t i = 0; i < m_armor.size(); i++){
        cv::circle(m_frame, m_armor[i], 5, cv::Scalar(255, 255, 255), -1);

        // 转换索引编号为字符串
        std::string index = std::to_string(i);

        // 在点的旁边绘制索引编号
        // 索引编号的文本位置偏移了一个 cv::Point(5, 5)，避免覆盖点本身
        cv::putText(m_frame, index, m_armor[i] + cv::Point2f(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);    
    } 
}

