#include "ArmorTrackerNode.h"
#include "cv_bridge/cv_bridge.h"
#include <eigen3/Eigen/Core>
#include <iostream>
#include "geometry_msgs/msg/point32.hpp"


typedef geometry_msgs::msg::Point32 PointType; 

PointType calculateCenterPoint(const std::vector<PointType>& corners) {
    PointType center;

    if (corners.size() != 4) {
        throw std::invalid_argument("Expected exactly 4 corners to calculate the center point.");
    }

    // 初始化中心点
    center.x = 0.0;
    center.y = 0.0;
    center.z = 0.0; // 如果是2D平面，可忽略z坐标

    // 累加每个顶点的坐标
    for (const auto& corner : corners) {
        center.x += corner.x;
        center.y += corner.y;
        center.z += corner.z; // 对于2D平面，z坐标可以忽略
    }

    // 计算平均值
    center.x /= 4.0;
    center.y /= 4.0;
    center.z /= 4.0; // 对于2D平面，可忽略z坐标

    return center;
}

void printApexs(const std::vector<PointType>& apexs){
    std::cout << "Apexs points:" << std::endl;
    for(size_t i = 0; i < apexs.size(); ++i){
        const auto &point = apexs[i];
        std::cout << "Point " << i + 1 << ": ("
                  << "x = " << point.x << ", "
                  << "y = " << point.y << ", "
                  << "z = " << point.z << ")"
                  << std::endl; 
    } 
}

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions &options) : 
    Node("armor_tracker", options)
{
    RCLCPP_INFO(this->get_logger(), "ArmorPredictNode is running...");

    // 订阅原始图像 " video_frames " 主题  
    m_frame_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "video_frames", 
        10, 
        std::bind(&ArmorTrackerNode::subFrameCallback, this, std::placeholders::_1)
    );

    // m_armors_sub = this->create_publisher<armor_interfaces::msg::Armor>("armor_detector/armors", rclcpp::SensorDataQoS());
    m_armors_sub = this->create_subscription<armor_interfaces::msg::Armor>(
        "armor_detector/armors", 
        rclcpp::SensorDataQoS(), 
        std::bind(&ArmorTrackerNode::subArmorsCallback, this, std::placeholders::_1)
    );

        // 初始化卡尔曼滤波器
        const int S_NUM = 4; // 状态维度 [x, y, vx, vy]
        const int M_NUM = 2; // 测量维度 [x, y]

        Eigen::Matrix<double, S_NUM, S_NUM> F = Eigen::Matrix<double, S_NUM, S_NUM>::Identity();
        F(0, 2) = 1.0; // x_new = x + vx
        F(1, 3) = 1.0; // y_new = y + vy

        Eigen::Matrix<double, M_NUM, S_NUM> H;
        H << 1, 0, 0, 0, 
             0, 1, 0, 0;

        Eigen::Matrix<double, S_NUM, S_NUM> Q = Eigen::Matrix<double, S_NUM, S_NUM>::Identity() * 0.01;
        Eigen::Matrix<double, M_NUM, M_NUM> R = Eigen::Matrix<double, M_NUM, M_NUM>::Identity() * 0.1;
        Eigen::Matrix<double, S_NUM, S_NUM> P = Eigen::Matrix<double, S_NUM, S_NUM>::Identity();

        ekf_ = std::make_shared<Ekf<S_NUM, M_NUM>>(F, H, F, H, Q, R, P);

        // 初始化状态
        Eigen::Matrix<double, 1, S_NUM> init_state;
        init_state << 0, 0, 0, 0;
        ekf_->initState(init_state);

}

    void ArmorTrackerNode::initEkf(){
        // const int state_dim = 4;
        // const int measurement_dim = 2;

        // 状态定义 [x, y, vx, vy]
        Eigen::Matrix<double, 1, 4> state_;

        // 时间间隔
        double delta_t = 0.01;

        // 状态转移矩阵 F
        Eigen::Matrix<double, 4, 4> F;
        F << 1, 0, delta_t, 0,
            0, 1, 0, delta_t,
            0, 0, 1, 0,
            0, 0, 0, 1;

        // 观测矩阵 H
        Eigen::Matrix<double, 2, 4> H;
        H << 1, 0, 0, 0,
            0, 1, 0, 0;
            
        // 将 FJacobi 和 HJacobi设为单位矩阵
        Eigen::Matrix<double, 4, 4> FJacobi = Eigen::Matrix<double, 4, 4>::Identity();
        Eigen::Matrix<double, 2, 4> HJacobi = Eigen::Matrix<double, 2, 4>::Identity();

        // 过程噪声协方差矩阵 Q
        Eigen::Matrix<double, 4, 4> Q;
        Q << 0.5, 0, 0, 0,
            0, 0.5, 0, 0,
            0, 0, 0.1, 0,
            0, 0, 0, 0.1;

        // 观测噪声协方差矩阵 R
        Eigen::Matrix<double, 2, 2> R;
        R << 1.0, 0,
            0, 1.0;

        // 初始误差协方差矩阵 P
        Eigen::Matrix<double, 4, 4> P = Eigen::Matrix<double, 4, 4>::Identity();

        Ekf<4, 2> ekf(F, H, FJacobi, HJacobi, Q, R, P);
    }

void ArmorTrackerNode::subArmorsCallback(const armor_interfaces::msg::Armor::SharedPtr armors_msg) {
    if (!armors_msg) {
        RCLCPP_WARN(this->get_logger(), "Received an empty message!");
        return;
    }

    PointType center = calculateCenterPoint(armors_msg->apexs);

    // 当前测量值
    Eigen::Matrix<double, 1, 2> measurement;
    measurement << center.x, center.y;

    // 更新卡尔曼滤波器
    ekf_->predict();
    ekf_->update(measurement);

    // 获取预测状态
    Eigen::Matrix<double, 1, 4> predicted_state = ekf_->getState();

    // // 构造预测结果消息
    // geometry_msgs::msg::PointStamped predicted_msg;
    // predicted_msg.header.stamp = this->get_clock()->now();
    // predicted_msg.header.frame_id = "map";
    // predicted_msg.point.x = predicted_state(0, 0); // x
    // predicted_msg.point.y = predicted_state(0, 1); // y
    // predicted_msg.point.z = 0.0;

    // // 发布预测结果
    // predicted_armor_publisher_->publish(predicted_msg); 

    std::cout << "predicted_state.x: " << predicted_state.x() << std::endl;
}


    void ArmorTrackerNode::subFrameCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        try
        {
            // 将 ROS 图像消息转换为 OpenCV 格式
            cv::Mat frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;

            // 获取预测状态
            Eigen::Matrix<double, 1, 4> predicted_state = ekf_->getState();

            double predicted_x = predicted_state(0, 0); // 提取 x 坐标
            double predicted_y = predicted_state(0, 1);

            // 在图像上绘制预测点
            cv::Point predicted_point(predicted_x, predicted_y);
            cv::circle(frame, predicted_point, 5, cv::Scalar(0, 255, 0), -1); // 绿色圆点

            // 显示图像
            cv::imshow("Predicted Image", frame);
            cv::waitKey(1);

        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorTrackerNode)