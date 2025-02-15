#include "ArmorTrackerNode.h"
#include "Ekf.hpp"
#include "cv_bridge/cv_bridge.h"

#include <armor_interfaces/msg/armor.hpp>  // 使用稳定的 ROS2 头文件
#include <Eigen/Dense>  // 只保留这个，去掉冗余 Eigen 头文件

#include <iostream>
#include <rclcpp/rclcpp.hpp>  // 这个已经包含 logging 和 utilities
#include <vector>
#include "geometry_msgs/msg/point.hpp"



typedef geometry_msgs::msg::Point32 PointType; 
const int S_NUM = 6; // 状态维度 [x, y, vx, vy, ax, ay]
const int M_NUM = 2; // 测量维度 [x, y]

// 相机内参
std::array<double, 9> intrinsic_matrix = {1806.202836613486, 0, 706.479880371389,
                                    0, 1812.980528629544, 546.7058549527911,
                                    0, 0, 1};

double fx = intrinsic_matrix[0];  // 焦距 fx
double fy = intrinsic_matrix[4];  // 焦距 fy
double cx = intrinsic_matrix[2];  // 光心 cx
double cy = intrinsic_matrix[5];  // 光心 cy

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
    this->declare_parameter("dt", 0.02);
    this->declare_parameter("u_vel", 0.1);
    this->declare_parameter("v_vel", 0.1);
    this->declare_parameter("u_a", 0.1);
    this->declare_parameter("v_a", 0.1);
    this->declare_parameter("Q", std::vector<double>{
        1, 0, 0.2, 0, 0.5 * 0.2 * 0.2, 0,
        0, 1, 0, 0.2, 0, 0.5 * 0.2 * 0.2,
        0, 0, 1, 0, 0.2, 0,
        0, 0, 0, 1, 0, 0.2,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1 
    });
    this->declare_parameter("R", std::vector<double>{
        1, 0,
        0, 1
    });

    u_vel_ = this->get_parameter("u_vel").as_double();
    v_vel_ = this->get_parameter("v_vel").as_double();
    u_a_ = this->get_parameter("u_a").as_double();
    v_a_ = this->get_parameter("v_a").as_double();

    RCLCPP_INFO(this->get_logger(), "Initial u_vel: %f, v_vel: %f", u_vel_, v_vel_);

    // 参数回调监听
    m_param_callback_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params) {
            for (const auto &param : params) {
                if (param.get_name() == "u_vel") {
                    u_vel_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated u_vel: %f", u_vel_);
                } else if (param.get_name() == "v_vel") {
                    v_vel_ = param.as_double();
                    RCLCPP_INFO(this->get_logger(), "Updated v_vel: %f", v_vel_);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }
    );

    RCLCPP_INFO(this->get_logger(), "ArmorPredictNode is running...");

    // 订阅原始图像 " video_frames " 主题  
    m_frame_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "video_frames", 
        10, 
        std::bind(&ArmorTrackerNode::subFrameCallback, this, std::placeholders::_1)
    );

    // m_armors_sub = this->create_publisher<armor_interfaces::msg::Armor>("armor_detector/armors", rclcpp::SensorDataQoS());
    m_armors_sub = this->create_subscription<armor_interfaces::msg::Armors>(
        "armor_detector/armors", 
        rclcpp::SensorDataQoS(), 
        std::bind(&ArmorTrackerNode::subArmorsCallback, this, std::placeholders::_1)
    );

    initEkf();
}

    void ArmorTrackerNode::initEkf(){
        // 时间间隔
        double delta_t = 0.2;

        // 状态转移矩阵 F
        Eigen::Matrix<double, S_NUM, S_NUM> F;
        F << 1, 0, delta_t, 0, 0.5 * delta_t * delta_t, 0,
            0, 1, 0, delta_t, 0, 0.5 * delta_t * delta_t,
            0, 0, 1, 0, delta_t, 0,
            0, 0, 0, 1, 0, delta_t,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        // 观测矩阵 H
        Eigen::Matrix<double, M_NUM, S_NUM> H;
        H << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;
            
        std::vector<double> Q_vec = this->get_parameter("Q").as_double_array();
        std::vector<double> R_vec = this->get_parameter("R").as_double_array();

        Eigen::Matrix<double, S_NUM, S_NUM> Q;
        Eigen::Matrix<double, M_NUM, M_NUM> R;

        if(Q_vec.size() == S_NUM * S_NUM && R_vec.size() == M_NUM * M_NUM) {
            Q = Eigen::Map<Eigen::Matrix<double, S_NUM, S_NUM>>(Q_vec.data());
            R = Eigen::Map<Eigen::Matrix<double, M_NUM, M_NUM>>(R_vec.data());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter Q or R size mismatch");
            rclcpp::shutdown();
        }

        // 初始误差协方差矩阵 P
        Eigen::Matrix<double, S_NUM, S_NUM> P = Eigen::Matrix<double, S_NUM, S_NUM>::Identity();
        // P *= 0.5;

        ekf_ = std::make_shared<Ekf<double, S_NUM, M_NUM>> (F, H, Q, R, P);
    }

void ArmorTrackerNode::subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg) {
    if (!armors_msg) {
        RCLCPP_WARN(this->get_logger(), "Received an empty message!");
        return;
    }

    if(armors_msg->armors.empty()){
        RCLCPP_WARN(this->get_logger(), "No armor data received!");
        return;
    }

    armor_interfaces::msg::Armor armor_msg = armors_msg->armors[0];
    
    PointType center = calculateCenterPoint(armor_msg.apexs);

    // 计算归一化相机坐标
    double u = (center.x - cx) / fx;
    double v = (center.y - cy) / fy;

    // 归一化相机坐标作为测量值输入到 EKF
    Eigen::Matrix<double, M_NUM, 1> measurement;
    measurement << u, v;


    // 如果是首次测量，初始化滤波器
    if (is_first_measurement_) {
        Eigen::Matrix<double, S_NUM, 1> init_state;

        init_state << u, v, u_vel_, v_vel_, u_a_, v_a_;
        
        // 初始化状态
        ekf_->initState(init_state);       

        // 记录首次时间戳
        last_time_ = armors_msg->header.stamp;
        is_first_measurement_ = false;
        return; // 不进行预测/更新，直接退出
    } 

    // rclcpp::Time current_time = armors_msg->header.stamp; 
    // double dt = (current_time - last_time_).seconds();
    // // RCLCPP_INFO(this->get_logger(), "Time interval dt: %f", dt);
    // last_time_ = current_time;

    double dt = this->get_parameter("dt").as_double();

    if (dt < 0.01) {  
        dt = 0.01;
    }

    // 更新 F 矩阵中的 dt
    Eigen::Matrix<double, S_NUM, S_NUM> F;
    F << 1, 0, dt, 0, 0.5 * dt * dt, 0,
         0, 1, 0, dt, 0, 0.5 * dt * dt,
         0, 0, 1, 0, dt, 0,
         0, 0, 0, 1, 0, dt,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
        
    ekf_->predict(dt);
    // 此时的 m_pre_state 是先验值（预测出的值）
    predicted_state = ekf_->getPreState();

    ekf_->update(measurement);
    // 此时的 m_state 是后验值（结合了m_pre_state）
    update_state = ekf_->getState();

    // 更新缓存的预测状态
    latest_predicted_state_ = ekf_->getState();
    has_predicted_state_ = true;
}


    void ArmorTrackerNode::subFrameCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        if (!ekf_) {
            RCLCPP_WARN(this->get_logger(), "EKF is not initialized yet.");
            return;
        }
        try
        {
            // 将 ROS 图像消息转换为 OpenCV 格式
            cv::Mat frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;

            if(has_predicted_state_){
                double up_x = update_state(0);
                double up_y = update_state(1);
                double pre_x = predicted_state(0);
                double pre_y = predicted_state(1);
                // double z = 1.0;  

                // 投影到像素坐标系
                int img_up_x = static_cast<int>(up_x * fx + cx);
                int img_up_y = static_cast<int>(up_y * fy + cy);
                int img_pre_x = static_cast<int>(pre_x * fx + cx);
                int img_pre_y = static_cast<int>(pre_y * fy + cy);

                if (img_up_x >= 0 && img_up_x < frame.cols && img_up_y >= 0 && img_up_y < frame.rows
                    &&img_pre_x >= 0 && img_pre_x < frame.cols && img_pre_y >= 0 && img_pre_y < frame.rows) {
                    // 后验点（绿色）
                    cv::circle(frame, cv::Point(img_up_x, img_up_y), 5, cv::Scalar(0, 255, 0), -1);
                    // 先验点（白色）
                    cv::circle(frame, cv::Point(img_pre_x, img_pre_y), 5, cv::Scalar(255, 255, 255), -1);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Predicted point out of image bounds: (%d, %d)", img_up_x, img_up_y);
                    RCLCPP_WARN(this->get_logger(), "Predicted point out of image bounds: (%d, %d)", img_pre_x, img_pre_y);
                }

            }

            // 显示图像
            cv::imshow("Predicted Position", frame);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorTrackerNode)