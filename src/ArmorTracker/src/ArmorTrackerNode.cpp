#include "ArmorTrackerNode.h"
#include "Ekf.hpp"
#include "cv_bridge/cv_bridge.h"
#include <armor_interfaces/msg/detail/armor__struct.hpp>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <rclcpp/logging.hpp>
#include "geometry_msgs/msg/point.hpp"

typedef geometry_msgs::msg::Point32 PointType; 
const int S_NUM = 4; // 状态维度 [x, y, vx, vy]
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
    this->declare_parameter("u_vel", 0.0);
    this->declare_parameter("v_vel", 0.0);

    u_vel_ = this->get_parameter("u_vel").as_double();
    v_vel_ = this->get_parameter("v_vel").as_double();

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
            return rcl_interfaces::msg::SetParametersResult().set__successful(true);
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
        // 状态定义 [x, y, vx, vy]
        // Eigen::Matrix<double, 1, 4> state_;

        // 时间间隔
        double delta_t = 0.01;

        // 状态转移矩阵 F
        Eigen::Matrix<double, S_NUM, S_NUM> F;
        F << 1, 0, delta_t, 0,
            0, 1, 0, delta_t,
            0, 0, 1, 0,
            0, 0, 0, 1;

        // 观测矩阵 H
        Eigen::Matrix<double, M_NUM, S_NUM> H;
        H << 1, 0, 0, 0,
            0, 1, 0, 0;
            
        // // 将 FJacobi 和 HJacobi设为单位矩阵
        // Eigen::Matrix<double, 4, 4> FJacobi = Eigen::Matrix<double, 4, 4>::Identity();
        // Eigen::Matrix<double, 2, 4> HJacobi = Eigen::Matrix<double, 2, 4>::Identity();

        // // 过程噪声协方差矩阵 Q
        // Eigen::Matrix<double, 4, 4> Q;
        // Q << 0.01, 0, 0.001, 0,
        // 0, 0.01, 0, 0.001,
        // 0.001, 0, 0.01, 0,
        // 0, 0.001, 0, 0.01;

        // // 观测噪声协方差矩阵 R
        // Eigen::Matrix<double, 2, 2> R;
        // R << 0.5, 0,
        //     0, 0.5;


        // 读取 Q 矩阵参数
        auto q_params = this->get_parameter("Q").as_double_array();
        Eigen::Matrix<double, S_NUM, S_NUM> Q;
        Q << q_params[0], q_params[1], q_params[2], q_params[3],
            q_params[4], q_params[5], q_params[6], q_params[7],
            q_params[8], q_params[9], q_params[10], q_params[11],
            q_params[12], q_params[13], q_params[14], q_params[15];

        // 读取 R 矩阵参数
        auto r_params = this->get_parameter("R").as_double_array();
        Eigen::Matrix<double, M_NUM, M_NUM> R;
        R << r_params[0], r_params[1],
            r_params[2], r_params[3];


        // 初始误差协方差矩阵 P
        Eigen::Matrix<double, 4, 4> P = Eigen::Matrix<double, 4, 4>::Identity();
        P *= 0.1;

        // Ekf<4, 2> ekf(F, H, FJacobi, HJacobi, Q, R, P);
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
        // 初始状态: [x, y, vx=0, vy=0]
        Eigen::Matrix<double, S_NUM, 1> init_state;

        init_state << u, v, u_vel_, v_vel_;
        // init_state << 100.0, 100.0, 0.0, 0.0;
        
        // 初始化状态（修正原硬编码错误）
        ekf_->initState(init_state);       

        // 记录首次时间戳
        last_time_ = armors_msg->header.stamp;
        is_first_measurement_ = false;
        return; // 不进行预测/更新，直接退出
    } 

    rclcpp::Time current_time = armors_msg->header.stamp; 
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0 || dt > 1.0) {  // 避免异常情况
        dt = 0.1;
    }

    // 更新 F 矩阵中的 dt
    Eigen::Matrix<double, 4, 4> F;
    F << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
        
    ekf_->setF(F); // 确保 Ekf 类支持动态更新 F

    // // 当前测量值
    // Eigen::Matrix<double, M_NUM, 1> measurement;
    // measurement << center.x, center.y;
    
    // 更新卡尔曼滤波器
    ekf_->predict();
    ekf_->update(measurement);

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

                // 获取预测状态（归一化相机坐标）
                Eigen::Vector4d predicted_state = ekf_->getState();

                // 获取预测状态 [x, y]，但需要知道 z
                double x = predicted_state(0);
                double y = predicted_state(1);
                double z = 1.0;  

                // 投影到像素坐标系
                int img_x = static_cast<int>((x / z) * fx + cx);
                int img_y = static_cast<int>((y / z) * fy + cy);

                // 边界检查，防止越界
                if (img_x < 0 || img_x >= frame.cols || img_y < 0 || img_y >= frame.rows) {
                    RCLCPP_WARN(this->get_logger(), "Predicted point out of image bounds: (%d, %d)", img_x, img_y);
                    return;
                }

                // 绘制预测点（绿色实心圆）
                cv::circle(frame, cv::Point(img_x, img_y), 5, cv::Scalar(0, 255, 0), -1);

                // // 绘制轨迹（历史点连线）
                // static std::vector<cv::Point> trajectory;
                // trajectory.push_back(cv::Point(img_x, img_y));
                // if (trajectory.size() > 30) trajectory.erase(trajectory.begin()); // 保留最近30帧
                // for (size_t i = 1; i < trajectory.size(); ++i) {
                //     cv::line(frame, trajectory[i-1], trajectory[i], cv::Scalar(255, 255, 255), 2);
                // }
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