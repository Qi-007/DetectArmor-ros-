#ifndef ARMOR_TRACKER_NODE_H
#define ARMOR_TRACKER_NODE_H

#include <eigen3/Eigen/Dense>  

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h> 

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

#include "Ekf.hpp"
#include "armor_interfaces/msg/armor.hpp"
#include "armor_interfaces/msg/armors.hpp"


template class Ekf<double, 6, 2>;  // 显式实例化模板

class ArmorTrackerNode : public rclcpp::Node{
public:
    ArmorTrackerNode() : Node("armor_tracker"), last_time_(this->now()) {
        RCLCPP_INFO(this->get_logger(), "ArmorTrackerNode initialized. Timestamp: %ld", last_time_.nanoseconds());
    }
    ArmorTrackerNode(const rclcpp::NodeOptions &options);
    void subArmorsCallback(const armor_interfaces::msg::Armors::SharedPtr armors_msg);
 
    void subFrameCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    
private:
    void initEkf();

    rclcpp::Subscription<armor_interfaces::msg::Armors>::SharedPtr m_armors_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_frame_sub; 

    std::shared_ptr<Ekf<double, 6, 2>> ekf_; // 卡尔曼滤波器对象

    rclcpp::Time last_time_;
    bool is_first_measurement_ = true;
    Eigen::Matrix<double, 6, 1> latest_predicted_state_;  // 缓存最近一次预测结果
    bool has_predicted_state_ = false;        // 标记是否已有预测数据

    double u_vel_ = 0.0;
    double v_vel_ = 0.0;
    double u_a_ = 0.0;
    double v_a_ = 0.0;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

    Eigen::Matrix<double, 6, 1> predicted_state;
    Eigen::Matrix<double, 6, 1> update_state;

};

#endif //