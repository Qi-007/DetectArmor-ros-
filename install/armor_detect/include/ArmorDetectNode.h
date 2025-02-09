#ifndef ARMOR_DETECT_NODE_H
#define ARMOR_DETECT_NODE_H

#include <armor_interfaces/msg/detail/armor__struct.hpp>
#include <atomic>
#include <iostream>
#include <rclcpp/qos_event.hpp>
#include <thread>
#include <memory.h>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "HikDriver/HikDriver.h"
#include "armor_interfaces/msg/armor.hpp"
#include "armor_interfaces/msg/armors.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FrameMgr {
public:
    void getImage();
public:
    // 相机获取的原图像
    cv::Mat frame;
    // 相机开关状态
    bool camera_status = false; 
};

class ArmorDetect {
public:
    void imageDetect(cv::Mat& frame);
};

class PnpSlover {
public:
    void pnpSlover(cv::Mat* rvec, cv::Mat* tvec, const std::vector<cv::Point2f>& points);
};

class ArmorDetectNode : public rclcpp::Node{
public:
    ArmorDetectNode(const rclcpp::NodeOptions &options);
private:
    //订阅图像节点
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Time last_frame_time_; // 记录上一帧的时间戳
    const double expected_interval_ = 0.033; // 期望帧间隔（30 FPS 对应 0.033 秒）
    const double max_skip_factor_ = 3.0;     // 最大跳帧倍数


    // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_cam_info_sub;
    std::shared_ptr<FrameMgr> m_frame_mgr;
    std::shared_ptr<ArmorDetect> m_armordetect;
    std::shared_ptr<PnpSlover> m_pnpslover;
    rclcpp::Publisher<armor_interfaces::msg::Armors>::SharedPtr m_armors_publish;
    std::thread m_detect_core;

    // Function
    void FrameCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    void process_frame(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  
};

// 装甲板信息
class ArmorInformation{
public:
    static ArmorInformation& getInstance(){
        static ArmorInformation armor_information;
        return armor_information;
    } // 单例模式确保全局唯一

    // 禁止拷贝和赋值操作
    ArmorInformation(const ArmorInformation&) = delete;
    ArmorInformation& operator=(const ArmorInformation&) = delete;

    void setNumber(const int& number); // 设置数据
    void setType(const std::string& type);     
    void setColor(const std::string& color);     
    void setDistance(const float& distance_to_center);     
    void setApexs(const std::vector<cv::Point2f>& apexs);     
    void setTvec(const cv::Mat& tvec);     
    void setRvec(const cv::Mat& rvec);    

    const std::uint8_t& getNumber() const;   // 获取数据
    const std::string& getType() const;
    const std::string& getColor() const;
    const float& getDistance() const;
    const std::vector<cv::Point2f>& getApexs() const;
    const cv::Mat& getTvec() const;
    const cv::Mat& getRvec() const;

private:
    ArmorInformation() = default;       // 构造函数私有化
    std::uint8_t m_number;
    std::string m_type;
    std::string m_color;
    float m_distance_to_center;       // 装甲板距相机的距离
    std::vector<cv::Point2f> m_apexs;       // 装甲板特征点           // 这可能会有问题
    cv::Mat m_tvec;         // 相机坐标系下装甲板位姿 
    cv::Mat m_rvec;
};




#endif // ARMOR_DETECT_NODE_H