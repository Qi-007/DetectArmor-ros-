#ifndef HIK_CAMERA_NODE_H
#define HIK_CAMERA_NODE_H

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <hik_camera_setup.h>

namespace hik_camera {

class HikCameraNode : public rclcpp::Node{
public:
    HikCameraNode(const rclcpp::NodeOptions &options);
private:
    HikDriver m_hik_driver;
    bool m_enable = true;
    image_transport::CameraPublisher m_camera_pub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_enable_srv;
    std::unique_ptr<camera_info_manager::CameraInfoManager> m_camera_info_manager;
    sensor_msgs::msg::CameraInfo m_camera_info_msg;
    sensor_msgs::msg::Image m_image_msg;
    MV_IMAGE_BASIC_INFO m_hik_img_info;
    std::thread m_core_thread;

    void initHikCamera();

    void core();
};
} // namespace hik_camera

#endif // HIK_CAMERA_NODE_H