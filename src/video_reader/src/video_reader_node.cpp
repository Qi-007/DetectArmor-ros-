#include <opencv2/highgui.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoReaderNode : public rclcpp::Node {
public:
    VideoReaderNode()
        : Node("video_reader_node"), video_capture_() {
        // 打开视频文件
        if (!video_capture_.open("/home/zjq/Desktop/MVS_video/short.avi")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file.");
            rclcpp::shutdown();
            return;
        }

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
                   .reliable()  
                   .keep_last(10); 

        // 创建图像发布者
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_frames", qos);

        // 创建定时器，控制发布频率（帧率）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(38),  // 假设视频帧率为 35 FPS
            std::bind(&VideoReaderNode::publishFrame, this));

    }

private:
    void publishFrame() {
        cv::Mat frame;
        video_capture_ >> frame; // 读取视频帧

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "No more frames available.");
            rclcpp::shutdown(); // 关闭节点
            return;
        }

        // cv::imshow("video_frame", frame);
        // cv::waitKey(1);

        // 转换为 ROS 图像消息
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        // 设置头信息
        img_msg->header.stamp = this->get_clock()->now();
        img_msg->header.frame_id = "video_frame";

        // 发布图像消息
        image_publisher_->publish(*img_msg);
    }

    cv::VideoCapture video_capture_;  // OpenCV 视频读取器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_; // 图像发布者
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoReaderNode>());
    rclcpp::shutdown();
    return 0;
}
