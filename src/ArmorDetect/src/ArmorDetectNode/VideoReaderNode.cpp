#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoReaderNode : public rclcpp::Node {
public:
    VideoReaderNode()
        : Node("video_reader_node"), video_capture_() {
        if (!video_capture_.open("/home/zjq/Desktop/MVS_video/short.avi")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source.");
            rclcpp::shutdown();
            return;
        }

        // 创建一个 ROS 2 发布者
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("video_frames", 50);

        // 创建定时器，每帧读取间隔为 30ms（33fps）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VideoReaderNode::readFrame, this));
    }

private:
    void readFrame() {
        cv::Mat frame;
        video_capture_ >> frame; // 读取一帧

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty frame.");
            return;
        }

        // 转换 OpenCV 图像为 ROS 2 消息
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->get_clock()->now();

        // 发布图像消息
        image_pub_->publish(*msg);

        // 在终端显示帧
        cv::imshow("Video Frame", frame);
        cv::waitKey(1); // 必须调用以刷新窗口
    }

    cv::VideoCapture video_capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoReaderNode>());
    rclcpp::shutdown();
    return 0;
}