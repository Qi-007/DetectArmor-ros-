#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriberNode : public rclcpp::Node {
public:
    ImageSubscriberNode() : Node("image_subscriber") {
        // 创建订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video_frames", 10, 
            std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1));
    }

private:
    // 回调函数：接收图像信息
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 将 ROS 图像消息转换为 OpenCV 图像
            cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // 显示图像
            cv::imshow("Received Image", image);
            cv::waitKey(10); // 必须调用，否则窗口无法刷新
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);

    // 创建节点并运行
    rclcpp::spin(std::make_shared<ImageSubscriberNode>());

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}
