// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>

// class ImageSubscriberNode : public rclcpp::Node {
// public:
//     cv::Mat frame;

// public:
//     ImageSubscriberNode() : Node("image_subscriber_node") {
//         // 创建订阅者
//         image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "image_raw", 10,
//             std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1));
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

//     void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//         try {
//             // 将 ROS 图像消息转换为 OpenCV 图像
//             ImageSubscriberNode::frame = cv_bridge::toCvCopy(msg, "rgb8")->image;

//             // 显示图像
//             cv::imshow("Received Image", frame);
//             cv::waitKey(30); // 必须调用以刷新图像窗口
//         } catch (cv_bridge::Exception &e) {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         }
//     }
// };
