#include "ArmorDetectNode.h"
#include "detect.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <cv_bridge/cv_bridge.h>

ArmorDetectNode::ArmorDetectNode(const rclcpp::NodeOptions &options) : 
    Node("armor_detector", options)
{ 
    std::cout << "Construct ArmorDetectNode" << std::endl;
    // m_armordetect = std::make_shared<ArmorDetect>();
    // m_pnpslover = std::make_shared<PnpSlover>();

    // 创建发布者 “armor_detector/armors”
    m_armors_publish = this->create_publisher<armor_interfaces::msg::Armor>("armor_detector/armors", rclcpp::SensorDataQoS().keep_last(10));

    // 创建订阅者
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video_frames", 
        10,
        std::bind(&ArmorDetectNode::process_frame, this, std::placeholders::_1));

}

    void ArmorDetectNode::FrameCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        if (!img_msg || img_msg->data.empty()){
            RCLCPP_WARN(this->get_logger(), "Received an invalid or empty image!");
            return;
        }   

        rclcpp::Time current_time = img_msg->header.stamp;

        // 检测是否是第一帧
        if (last_frame_time_.nanoseconds() == 0) {
            last_frame_time_ = current_time;
            ArmorDetectNode::process_frame(img_msg);
            return;
        }

        // 计算时间间隔
        double time_diff = (current_time - last_frame_time_).seconds();

        // 检测跳帧, 不处理 
        if (time_diff > expected_interval_ * max_skip_factor_) {
            RCLCPP_WARN(this->get_logger(),
                        "Frame jump detected! Time diff: %.3f seconds. Skipping to the latest frame.",
                        time_diff);

            // 更新时间到最新帧
            last_frame_time_ = current_time;
            return; // 跳过处理该帧

        } else {
            // 正常处理帧
            last_frame_time_ = current_time;
            process_frame(img_msg);
        }       
    }

    void ArmorDetectNode::process_frame(const sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        // 获取图像数据并处理
        try {
            cv::Mat cv_image = cv_bridge::toCvShare(img_msg, "bgr8")->image;

            // cv::imshow("前哨站", cv_image);
            // cv::waitKey(1);

            // 处理图像进行目标检测
            ArmorDetect detector;
            detector.imageDetect(cv_image);

            // 获取 Armor 信息
            const float& distence = ArmorInformation::getInstance().getDistance();
            const std::vector<cv::Point2f>& armor_points = ArmorInformation::getInstance().getApexs();
            const cv::Mat& tvec = ArmorInformation::getInstance().getTvec();
            const cv::Mat& rvec = ArmorInformation::getInstance().getRvec();

            // 将 rvec 转换为旋转矩阵
            cv::Mat rotation_matrix;
            if (rvec.empty()) {
                std::cerr << "rvec is empty, skipping this iteration." << std::endl;
                return;
            }
            cv::Rodrigues(rvec, rotation_matrix);

            // 将旋转矩阵转换为四元数
            tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
            );

            tf2::Quaternion quaternion;
            tf2_rotation_matrix.getRotation(quaternion);

            // 创建 Pose 消息
            armor_interfaces::msg::Armor armor_msg;
            armor_msg.pose.position.x = tvec.at<double>(0, 0);
            armor_msg.pose.position.y = tvec.at<double>(1, 0);
            armor_msg.pose.position.z = tvec.at<double>(2, 0);
            armor_msg.pose.orientation = tf2::toMsg(quaternion);
            
            // 将 cv::Point2f 转换为 geometry_msgs::msg::Point32
            std::vector<geometry_msgs::msg::Point32> msg_points;
            for (const auto& cv_point : armor_points) {
                geometry_msgs::msg::Point32 msg_point;
                msg_point.x = cv_point.x;
                msg_point.y = cv_point.y;
                msg_point.z = 0.0f;  // 默认值，通常设置为 0，因为 cv::Point2f 没有 z 坐标
                msg_points.push_back(msg_point);
            }

            armor_msg.distance_to_center = distence;
            armor_msg.apexs = msg_points;

            // 发布消息
            m_armors_publish->publish(armor_msg);

            ArmorInformation::getInstance().setTvec({});
            ArmorInformation::getInstance().setRvec({});

        } catch (const cv_bridge::Exception& e) {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        }
    }

    // 设置数据
    void ArmorInformation::setNumber(const int& number){
        m_number = number;
    }
    void ArmorInformation::setType(const std::string& type){
        m_type = type;
    }    
    void ArmorInformation::setColor(const std::string& color){
        m_color = color;
    }     
    void ArmorInformation::setDistance(const float& distance_to_center){
        // std::cout << "Successful setDistance" << std::endl;
        m_distance_to_center = distance_to_center;
    }     
    void ArmorInformation::setApexs(const std::vector<cv::Point2f>& apexs){
        m_apexs = apexs;
    }     
    void ArmorInformation::setTvec(const cv::Mat& tvec){
        m_tvec = tvec.clone();
    }     
    void ArmorInformation::setRvec(const cv::Mat& rvec){
        m_rvec = rvec.clone();
    }

    // 获取数据
    const std::uint8_t& ArmorInformation::getNumber() const{
        return ArmorInformation::m_number;
    } 
    const std::string& ArmorInformation::getType() const{
        return ArmorInformation::m_type;
    }
    const std::string& ArmorInformation::getColor() const{
        return ArmorInformation::m_color;
    }
    const float& ArmorInformation::getDistance() const{
        return ArmorInformation::m_distance_to_center;
    }
    const std::vector<cv::Point2f>& ArmorInformation::getApexs() const{
        return ArmorInformation::m_apexs;
    }
    const cv::Mat& ArmorInformation::getTvec() const{
        return ArmorInformation::m_tvec;
    }
    const cv::Mat& ArmorInformation::getRvec() const{
        return ArmorInformation::m_rvec;
    }

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorDetectNode)

