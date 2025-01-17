cmake_minimum_required(VERSION 3.5)
project(image_subscriber)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 找到依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(image_subscriber_node src/ImageSubscriberNode.cpp)

# 链接依赖
ament_target_dependencies(image_subscriber_node rclcpp sensor_msgs cv_bridge OpenCV)

# 安装可执行文件
install(TARGETS
  image_subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
