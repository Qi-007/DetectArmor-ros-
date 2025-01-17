cmake_minimum_required(VERSION 3.5)
project(video_reader)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(video_reader_node src/video_reader_node.cpp)

# 链接依赖
ament_target_dependencies(video_reader_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
)

# 安装目标文件
install(TARGETS
    video_reader_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
