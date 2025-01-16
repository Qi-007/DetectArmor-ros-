cmake_minimum_required(VERSION 3.17)
project(armor_detect)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(HikDriver)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message("-- CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# 添加
# 如果路径中没有特殊字符（如空格、特殊符号）或 CMake 变量，需要或可以不加双引号
# 如果路径中包含特殊字符（特别是空格），则必须使用双引号，以防止路径被错误分割或解释
set(CMAKE_PREFIX_PATH "/opt/ros/humble/share/cv_bridge/cmake")  
set(CMAKE_PREFIX_PATH "/usr/local/runtime/cmake")
find_package(ament_cmake_auto REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)  

ament_auto_find_build_dependencies()

# 此命令只需要包含.cpp文件会自动在相应目录结构下寻找头文件
ament_auto_add_library(${PROJECT_NAME} SHARED  
  DIRECTORY src
)

##
# add_executable(image_subscriber_node src/ArmorDetectNode/ImageSubscriberNode.cpp)
# target_link_libraries(image_subscriber_node ${OpenCV_LIBS})
# ament_target_dependencies(image_subscriber_node rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(video_reader_node src/ArmorDetectNode/VideoReaderNode.cpp)
target_link_libraries(video_reader_node ${OpenCV_LIBS})
ament_target_dependencies(video_reader_node rclcpp sensor_msgs cv_bridge OpenCV)
##

target_include_directories(${PROJECT_NAME} PUBLIC include )
target_include_directories(${PROJECT_NAME} PUBLIC HikDriver)
target_include_directories(${PROJECT_NAME} PUBLIC hik_sdk/include)
target_include_directories(${PROJECT_NAME} PUBLIC /opt/MVS/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenVINO_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/ArmorDetect/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/ArmorDetect/HikDriver/hik_sdk/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/src/ArmorDetect/HikDriver/include/HikDriver)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  # hik_camera
  armor_interfaces
  OpenCV
)

target_link_libraries(${PROJECT_NAME} 
    ${OpenCV_LIBS}    
    # hik_camera
    hik_driver
    Eigen3::Eigen
    openvino::runtime)

target_link_libraries(${PROJECT_NAME} 
    X11
    MvCameraControl
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ArmorDetectNode"
  # EXECUTABLE image_subscriber_node
  EXECUTABLE ${PROJECT_NAME}_node
)

# # 连接到静态库hik_driver
# add_library(hik_driver STATIC IMPORTED)
# set_target_properties(hik_driver PROPERTIES
#     IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libhik_driver.a
# )
# target_include_directories(hik_driver INTERFACE
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )

# ##
# install(TARGETS
#   image_subscriber_node
#   DESTINATION lib/${PROJECT_NAME}
# )
# ##

##
install(TARGETS
  video_reader_node
  DESTINATION lib/${PROJECT_NAME}
)
##

ament_auto_package(
INSTALL_TO_SHARE
  launch
)