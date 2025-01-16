find_package(Threads)
find_package(fmt REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)

# 静态库
add_library(hik_driver STATIC include/HikDriver/HikDriver.h src/HikDriver.cpp)
set_target_properties(hik_driver PROPERTIES
        LINKER_LANGUAGE CXX
        )

target_include_directories(hik_driver PUBLIC include)
target_include_directories(hik_driver PUBLIC /opt/MVS/include)
# target_include_directories(hik_driver PUBLIC include/HikDriv)
# target_include_directories(hik_driver PUBLIC hik_sdk/include)
# target_link_directories(hik_driver PUBLIC hik_sdk/lib)

target_link_libraries(hik_driver PUBLIC
    Threads::Threads
    glog::glog
    fmt::fmt
    ${OpenCV_LIBS}
    X11
    MvCameraControl)

# 添加 -fPIC 编译选项 指示编译生成可以在任何内存地址执行的代码 确保hik_driver可以作为共享库进行正确的构建和连接
set_target_properties(hik_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
