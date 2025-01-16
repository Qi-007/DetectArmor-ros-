#include "ArmorDetectNode.h"

void FrameMgr::getImage(){
    FrameMgr::camera_status = true;

    HikDriver hik_driver(0);
    if(hik_driver.isConnected()){
        hik_driver.setExposureTime(30000);
        hik_driver.setGain(15);
        hik_driver.showParamInfo();
        hik_driver.startReadThread();
    }

    // while(FrameMgr::camera_status){
        HikFrame Hik_image = hik_driver.getFrame();

        FrameMgr::frame = Hik_image.getRgbFrame()->clone();

        if(frame.empty()){
            std::cout << " frame is empty " << std::endl;
            return;
        }
    // }
}

// void FrameMgr::getImage(){
//     cv::VideoCapture cap(cam0, cv::CAP_V4L2);
//     // 设置 MJPG 格式
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
//     // 设置分辨率为 1280x960
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
//     // 设置帧率为 30 FPS
//     cap.set(cv::CAP_PROP_FPS, 30);
// }
