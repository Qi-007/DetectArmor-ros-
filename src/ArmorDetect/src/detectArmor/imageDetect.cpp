#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
// #include <thread>
#include "ArmorDetectNode.h"
#include "detect.h"
#include "PnPslover.h"
#include <vector>

// 相机内参
std::array<double, 9> intrinsic_matrix = {1806.202836613486, 0, 706.479880371389,
                                     0, 1812.980528629544, 546.7058549527911,
                                     0, 0, 1};
// 畸变系数
std::vector<double> distortion_vector = {-0.1066716215207354, 1.026102429629, -0.0003129016621888697, -0.001878173941429469, -6.758932699953562};

void ArmorDetect::imageDetect(cv::Mat& frame){    
    // cv::Mat binaryImage,    // 原图处理后的图像
    //         dst;     // 通道相减处理后的图像

    cv::Mat binaryImage = cv::Mat::zeros(frame.size(), CV_8UC1);  
    cv::Mat dst = cv::Mat::zeros(frame.size(), CV_8UC1); 

    std::vector<std::vector<cv::Point>> all_contours;     //未经筛选的轮廓
    std::vector<LightDescriptor> lights;       //筛选过宽高比的矩形
    std::vector<std::pair<LightDescriptor, LightDescriptor>> matching_lights;    //根据倾斜角等筛选出的配对灯条
    std::vector<std::pair<LightDescriptor, LightDescriptor>> foundArmor;      // 识别后的装甲板

    // imageDispose::image_dispose(frame, binaryImage);

    // imageDispose::minus_dispose(frame, dst);

    // cv::imshow("前哨站", frame);

    // 通道相减图像的二值化处理
    std::thread minusDispose(imageDispose::minus_dispose, std::ref(frame), std::ref(dst));

    // 原图像的二值化处理
    std::thread frameDispose(imageDispose::image_dispose, std::ref(frame), std::ref(binaryImage));

    // 等待线程执行完毕
    minusDispose.join();
    frameDispose.join();

    cv::Mat image = cv::Mat::zeros(frame.size(), CV_8UC1);
    bitwise_and(dst, binaryImage, image);


    // cv::imshow("image", image);

    // // 寻找轮廓
    // findContours(image, all_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // if(all_contours.size() == 0){
    //     cv::imshow("前哨站", frame);
    //     return;
    // }

    // //识别灯条
    // findLightBar all_lightBar;
    // lights = all_lightBar.Lights(all_contours);
    // if(all_contours.size() == 0){
    //     cv::imshow("前哨站", frame);
    //     return;
    // }

    // //匹配灯条
    // matchingLightBar right_lightBar;
    // matching_lights = right_lightBar.matchLight(lights);
    // if(lights.size() == 0){
    //     cv::imshow("前哨站", frame);
    //     return;
    // }

    // // 识别装甲板
    // findArmor armors;
    // foundArmor = armors.find_Armor(matching_lights);
    // if(matching_lights.size() == 0){
    //     cv::imshow("前哨站", frame);
    //     return; 
    // }

    // // 匹配装甲板
    // std::vector<cv::Point2f> armor;
    // matchingArmor all_armors;
    // frame = all_armors.matchingArmors(foundArmor, frame, armor);

    // if(!armor.empty()){ 
    //     cv::Mat tvec, rvec;
    //     rvec.create(3, 1, CV_64F);  // 创建 3x1 的双精度矩阵
    //     tvec.create(3, 1, CV_64F);  // 创建 3x1 的双精度矩阵
    //     armor_auto_aim:: ArmorPnpSlover slover(intrinsic_matrix, distortion_vector);
    //     slover.drawPoints(armor, frame);
    //     slover.pnpSlove(armor, rvec, tvec);
    //     // std::cout <<  "rvec = " << rvec.at<double>(0,0) << " ," << rvec.at<double>(1,0) << " ," << rvec.at<double>(2,0) << std::endl;
    //     // std::cout << "tvec = " << tvec.at<double>(0,0) << " ," << tvec.at<double>(1,0) << " ," << tvec.at<double>(2,0) << std::endl;

    //     // 计算相机距离被测物的实际距离
    //     float distance = sqrt(tvec.at<double>(0,0) * tvec.at<double>(0,0) + tvec.at<double>(1,0) * tvec.at<double>(1,0) + tvec.at<double>(2,0) * tvec.at<double>(2,0)); 
    //     if(distance > 0){
    //         // std::cout << "distance = "<< distance << std::endl;
    //         ArmorInformation::getInstance().setDistance(distance);
    //         ArmorInformation::getInstance().setApexs(armor);
    //         ArmorInformation::getInstance().setTvec(tvec);
    //         ArmorInformation::getInstance().setRvec(rvec);
    
    //         // std::cout <<  "rvec = " << rvec.at<double>(0,0) << " ," << rvec.at<double>(1,0) << " ," << rvec.at<double>(2,0) << std::endl;
    //         // std::cout << "tvec = " << tvec.at<double>(0,0) << " ," << tvec.at<double>(1,0) << " ," << tvec.at<double>(2,0) << std::endl;
    //     }
    // }

    // cv::imshow("前哨站", frame);

    all_contours.clear();       // 清空上一帧的轮廓
    lights.clear();    // 清空上一帧筛选的矩形
    matching_lights.clear();    // 清空上一帧的配对灯条
    foundArmor.clear();     // 清空上一帧的装甲板
    // armor.clear();     

    cv::waitKey(1);

    // cv::destroyAllWindows();

}
