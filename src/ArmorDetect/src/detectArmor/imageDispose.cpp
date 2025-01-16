#include "detect.h"

// 通道相减图像的预处理
void imageDispose::minus_dispose(const cv::Mat& m_frame, cv::Mat& m_dst){
    if(m_frame.empty()){
        std::cout << "通道相减图像的预处理: m_frame 为空" << std::endl;
        return;
    }
    imageDispose minus_dispose;

    // 使用高斯函数平滑图像，减少噪声
    cv::Mat blurred, red_minus_blue, blue_minus_red, binary, open, close;
    blurred = minus_dispose.imageGaussion(m_frame);

    // 红蓝通道相减，强调红色区域
    red_minus_blue = minus_dispose.stressRed(blurred);

    // // 蓝红通道相减，强调蓝色区域
    // blue_minus_red = minus_dispose.stressBlue(blurred);

    // 对通道相减图像进行二值化处理
    binary = minus_dispose.imageThreshold(red_minus_blue, threshold::minus_image_threshold);

    // 对二值化后的图像进行开运算处理
    open = minus_dispose.imageOpen(binary);

    // 进行闭运算处理
    close = minus_dispose.imageClose(open);

    // 对闭运算后的图像进行膨胀处理
    m_dst = minus_dispose.imageDilate(close);
}

// 原图像的预处理
void imageDispose::image_dispose(const cv::Mat& m_frame, cv::Mat& m_binaryImage){
    imageDispose frame_dispose;

    // 使用高斯函数平滑图像，减少噪声
    cv::Mat blurred;
    blurred = frame_dispose.imageGaussion(m_frame);

    // 对原图像进行二值化处理
    m_binaryImage = frame_dispose.imageThreshold(blurred, threshold::original_image_threshold);
}


//使用高斯函数平滑图像，减少噪声
cv::Mat imageDispose:: imageGaussion(const cv::Mat& m_frame){
    cv::Mat m_blurred;
    GaussianBlur(m_frame, m_blurred, cv::Size(5, 5), 10, 20);
    return m_blurred;
}

// 通道相减，强调蓝色
cv::Mat imageDispose:: stressBlue(const cv::Mat& m_frame){
    //分离通道
    std::vector<cv::Mat> channels(3);
    split(m_frame, channels);   //分理出B,G,R通道

    // 拉高蓝色通道
    cv::Mat blueChannel = channels[0];
    // blueChannel += 45; // 增加蓝色通道值

    //蓝色通道减去红色通道
    cv::Mat m_blue_minus_red;
    subtract(blueChannel, channels[2], m_blue_minus_red);     //B-R

    //归一化到可视范围
    normalize(m_blue_minus_red, m_blue_minus_red, 0, 255, cv::NORM_MINMAX);

    return m_blue_minus_red;
}

// 通道相减，强调红色
cv::Mat imageDispose:: stressRed(const cv::Mat& m_frame){
    //分离通道
    std::vector<cv::Mat> channels(3);
    split(m_frame, channels);   //分理出B,G,R通道

    // 拉高红色通道
    cv::Mat redChannel = channels[2];
    // redChannel += 45;   // 增加红色通道值

    //红色通道减去蓝色通道
    cv::Mat m_red_minus_blue;
    subtract(redChannel, channels[0], m_red_minus_blue);     //R-B

    //归一化到可视范围
    normalize(m_red_minus_blue, m_red_minus_blue, 0, 255, cv::NORM_MINMAX);

    return m_red_minus_blue;
}

//对彩色/灰色图像进行二值化处理
cv::Mat imageDispose:: imageThreshold(const cv::Mat& frame, const double& thresh){
    cv::Mat m_frame = frame.clone();
    if(m_frame.channels() != 1){
        cvtColor(m_frame, m_frame, cv::COLOR_BGR2GRAY);
    }
    // cout << m_gary.channels() << endl;
    // cvtColor(m_gary, m_gary, COLOR_BGR2GRAY);
    cv::Mat m_binaryImage;
    // 对灰度图进行阈值化处理
    cv::threshold(m_frame, m_binaryImage, thresh, 255, cv::THRESH_BINARY);

    return m_binaryImage;
}

//对二值化图像进行膨胀处理
cv::Mat imageDispose:: imageDilate(const cv::Mat& m_binaryImage){
    // 创建结构元素
    cv::Mat m_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // 存储膨胀结果
    cv::Mat m_dst;
    dilate(m_binaryImage, m_dst, m_kernel);

    return m_dst;
}

// 对二值化图像进行闭运算处理
cv::Mat imageDispose:: imageClose(const cv::Mat& binary){
    // 创建结构元素
    cv::Mat m_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::Mat close;
    morphologyEx(binary, close, cv::MORPH_CLOSE, m_kernel);

    return close;
}

// 对二值化图像进行开运算处理
cv::Mat imageDispose:: imageOpen(const cv::Mat& binary){
    // 创建结构元素
    cv::Mat m_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::Mat open;
    morphologyEx(binary, open, cv::MORPH_OPEN, m_kernel);

    return open;
}