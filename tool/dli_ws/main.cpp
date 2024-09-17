#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 打开默认摄像头（0 表示第一个摄像头）
    cv::VideoCapture cap(0);

    // 检查是否成功打开摄像头
    if (!cap.isOpened()) {
        std::cout << "无法打开摄像头" << std::endl;
        return -1;
    }

    cv::Mat frame;
    char pressedKey;

    while (true) {
        // 从摄像头捕获一帧
        cap >> frame;

        // 检查帧是否为空（如果视频结束）
        if (frame.empty()) {
            std::cout << "无法读取帧" << std::endl;
            break;
        }

        // 显示捕获的帧
        cv::imshow("Video Stream", frame);

        // 等待 30 毫秒，检查用户是否按下键
        // 如果按下键是 'q'，则退出循环
        pressedKey = cv::waitKey(30);
        if (pressedKey == 'q') {
            break;
        }
    }

    // 释放摄像头资源
    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}
