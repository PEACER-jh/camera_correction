#include <cmath>
#include <vector>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

int main() 
{
    cv::namedWindow("FRAME_LEFT", cv::WINDOW_NORMAL);
    cv::namedWindow("FRAME_RIGHT", cv::WINDOW_NORMAL);
    cv::Mat f0, f1;
    std::vector<cv::VideoCapture> capture;
    
    int capture_id = 0;
    cv::VideoCapture cap;
    for(int i = 0; i < 4; i++){
        if(capture_id == 2)
            break;
        try {
            cap.open(i, cv::CAP_V4L2);
            if(cap.isOpened()){
                capture.push_back(cap);
                capture_id++;
                std::cout << "capture: " << capture_id << " is opened" << std::endl;
            }
        } catch(cv::Exception& e) {
            std::cout << "Error opening video stream or file" << std::endl;
            std::cout << e.what() << std::endl; 
        }
    }
    std::cout << "capture number: " << capture.size() << std::endl;
    cv::VideoCapture c0 = capture.front();
    cv::VideoCapture c1 = capture.back();

    while(c0.isOpened() && c1.isOpened())
    {
        c0 >> f0;
        c1 >> f1;
        cv::imshow("FRAME_LEFT", f0);
        cv::imshow("FRAME_RIGHT", f1);
        cv::waitKey(100);
    }

    c0.release();
    c1.release();
    cv::destroyAllWindows();
    return 0;
}