#include <cmath>
#include <future>
#include <thread>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "./include/drawHistogram.hpp"
#include "./include/autoExposure.hpp"

int main()
{
    cv::VideoCapture cap;
    cap.open(0);
    if(!cap.isOpened()){
        std::cout << "Error opening video stream!" << std::endl;
        return -1;
    }

    std::thread raw_thread ,pro_thread;
    std::promise<cv::Mat> raw_promise, pro_promise;
    std::future<cv::Mat> raw_future = raw_promise.get_future();
    std::future<cv::Mat> pro_future = pro_promise.get_future();

    cv::Mat frame;
    bool GrayORColor = 0; // 0 for gray, 1 for color
    while(cap.isOpened())
    {
        cap >> frame;
        if(frame.empty()){
            std::cout << "Error reading frame!" << std::endl;
            return -2;
        }
        int rows = frame.rows;
        int cols = frame.cols;
        cv::Mat dst(rows, cols, CV_8UC3);
        dst = frame.clone();

        cv::Mat hist;
        if(GrayORColor == 0){
            raw_thread = std::thread([&raw_promise, &dst]()
                {raw_promise.set_value(drawGrayHistogram(dst));});
            hist = raw_future.get();
        }
        else if(GrayORColor == 1){
            raw_thread = std::thread([&raw_promise, &dst]()
                {raw_promise.set_value(drawColorHistogram(dst));});
            hist = raw_future.get();
        }

        // 自动曝光
        cv::Mat AE(dst.size(), CV_8UC3);
        AutoExposureMode mode = AutoExposureMode::AVERAGE_METERING;
        AE = autoExposure(dst, hist, mode);
        pro_thread = std::thread([&AE](){drawGrayHistogram(AE);});
        
        raw_thread.join();
        pro_thread.join();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}