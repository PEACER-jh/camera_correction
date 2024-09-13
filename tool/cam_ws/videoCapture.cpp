#include <cmath>
#include <future>
#include <thread>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

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

    // std::thread raw_thread ,pro_thread;
    // std::promise<cv::Mat> raw_promise, pro_promise;
    // std::future<cv::Mat> raw_future = raw_promise.get_future();
    // std::future<cv::Mat> pro_future = pro_promise.get_future();

    cv::Mat frame;
    bool GrayORColor = 1; // 0 for gray, 1 for color
    while(cap.isOpened())
    {
        cap >> frame;
        if(frame.empty()){
            std::cout << "Error reading frame!" << std::endl;
            return -2;
        }
        int rows = frame.rows;
        int cols = frame.cols;
        cv::Mat src(rows, cols, CV_8UC3) ,dst(rows, cols, CV_8UC3);
        src = frame.clone();
        dst = frame.clone();

        bool Raw = true;
        cv::Mat hist;
        if(GrayORColor == 0){
            // raw_thread = std::thread([&raw_promise, &dst]()
            //     {raw_promise.set_value(drawGrayHistogram(dst));});
            // hist = raw_future.get();
            hist = drawGrayHistogram(src, Raw);
        }
        else if(GrayORColor == 1){
            // raw_thread = std::thread([&raw_promise, &dst]()
            //     {raw_promise.set_value(drawColorHistogram(dst));});
            // hist = raw_future.get();
            hist = drawColorHistogram(src, Raw);
        }

        // 自动曝光
        cv::Mat AE(dst.size(), CV_8UC3);
        AutoExposureMode mode = AutoExposureMode::CENTER_WEIGHTED_METERING;
        auto currAE = std::chrono::steady_clock::now();
        AE = autoExposure(dst, hist, mode);
        std::chrono::duration<double> costAE = std::chrono::steady_clock::now() - currAE;
        std::cout << "auto exposure cost: " << 1.0 / costAE.count() << " fps" << std::endl;
        drawColorHistogram(AE, !Raw);
        // drawGrayHistogram(AE, !Raw);
        // pro_thread = std::thread([&AE](){drawGrayHistogram(AE);});
        
        // raw_thread.join();
        // pro_thread.join();
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}