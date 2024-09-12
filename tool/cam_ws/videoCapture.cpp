#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "./include/drawHistogram.hpp"

int main()
{
    cv::VideoCapture cap;
    cap.open(0);
    if(!cap.isOpened()){
        std::cout << "Error opening video stream!" << std::endl;
        return -1;
    }

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
        cv::Mat dst(rows, cols, CV_8UC3);
        dst = frame.clone();

        cv::Mat hist;
        if(GrayORColor == 0)
        {

            drawGrayHistogram(dst);
        }
        else if(GrayORColor == 1)
        {

            drawColorHistogram(dst);
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}