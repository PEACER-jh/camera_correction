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
    // double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-5;
    // double fx = 458.654,     fy = 457.296,    cx = 367.251,    cy = 248.375;
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

        // for(int v = 0; v < rows; ++v)
        //     for(int u = 0; u < cols; ++u)
        // {
        //     double x = (u - cx) / fx;
        //     double y = (v - cy) / fy;
        //     double r = std::sqrt(x * x + y * y);
        //     double xd = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
        //     double yd = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
        //     int ud = fx * xd + cx;
        //     int vd = fy * yd + cy;

        //     if(ud >= 0 && vd >= 0 && ud < cols && vd < rows)
        //         dst.at<uchar>(v, u) = frame.at<uchar>((int)vd, (int)ud);
        //     else
        //         dst.at<uchar>(v, u) = 0;
        // }

        if(GrayORColor == 0)
            drawGrayHistogram(dst);
        else if(GrayORColor == 1)
            drawColorHistogram(dst);
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}