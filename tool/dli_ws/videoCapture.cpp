#include <vector>
#include <chrono>
#include <iostream>

#include "include/features.hpp"
#include "include/cameraRegister.hpp"

int main()
{
    cv::Mat fl, fr, frame;
    cv::VideoCapture cl, cr;
    std::vector<cv::VideoCapture> captures;
    cameraRegister(captures);
    cameraRegister(captures, cl, cr);

    cv::namedWindow("RAW_FRAME", cv::WINDOW_NORMAL);
    while(cl.isOpened() && cr.isOpened())
    {
        cl >> fl;
        cr >> fr;
        cv::hconcat(fl, fr, frame);
        // cv::imshow("RAW_FRAME", frame);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        computeFeatures_ORB(fl, fr);
        std::chrono::duration<double> time_used = std::chrono::steady_clock::now() - t1;
        std::cout << "******time used: " << 1.0 / time_used.count() << " FPS******" << std::endl;

    }

    cl.release();
    cr.release();
    cv::destroyAllWindows();
    return 0;
}