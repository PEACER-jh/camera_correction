#include <vector>
#include <chrono>
#include <iostream>
#include "include/cameraRegister.hpp"

int main()
{
    cv::Mat fl, fr;
    cv::VideoCapture cl, cr;
    std::vector<cv::VideoCapture> captures;
    cameraRegister(captures);
    cameraRegister(captures, cl, cr);

    return 0;
}