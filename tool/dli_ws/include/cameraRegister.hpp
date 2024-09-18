#ifndef CAMERA_REGISTER_HPP
#define CAMERA_REGISTER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

bool cameraRegister(std::vector<cv::VideoCapture> & captures);
bool cameraRegister(std::vector<cv::VideoCapture> & captures, cv::VideoCapture & cl, cv::VideoCapture & cr);

#endif // CAMERA_REGISTER_HPP