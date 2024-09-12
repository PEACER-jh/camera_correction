#ifndef DRAW_HISTOGRAM_HPP
#define DRAW_HISTOGRAM_HPP

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMAGE_WINDOW   "Image"
#define HIST_WINDOW    "Histogram"
#define COMBINE_WINDOW "Combine"

void newWindows(bool isCombine = true);                                 // 创建窗口
void showWindows(cv::Mat& img, cv::Mat& hist, bool isCombine = true);   // 显示窗口

void cameraCalibration(cv::Mat& frame); // 相机标定

cv::Mat drawColorHistogram(cv::Mat & img); // 绘制彩色直方图
cv::Mat drawGrayHistogram(cv::Mat & img);  // 绘制灰度直方图

#endif // DRAW_HISTOGRAM_HPP