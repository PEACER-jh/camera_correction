#ifndef DRAW_HISTOGRAM_HPP
#define DRAW_HISTOGRAM_HPP

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMAGE_WINDOW_RAW   "Image_Raw"
#define HIST_WINDOW_RAW    "Histogram_Raw"
#define COMBINE_WINDOW_RAW "Combine_Raw"

#define IMAGE_WINDOW_PRO   "Image_Pro"
#define HIST_WINDOW_PRO    "Histogram_Pro"
#define COMBINE_WINDOW_PRO "Combine_Pro"

void newWindows(bool isCombine = true, bool isRaw = true);                                 // 创建窗口
void showWindows(cv::Mat& img, cv::Mat& hist, bool isCombine = true, bool isRaw = true);   // 显示窗口

void cameraCalibration(cv::Mat& frame); // 相机标定

cv::Mat drawColorHistogram(cv::Mat & img, bool isRaw); // 绘制彩色直方图
cv::Mat drawGrayHistogram(cv::Mat & img, bool isRaw);  // 绘制灰度直方图

#endif // DRAW_HISTOGRAM_HPP