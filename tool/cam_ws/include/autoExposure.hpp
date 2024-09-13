#ifndef AUTO_EXPOSURE_HPP
#define AUTO_EXPOSURE_HPP

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

enum class AutoExposureMode
{
    AVERAGE_METERING,               // 平均测光
    CENTER_WEIGHTED_METERING,       // 中心加权测光
    SPOT_METERING,                  // 点测光
    MATRIX_OR_MULTI_ZONE_METERING,  // 矩阵测光或多区域测光
    DYNAMIC_RANGE_OPTIMIZATION,     // 动态范围优化
};

cv::Mat autoExposure(cv::Mat& src, cv::Mat& hist, AutoExposureMode mode = AutoExposureMode::AVERAGE_METERING);

double AverageMetering(cv::Mat& src, double ideal_brightness = 128.0);
double CenterWeightedMetering(cv::Mat& src, double ideal_brightness = 128.0, double center_weight = 0.5);
// double SpotMetering(cv::Mat& src, double ideal_brightness = 128.0);
// double MatrixOrMultiZoneMetering(cv::Mat& src, double ideal_brightness = 128.0);
// double DynamicRangeOptimization(cv::Mat& src, double ideal_brightness = 128.0);

#endif // AUTO_EXPOSURE_HPP