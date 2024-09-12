#ifndef AUTO_EXPOSURE_HPP
#define AUTO_EXPOSURE_HPP

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

enum class AutoExposureMode
{
    AVERAGE_METERING,
    CENTER_WEIGHTED_METERING,
    SPOT_METERING,
    MATRIX_OR_MULTI_ZONE_METERING,
    DYNAMIC_RANGE_OPTIMIZATION,
};

void autoExposure(cv::Mat& src, cv::Mat& hist, AutoExposureMode mode = AutoExposureMode::AVERAGE_METERING);

double AverageMetering(cv::Mat& hist, double ideal_brightness = 128.0);
void CenterWeightedMetering(cv::Mat& hist);
void SpotMetering(cv::Mat& hist);
void MatrixOrMultiZoneMetering(cv::Mat& hist);
void DynamicRangeOptimization(cv::Mat& hist);

#endif // AUTO_EXPOSURE_HPP