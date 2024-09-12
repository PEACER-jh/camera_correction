#include "include/autoExposure.hpp"

void autoExposure(cv::Mat& src, cv::Mat& hist, AutoExposureMode mode = AutoExposureMode::AVERAGE_METERING)
{
    cv::Mat gray;
    if(src.type() != CV_8UC1)
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    else
        gray = src.clone();

    switch(mode)
    {
        case AutoExposureMode::AVERAGE_METERING:
            double ideal = 128.0;
            AverageMetering(hist, ideal);
            break;
        case AutoExposureMode::CENTER_WEIGHTED_METERING:
            CenterWeightedMetering(hist);
            break;
        case AutoExposureMode::SPOT_METERING:
            SpotMetering(hist);
            break;
        case AutoExposureMode::MATRIX_OR_MULTI_ZONE_METERING: 
            MatrixOrMultiZoneMetering(hist);
            break;
        case AutoExposureMode::DYNAMIC_RANGE_OPTIMIZATION:
            DynamicRangeOptimization(hist);
            break;
        default:
            std::cout << "Invalid mode!" << std::endl; 
            return;
    }




}

double AverageMetering(cv::Mat& hist, double ideal_brightness = 128.0)
{
    
}