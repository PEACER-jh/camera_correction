#include "../include/autoExposure.hpp"

cv::Mat autoExposure(cv::Mat& src, cv::Mat& hist, AutoExposureMode mode)
{
    cv::Mat gray;
    if(src.type() != CV_8UC1)
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    else
        gray = src.clone();

    double ideal = 128.0;
    double factor;
    switch(mode)
    {
        case AutoExposureMode::AVERAGE_METERING:
            factor = AverageMetering(hist, ideal);
            break;
        // case AutoExposureMode::CENTER_WEIGHTED_METERING:
        //     CenterWeightedMetering(hist, ideal);
        //     break;
        // case AutoExposureMode::SPOT_METERING:
        //     SpotMetering(hist);
        //     break;
        // case AutoExposureMode::MATRIX_OR_MULTI_ZONE_METERING: 
        //     MatrixOrMultiZoneMetering(hist);
        //     break;
        // case AutoExposureMode::DYNAMIC_RANGE_OPTIMIZATION:
        //     DynamicRangeOptimization(hist);
        //     break;
        default:
            std::cout << "Invalid mode!" << std::endl; 
            return src;
    }

    cv::Mat ae_img;
    src.convertTo(ae_img, CV_8UC3, factor, 0);
    return ae_img;
}

double AverageMetering(cv::Mat& hist, double ideal_brightness)
{
    int rows = hist.rows;
    int cols = hist.cols;
    double pixels = rows * cols;
    double brightness = 0.0;

    cv::namedWindow("DEBUG: histogram", cv::WINDOW_NORMAL);
    cv::imshow("DEBUG: histogram", hist);
    
    for(int i = 0; i < 256; ++i)
        brightness += hist.at<double>(i) * i;
    double average_brightness = brightness / pixels;
    double adjustment_factor = ideal_brightness / average_brightness;

    std::cout << "*********************************************" << std::endl;
    std::cout << "total brightness of image: " << brightness << std::endl;
    std::cout << "average brightness of image: " << average_brightness << std::endl;
    std::cout << "adjustment factor of image: " << adjustment_factor << std::endl;
    std::cout << "*********************************************" << std::endl;

    return adjustment_factor;
}

// double CenterWeightedMetering(cv::Mat& hist, double ideal_brightness = 128.0) {}
// double SpotMetering(cv::Mat& hist, double ideal_brightness = 128.0) {}
// double MatrixOrMultiZoneMetering(cv::Mat& hist, double ideal_brightness = 128.0) {}
// double DynamicRangeOptimization(cv::Mat& hist, double ideal_brightness = 128.0) {}