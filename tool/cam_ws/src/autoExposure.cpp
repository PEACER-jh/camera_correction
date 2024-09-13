#include "../include/autoExposure.hpp"

cv::Mat autoExposure(cv::Mat& src, cv::Mat& hist, AutoExposureMode mode)
{
    cv::Mat gray;
    if(src.type() != CV_8UC1)
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    else
        gray = src.clone();

    double ideal = 64.0;
    double weight = 5.0;
    double factor = 1.0;
    switch(mode)
    {
        case AutoExposureMode::AVERAGE_METERING:
            factor = AverageMetering(gray, ideal);
            break;
        case AutoExposureMode::CENTER_WEIGHTED_METERING:
            factor = CenterWeightedMetering(gray, ideal, weight);
            break;
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

double AverageMetering(cv::Mat& src, double ideal_brightness)
{
    double average_brightness = cv::mean(src)[0];
    double adjustment_factor = ideal_brightness / average_brightness;

    std::cout << "*********************************************" << std::endl;
    std::cout << "AE: AVERAGE METERING" << std::endl;
    std::cout << "average brightness of image: " << average_brightness << std::endl;
    std::cout << "adjustment factor of image: " << adjustment_factor << std::endl;
    std::cout << "*********************************************" << std::endl;

    return adjustment_factor;
}

double CenterWeightedMetering(cv::Mat& src, double ideal_brightness, double center_weight) 
{
    int rows = src.rows;
    int cols = src.cols;

    cv::Mat weightMat = cv::Mat::zeros(rows, cols, CV_32FC1);
    cv::Point2f center(cols / 2.0, rows / 2.0);
    double radius = std::sqrt(center.x * center.x + center.y * center.y);

    for(int v = 0; v < rows; ++v)
        for(int u = 0; u < cols; ++u)
        {
            double distance = std::sqrt(std::pow(u - center.x, 2) + std::pow(v - center.y, 2));
            weightMat.at<float>(v, u) = center_weight * (1.0 - distance / radius);
        }

    cv::Mat img;
    cv::Mat dst(src.size(), CV_32FC1);
    src.convertTo(img, CV_32FC1);
    cv::multiply(img, weightMat, dst);
    double weighted_average_brightness = cv::mean(dst)[0];
    double adjustment_factor = ideal_brightness / weighted_average_brightness;

    std::cout << "*********************************************" << std::endl;
    std::cout << "AE: CENTER WEIGHTED METERING" << std::endl;
    std::cout << "average brightness of image: " << weighted_average_brightness << std::endl;
    std::cout << "adjustment factor of image: " << adjustment_factor << std::endl;
    std::cout << "*********************************************" << std::endl;

    return adjustment_factor;
}




// double SpotMetering(cv::Mat& src, double ideal_brightness = 128.0) {}
// double MatrixOrMultiZoneMetering(cv::Mat& src, double ideal_brightness = 128.0) {}
// double DynamicRangeOptimization(cv::Mat& src, double ideal_brightness = 128.0) {}