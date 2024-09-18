#ifndef FEATURES_HPP
#define FEATURES_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

void computeFeatures_ORB(const cv::Mat& l, const cv::Mat& r);


#endif // FEATURES_HPP