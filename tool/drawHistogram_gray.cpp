#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

int main() 
{
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Histogram", cv::WINDOW_NORMAL);
    cv::Mat image = cv::imread("../doc/wjy1.jpg", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    int histSize = 256;  
    float range[] = {0, 256};  
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double) hist_w / histSize);

    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0));
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX);

    for (int i = 1; i < histSize; i++) {
        cv::line(histImage, 
                 cv::Point(bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1))),
                 cv::Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
                 cv::Scalar(255), 2, 8, 0);
    }

    cv::imshow("Image", image);
    cv::imshow("Histogram", histImage);
    cv::waitKey(0);

    return 0;
}

