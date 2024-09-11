#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

int main()
{
    cv::namedWindow("Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Histogram", cv::WINDOW_NORMAL);
    cv::Mat image = cv::imread("../doc/wjy2.jpg", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    std::vector<cv::Mat> bgr_planes;
    cv::split(image, bgr_planes); 

    int histSize = 256; 
    float range[] = {0, 256}; 
    const float* histRange = {range};

    bool uniform = true;
    bool accumulate = false;

    cv::Mat b_hist, g_hist, r_hist;

    cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound((double) hist_w / histSize);

    cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX);
    cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX);
    cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX);

    for (int i = 1; i < histSize; i++) {
        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1))),
                 cv::Point(bin_w*(i), hist_h - cvRound(b_hist.at<float>(i))),
                 cv::Scalar(255, 0, 0), 2, 8, 0);  // 蓝色通道

        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1))),
                 cv::Point(bin_w*(i), hist_h - cvRound(g_hist.at<float>(i))),
                 cv::Scalar(0, 255, 0), 2, 8, 0);  // 绿色通道

        cv::line(histImage, cv::Point(bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1))),
                 cv::Point(bin_w*(i), hist_h - cvRound(r_hist.at<float>(i))),
                 cv::Scalar(0, 0, 255), 2, 8, 0);  // 红色通道
    }

    cv::imshow("Image", image);
    cv::imshow("Histogram", histImage);
    cv::waitKey(0);

    return 0;
}
