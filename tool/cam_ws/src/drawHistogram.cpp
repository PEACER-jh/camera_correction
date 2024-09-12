#include "../include/drawHistogram.hpp"

bool isCombine = true;                              // 直方图、原图组合
int histSize = 256;                                 // 灰度级数
float range[] = {0, 256};                           // 灰度值范围
const float* histRange = {range};
int hist_w = 512;                                   // 直方图宽度
int hist_h = 400;                                   // 直方图高度
int bin_w = cvRound((double)hist_w / histSize);     // 直方图柱子宽度

void drawGrayHistogram(cv::Mat & img)
{
    cv::Mat hist;
    cv::Mat src = img.clone();
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);

    cv::calcHist(&src, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    cv::Mat histImg(hist_h, hist_w, CV_8UC1, cv::Scalar(0, 0, 0));
    cv::normalize(hist, hist, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
    for(int i = 1; i < histSize; ++i){
        cv::line(histImg, cv::Point(bin_w * (i -1), hist_h - cvRound(hist.at<float>(i - 1))), 
                    cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))),
                    cv::Scalar(255, 255, 255), 2, 8, 0);
    }

    newWindows(isCombine);
    showWindows(src, histImg, isCombine);
}

void drawColorHistogram(cv::Mat & img)
{
    cv::Mat histB, histG, histR;
    cv::Mat src = img.clone();
    std::vector<cv::Mat> channels;
    cv::split(src, channels);

    cv::calcHist(&channels[0], 1, 0, cv::Mat(), histB, 1, &histSize, &histRange, true, false);
    cv::calcHist(&channels[1], 1, 0, cv::Mat(), histG, 1, &histSize, &histRange, true, false);
    cv::calcHist(&channels[2], 1, 0, cv::Mat(), histR, 1, &histSize, &histRange, true, false);
    cv::normalize(histB, histB, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(histG, histG, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(histR, histR, 0, hist_h, cv::NORM_MINMAX, -1, cv::Mat());
    
    cv::Mat histImg(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
    for(int i = 1; i < histSize; ++i){
        cv::line(histImg, cv::Point(bin_w * (i -1), hist_h - cvRound(histB.at<float>(i - 1))), 
                    cv::Point(bin_w * (i), hist_h - cvRound(histB.at<float>(i))),
                    cv::Scalar(255, 0, 0), 2, 8, 0);
        cv::line(histImg, cv::Point(bin_w * (i -1), hist_h - cvRound(histG.at<float>(i - 1))), 
                    cv::Point(bin_w * (i), hist_h - cvRound(histG.at<float>(i))),
                    cv::Scalar(0, 255, 0), 2, 8, 0);
        cv::line(histImg, cv::Point(bin_w * (i -1), hist_h - cvRound(histR.at<float>(i - 1))), 
                    cv::Point(bin_w * (i), hist_h - cvRound(histR.at<float>(i))),
                    cv::Scalar(0, 0, 255), 2, 8, 0);
    }

    newWindows(isCombine);
    showWindows(src, histImg, isCombine);
}

void newWindows(bool isCombine)
{
    if(isCombine)
        cv::namedWindow(COMBINE_WINDOW, cv::WINDOW_NORMAL);  
    else
    {
        cv::namedWindow(IMAGE_WINDOW, cv::WINDOW_NORMAL);
        cv::namedWindow(HIST_WINDOW, cv::WINDOW_NORMAL);
    }
}

void showWindows(cv::Mat& img, cv::Mat& hist, bool isCombine)
{
    if(isCombine)
    {
        cv::Mat dst, canvas;
        cv::resize(img, dst, hist.size(), 0, 0, cv::INTER_AREA);
        cv::hconcat(dst, hist, canvas);
        cv::imshow(COMBINE_WINDOW, canvas);
    }
    else
    {
        cv::imshow(IMAGE_WINDOW, img);
        cv::imshow(HIST_WINDOW, hist);
    }
    cv::waitKey(10);
}
