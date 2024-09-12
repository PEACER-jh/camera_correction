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

void cameraCalibration(cv::Mat& frame)
{
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-5;
    double fx = 458.6540000, fy = 457.296000, cx = 367.251000, cy = 248.375000;

    int rows = frame.rows;
    int cols = frame.cols;

    for(int v = 0; v < rows; ++v)
        for(int u = 0; u < cols; ++u)
        {
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double r = std::sqrt(x * x + y * y);
            double xd = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double yd = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            int ud = fx * xd + cx;
            int vd = fy * yd + cy;

            if(ud >= 0 && vd >= 0 && ud < cols && vd < rows)
                frame.at<uchar>(v, u) = frame.at<uchar>((int)vd, (int)ud);
            else
                frame.at<uchar>(v, u) = 0;
        }
}
