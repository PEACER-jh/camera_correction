#include <cmath>
#include <vector>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

int main() 
{
    cv::Mat fl, fr;
    std::vector<cv::VideoCapture> capture;
    
    int capture_id = 0;
    cv::VideoCapture cap;
    for(int i = 0; i < 4; i++){
        if(capture_id == 2)
            break;
        try {
            cap.open(i, cv::CAP_V4L2);
            if(cap.isOpened()){
                capture.push_back(cap);
                capture_id++;
                std::cout << "capture: " << capture_id << " is opened" << std::endl;
            }
        } catch(cv::Exception& e) {
            std::cout << "Error opening video stream or file" << std::endl;
            std::cout << e.what() << std::endl; 
        }
    }
    std::cout << "capture number: " << capture.size() << std::endl;
    cv::VideoCapture cl, cr;
    cv::namedWindow("CHOOSE", cv::WINDOW_NORMAL);
    cv::Mat frame;
    // for(auto cap : capture)
    for(int i = 0; i < capture.size(); i++)
    {
        int choose = -1;
        while(capture[i].isOpened())
        {
            capture[i] >> frame;
            cv::imshow("CHOOSE", frame);
            choose = cv::waitKey(30);
            if(choose == 0){
                cl = capture[i];
                break;
            }
            else if(choose == 1){
                cr = capture[i];
                break;
            } 
        }
    }
    delete [] capture.data();
    cv::destroyWindow("CHOOSE");

    cv::namedWindow("FRAME_LEFT", cv::WINDOW_NORMAL);
    cv::namedWindow("FRAME_RIGHT", cv::WINDOW_NORMAL);
    while(cl.isOpened() && cr.isOpened())
    {
        cl >> fl;
        cr >> fr;
        cv::imshow("FRAME_LEFT", fl);
        cv::imshow("FRAME_RIGHT", fr);
        cv::waitKey(100);
    }

    cl.release();
    cr.release();
    cv::destroyAllWindows();
    return 0;
}