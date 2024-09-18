#include "../include/cameraRegister.hpp"

bool cameraRegister(std::vector<cv::VideoCapture> & captures)
{
    int capture_id = 0;
    cv::VideoCapture cap;
    for(int i = 0; i < 4; i++){
        if(capture_id == 2)
            break;
        try {
            cap.open(i, cv::CAP_V4L2);
            if(cap.isOpened()){
                captures.push_back(cap);
                capture_id++;
                std::cout << "capture: " << capture_id << " is opened" << std::endl;
            }
        } catch(cv::Exception& e) {
            std::cout << "Error opening video stream or file" << std::endl;
            std::cout << e.what() << std::endl;
            return false; 
        }
    }
    std::cout << "capture number: " << captures.size() << std::endl;
    return true;
}

bool cameraRegister(std::vector<cv::VideoCapture> & captures, cv::VideoCapture & cl, cv::VideoCapture & cr)
{
    cv::namedWindow("CHOOSE", cv::WINDOW_NORMAL);
    cv::Mat frame;
    for(auto cap : captures)
    {
        int choose = -1;
        while(cap.isOpened())
        {
            cap >> frame;
            cv::imshow("CHOOSE", frame);
            choose = cv::waitKey(30);
            if(choose == '0'){
                cl = cap;
                break;
            }
            else if(choose == '1'){
                cr = cap;
                break;
            }
            else if(choose == 'q'){
                return false;
            }
        }
    }
    cv::destroyWindow("CHOOSE");
    return true;
}
