#include "exposure_node.hpp"

namespace rmos_utils
{
ExposureNode::ExposureNode(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("exposure_node", options),
    is_auto_exposure_(false),
    is_auto_white_balance_(false),
    exp_ideal_(128.0),
    exp_weight_(1.0)
{
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", rclcpp::Node::get_name());

    this->exp_ideal_ = this->declare_parameter<double>("exp_ideal", 128.0);
    this->exp_weight_ = this->declare_parameter<double>("exp_weight", 1.0);
    this->ae_a = this->declare_parameter<double>("ae_a", 1.0);
    this->ae_b = this->declare_parameter<double>("ae_b", 0.0);
    this->awb_a = this->declare_parameter<double>("awb_a", 1.0);
    this->awb_b = this->declare_parameter<double>("awb_b", 0.0);
    this->AutoExposureMode_ = this->declare_parameter<rmos_utils::AutoExposureMode>
                                                    ("exp_auto_exposure_mode", rmos_utils::AutoExposureMode::AVERAGE_METERING);
    this->AutoWhiteBalanceMode_ = this->declare_parameter<rmos_utils::AutoWhiteBalanceMode>
                                                    ("exp_auto_white_balance_mode", rmos_utils::AutoWhiteBalanceMode::GrayWorldAssumption);

    image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                this, "/image_raw", std::bind(&ExposureNode::RawImageCallBack, this, std::placeholders::_1),
                "raw", rmw_qos_profile_default));
    camera_exp_info_clt_ = this->create_client<rmos_interfaces::srv::CameraInfo>("/camera/exposure_info");
    camera_exp_info_sub_ = this->create_subscription<rmos_interfaces::msg::CameraInfo>
                ("/camera/exp_info", 10, std::bind(&ExposureNode::ImageInfoCallBack, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>
                ("/camera/info", rclcpp::SensorDataQoS(), std::bind(&ExposureNode::CameraInfoCallBack, this, std::placeholders::_1));

}

ExposureNode::~ExposureNode()
{
    RCLCPP_INFO(this->get_logger(), "exposure node is destroyed");
}

void ExposureNode::RawImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img)
{
    this->image_ = cv_bridge::toCvShare(img, "bgr8")->image;
    double ae_factor = 1.0;
    std::vector<double> awb_factor = {1.0, 1.0, 1.0};
    if(this->is_auto_exposure_)
        ae_factor = AutoExposure(image_);
    if(this->is_auto_white_balance_)
        awb_factor = AutoWhiteBalance(image_);

    double ae_exposure = ae_a * ae_factor + ae_b;
    std::vector<double> awb_gain;
    for(int i = 0; i < 3; i++)
        awb_gain.push_back(awb_a * awb_factor[i] + awb_b);
    
    this->camera_info_msg_.exposure = ae_exposure;
    this->camera_info_msg_.red_gain = awb_gain[0];
    this->camera_info_msg_.green_gain = awb_gain[1];
    this->camera_info_msg_.blue_gain = awb_gain[2];
    auto exp_request = std::make_shared<rmos_interfaces::srv::CameraInfo::Request>(this->camera_info_msg_);
    auto exp_result = this->camera_exp_info_clt_->async_send_request(exp_request);
}

void ExposureNode::CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    RCLCPP_INFO(this->get_logger(), "receive camera infomation");
    this->camera_info_ = *info;
    this->camera_matrix.create(3, 3, CV_64FC1);
    this->dist_coeffs_.create(1, 5, CV_64FC1);
    for(int i = 0; i < 9; i++)
        this->camera_matrix.at<double>(i / 3, 1 % 3) = this->camera_info_.k[i];
    for(int i = 0; i < 5; i++)
        this->dist_coeffs_.at<double>(0, i) = this->camera_info_.d[i];
    this->camera_info_sub_.reset();
}

void ExposureNode::ImageInfoCallBack(const rmos_interfaces::msg::CameraInfo::ConstSharedPtr info)
{
    camera_info_msg_.auto_exposure = info->auto_exposure;
    camera_info_msg_.exposure = info->exposure;
    camera_info_msg_.auto_white_balance = info->auto_white_balance;
    camera_info_msg_.gain = info->gain;
    camera_info_msg_.red_gain = info->red_gain;
    camera_info_msg_.blue_gain = info->blue_gain;
    camera_info_msg_.green_gain = info->green_gain;
    this->is_auto_exposure_ = info->auto_exposure;
    this->is_auto_white_balance_ = info->auto_white_balance;
}

cv::Mat ExposureNode::drawHistogram(const cv::Mat & img, bool is_gray) 
{
    int histSize = 256;                                 // 灰度级数
    float range[] = {0, 256};                           // 灰度值范围
    const float* histRange = {range};
    int hist_w = 512;                                   // 直方图宽度
    int hist_h = 400;                                   // 直方图高度
    int bin_w = cvRound((double)hist_w / histSize);     // 直方图柱子宽度
    if(is_gray) {
        cv::Mat hist, hist_norm;
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
        return histImg;
    } else {
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
        return histImg;
    }
}

double ExposureNode::AutoExposure(const cv::Mat & img)
{
    cv::Mat gray;
    double factor = 1.0;
    if(image_.type() != CV_8UC1)
        cv::cvtColor(image_, gray, cv::COLOR_BGR2GRAY);
    else
        gray = image_.clone();
    
    switch(this->AutoExposureMode_)
    {
        case AutoExposureMode::AVERAGE_METERING:
        {
            double average_brightness = cv::mean(gray)[0];
            factor = this->exp_ideal_ / average_brightness;

std::cout << "*********************************************" << std::endl;
std::cout << "AE: AVERAGE METERING" << std::endl;
std::cout << "average brightness of image: " << average_brightness << std::endl;
std::cout << "adjustment factor of image: " << factor << std::endl;
std::cout << "*********************************************" << std::endl;
            return factor;
        }
        case AutoExposureMode::CENTER_WEIGHTED_METERING:
        {
            int rows = gray.rows;
            int cols = gray.cols;

            cv::Mat weightMat = cv::Mat::zeros(rows, cols, CV_32FC1);
            cv::Point2f center(cols / 2.0, rows / 2.0);
            double radius = std::sqrt(center.x * center.x + center.y * center.y);

            for(int v = 0; v < rows; ++v)
                for(int u = 0; u < cols; ++u)
                {
                    double distance = std::sqrt(std::pow(u - center.x, 2) + std::pow(v - center.y, 2));
                    weightMat.at<float>(v, u) = this->exp_weight_ * (1.0 - distance / radius);
                }

            cv::Mat img;
            cv::Mat dst(gray.size(), CV_32FC1);
            gray.convertTo(img, CV_32FC1);
            cv::multiply(img, weightMat, dst);
            double weighted_average_brightness = cv::mean(dst)[0];
            double adjustment_factor = this->exp_ideal_ / weighted_average_brightness;

std::cout << "*********************************************" << std::endl;
std::cout << "AE: CENTER WEIGHTED METERING" << std::endl;
std::cout << "average brightness of image: " << weighted_average_brightness << std::endl;
std::cout << "adjustment factor of image: " << adjustment_factor << std::endl;
std::cout << "*********************************************" << std::endl;
            return factor;
        }
        case AutoExposureMode::SPOT_METERING:
        case AutoExposureMode::MATRIX_OR_MULTI_ZONE_METERING:
        case AutoExposureMode::DYNAMIC_RANGE_OPTIMIZATION:
        {
            RCLCPP_WARN(this->get_logger(), "AutoExposureMode not supported");
            return factor;
        }
    }
}

std::vector<double> ExposureNode::AutoWhiteBalance(const cv::Mat & img) 
{
    RCLCPP_WARN(this->get_logger(), "AutoWhiteBalance not supported");
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_utils::ExposureNode)