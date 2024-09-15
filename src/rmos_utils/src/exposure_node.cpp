#include "exposure_node.hpp"

namespace rmos_utils
{
ExposureNode::ExposureNode(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("exposure_node", options),
    is_auto_exposure_(false),
    is_auto_white_balance_(false)
{
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", rclcpp::Node::get_name());

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

void ExposureNode::AutoExposure()
{

}

void ExposureNode::AutoWhiteBalance()
{

}

}