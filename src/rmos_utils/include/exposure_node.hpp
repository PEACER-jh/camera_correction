#ifndef EXPOSURE_NODE_HPP
#define EXPOSURE_NODE_HPP

#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/publisher.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rmos_interfaces/msg/camera_info.hpp"
#include "rmos_interfaces/srv/camera_info.hpp"

namespace rmos_utils
{
class ExposureNode : public rclcpp::Node
{
public:
    ExposureNode(const rclcpp::NodeOptions & options);
    ~ExposureNode();

    std::shared_ptr<image_transport::Subscriber> image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Client<rmos_interfaces::srv::CameraInfo>::SharedPtr camera_exp_info_clt_;
    rclcpp::Subscription<rmos_interfaces::msg::CameraInfo>::SharedPtr camera_exp_info_sub_;

private:
    sensor_msgs::msg::Image::SharedPtr img_msg_;
    sensor_msgs::msg::CameraInfo camera_info_;
    rmos_interfaces::msg::CameraInfo camera_info_msg_;

    cv::Mat image_;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs_;
    bool is_auto_exposure_;
    bool is_auto_white_balance_;
    // std::thread exposure_thread_;
    // void exposure_thread_lambda();

    void RawImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img);
    void CameraInfoCallBack(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
    
    void ImageInfoCallBack(const rmos_interfaces::msg::CameraInfo::ConstSharedPtr info);
    void AutoExposure();
    void AutoWhiteBalance();

};

}

#endif // EXPOSURE_NODE_HPP