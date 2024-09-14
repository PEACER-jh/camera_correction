#ifndef SENSOR_NODE_HPP_
#define SENSOR_NODE_HPP_

#include <string>
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
#include "daheng.hpp"

namespace rmos_sensor
{

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(const std::string & node_name, const rclcpp::NodeOptions & options);
    ~CameraNode();

private:
    image_transport::CameraPublisher img_pub_;
    sensor_msgs::msg::CameraInfo camera_info_;
    sensor_msgs::msg::Image::SharedPtr img_msg_;
    rmos_interfaces::msg::CameraInfo camera_info_msg_;
    // std_msgs::msg::Int32 exp_msg;
    
    cv::Mat image_;
    uint32_t frame_id_;
    std::thread capture_thread_;
    std::shared_ptr<rmos_camera::DahengCamera> camera_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;  
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

public:
    rcl_interfaces::msg::SetParametersResult ParamtersCallBack(const std::vector<rclcpp::Parameter> & param);
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

private:
    void capture_thread_lambda();
    void AutoExposureCallBack(rmos_interfaces::msg::CameraInfo::SharedPtr info);

};

}

#endif // SENSOR_NODE_HPP_