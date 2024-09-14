#include "include/camera_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rmos_sensor
{
CameraNode::CameraNode(const std::string & node_name, const rclcpp::NodeOptions & options) : 
    rclcpp::Node("daheng_camera", options),
    frame_id_(0)
{
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());

    camera_->cam_params_.width = this->declare_parameter<size_t>("/camera/width", 1920);
    camera_->cam_params_.height =  this->declare_parameter<size_t>("/camera/height", 1200);
    camera_->cam_params_.auto_exposure = this->declare_parameter<bool>("/camera/auto_exposure", false);
    camera_->cam_params_.exposure = this->declare_parameter<int>("/camera/exposure", 1000);
    camera_->cam_params_.brightness = this->declare_parameter<float>("/camera/brightness", 0.0);
    camera_->cam_params_.auto_white_balance = this->declare_parameter<bool>("/camera/auto_white_balance", false);
    camera_->cam_params_.white_balance = this->declare_parameter<float>("/camera/white_balance", 0.0);
    camera_->cam_params_.gain = this->declare_parameter<float>("/camera/gain", 0.0);
    camera_->cam_params_.r_gain = this->declare_parameter<float>("/camera/r_gain", 19.8);
    camera_->cam_params_.g_gain = this->declare_parameter<float>("/camera/g_gain", 19.8);
    camera_->cam_params_.b_gain = this->declare_parameter<float>("/camera/b_gain", 19.8);
    camera_->cam_params_.gamma = this->declare_parameter<float>("/camera/gamma", 0.0);
    camera_->cam_params_.contrast = this->declare_parameter<float>("/camera/contrast", 0.0);
    camera_->cam_params_.saturation = this->declare_parameter<float>("/camera/saturation", 0.0);
    camera_->cam_params_.hue = this->declare_parameter<float>("/camera/hue", 0.0);
    camera_->cam_params_.fps = this->declare_parameter<int>("/camera/fps", 100);

    camera_info_msg_.auto_exposure = camera_->cam_params_.auto_exposure;
    camera_info_msg_.exposure = camera_->cam_params_.exposure;
    camera_info_msg_.auto_white_balance = camera_->cam_params_.auto_white_balance;
    camera_info_msg_.gain = camera_->cam_params_.gain;
    camera_info_msg_.red_gain = camera_->cam_params_.r_gain;
    camera_info_msg_.green_gain = camera_->cam_params_.g_gain;
    camera_info_msg_.blue_gain = camera_->cam_params_.b_gain;

    bool is_open = camera_->SensorOpen();
    img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);
    
    cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "DahengCam");
    auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_");
    auto yaml_path = "file://" + pkg_path + "/config/daheng_camera_info.yaml";
    if(!cam_info_manager_->loadCameraInfo(yaml_path)) {
        RCLCPP_WARN(this->get_logger(), "load camera info failed ");
    } else {
        camera_info_ = cam_info_manager_->getCameraInfo();
    }



}








}