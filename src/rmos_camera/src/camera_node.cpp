#include "include/camera_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rmos_sensor
{
CameraNode::CameraNode(const std::string & node_name, const rclcpp::NodeOptions & options) : 
    rclcpp::Node("daheng_camera", options),
    frame_id_(0),
    time_offset(0)
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
    
    cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "DahengCam");
    auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_");
    auto yaml_path = "file://" + pkg_path + "/config/daheng_camera_info.yaml";
    if(!cam_info_manager_->loadCameraInfo(yaml_path)) {
        RCLCPP_WARN(this->get_logger(), "load camera info failed ");
    } else {
        camera_info_ = cam_info_manager_->getCameraInfo();
    }

    img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/info", 10);
    capture_thread_ = std::thread{[this]()->void {this->capture_thread_lambda();}};
    callback_handle_ = this->add_on_set_parameters_callback
                (std::bind(&CameraNode::ParamtersCallBack, this, std::placeholders::_1));



}

CameraNode::~CameraNode()
{
    if(this->capture_thread_.joinable()){
        this->capture_thread_.join();
    }
    this->camera_->SensorShut();
    RCLCPP_INFO(this->get_logger(), "camera node is destroyed");
}

void CameraNode::capture_thread_lambda()
{
    while(rclcpp::ok())
    {
        if(!this->camera_->is_open_)
        {
            RCLCPP_WARN(this->get_logger(), "camera is not open ");
            this->camera_->SensorOpen();
        }
        if(this->camera_->SensorRun(this->image_))
        {
            this->img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
            (*img_msg_).header.stamp = this->camera_info_.header.stamp = this->now() + rclcpp::Duration(0, this->time_offset);
            (*img_msg_).header.frame_id = "camera";
            camera_info_.header.frame_id = (*img_msg_).header.frame_id;
            
            camera_info_pub_->publish(camera_info_);
            img_pub_.publish(*img_msg_, camera_info_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "%s", this->camera_->error_message_.c_str());
            this->camera_->SensorOpen();
        }
    }
}

rcl_interfaces::msg::SetParametersResult





}