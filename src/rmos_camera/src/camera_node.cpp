#include "camera_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rmos_camera
{
// CameraNode::CameraNode(const std::string & node_name, const rclcpp::NodeOptions & options) :
CameraNode::CameraNode(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("daheng_camera", options),
    frame_id_(0),
    time_offset(0)
{
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", rclcpp::Node::get_name());

    this->time_offset = this->declare_parameter<int>("/camera/time_offset", 0);

    this->camera_ = std::make_shared<rmos_camera::DahengCamera>();
    camera_->cam_params_.width = this->declare_parameter<int>("/camera/width", 1920);
    camera_->cam_params_.height = this->declare_parameter<int>("/camera/height", 1200);
    camera_->cam_params_.auto_exposure = this->declare_parameter<bool>("/camera/auto_exposure", false);
    camera_->cam_params_.exposure = this->declare_parameter<int>("/camera/exposure", 10000);
    camera_->cam_params_.brightness = this->declare_parameter<float>("/camera/brightness", 0.0);
    camera_->cam_params_.auto_white_balance = this->declare_parameter<bool>("/camera/auto_white_balance", false);
    camera_->cam_params_.white_balance = this->declare_parameter<float>("/camera/white_balance", 0.0);
    camera_->cam_params_.gain = this->declare_parameter<float>("/camera/gain", 9.0);
    camera_->cam_params_.r_gain = this->declare_parameter<float>("/camera/r_gain", 15.8);
    camera_->cam_params_.g_gain = this->declare_parameter<float>("/camera/g_gain", 100.8);
    camera_->cam_params_.b_gain = this->declare_parameter<float>("/camera/b_gain", 10.0);
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
    auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_bringup");
    auto yaml_path = "file://" + pkg_path + "/config/daheng_cam_info.yaml";
    if(!cam_info_manager_->loadCameraInfo(yaml_path)) {
        RCLCPP_WARN(this->get_logger(), "load camera info failed ");
    } else {
        camera_info_ = cam_info_manager_->getCameraInfo();
    }

    img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/info", 10);
    camera_exp_info_pub_ = this->create_publisher<rmos_interfaces::msg::CameraInfo>("/camera/exp_info", 10);
    camera_exp_info_srv_ = this->create_service<rmos_interfaces::srv::CameraInfo>
                ("/camera/exposure_info", std::bind(&CameraNode::AutoExposureCallBack, this, std::placeholders::_1, std::placeholders::_2));

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

            rmos_interfaces::msg::CameraInfo info;
            info.exposure = this->camera_->cam_params_.auto_exposure;
            info.exposure = this->camera_->cam_params_.exposure;
            info.auto_white_balance = this->camera_->cam_params_.auto_white_balance;
            info.gain = this->camera_->cam_params_.gain;
            info.red_gain = this->camera_->cam_params_.r_gain;
            info.blue_gain = this->camera_->cam_params_.b_gain;
            info.green_gain = this->camera_->cam_params_.g_gain;

            camera_exp_info_pub_->publish(info);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "%s", this->camera_->error_message_.c_str());
            this->camera_->SensorOpen();
        }
    }
}

rcl_interfaces::msg::SetParametersResult CameraNode::ParamtersCallBack(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    
    for(const auto &param : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "%s", param.get_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.get_type_name().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", param.value_to_string().c_str());

        if(param.get_name() == "time_offset") {
            this->time_offset = param.as_int();
        } else {
            RCLCPP_WARN(this->get_logger(), "unknown or unchange parameter %s", param.get_name().c_str());
        }
    }

    return result;
}

void CameraNode::AutoExposureCallBack(const std::shared_ptr<rmos_interfaces::srv::CameraInfo::Request> request,
                                            std::shared_ptr<rmos_interfaces::srv::CameraInfo::Response> response)
{
    this->camera_->cam_params_.auto_exposure = request->info.auto_exposure;
    this->camera_->cam_params_.auto_white_balance = request->info.auto_white_balance;

    if(!request->info.auto_exposure && !request->info.auto_white_balance)
    {
        RCLCPP_INFO(this->get_logger(), "auto exposure and white balance are both off");
        return;
    }


    if(request->info.auto_exposure)
    {
        this->camera_->cam_params_.exposure = request->info.exposure;
        this->camera_->SetParam(rmos_camera::CamParamsEnum::Exposure, this->camera_->cam_params_.exposure);
    }
    if(request->info.auto_white_balance)
    {
        this->camera_->cam_params_.r_gain = request->info.red_gain;
        this->camera_->cam_params_.g_gain = request->info.green_gain;
        this->camera_->cam_params_.b_gain = request->info.blue_gain;
        this->camera_->SetParam(rmos_camera::CamParamsEnum::RGain, this->camera_->cam_params_.r_gain);
        this->camera_->SetParam(rmos_camera::CamParamsEnum::GGain, this->camera_->cam_params_.g_gain);
        this->camera_->SetParam(rmos_camera::CamParamsEnum::BGain, this->camera_->cam_params_.b_gain);
    }

}

} // namespace rmos_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_camera::CameraNode)