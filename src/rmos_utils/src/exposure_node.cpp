#include "exposure_node.hpp"

namespace rmos_utils
{
ExposureNode::ExposureNode(const rclcpp::NodeOptions & options) : 
    rclcpp::Node("exposure_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting node [%s]", rclcpp::Node::get_name());

    image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                this, "/image_raw", std::bind(&ExposureNode::RawImageCallBack, this, std::placeholders::_1),
                "raw", rmw_qos_profile_default));

    camera_exp_info_clt_ = this->create_client<rmos_interfaces::srv::CameraInfo>("/camera/exposure_info");





}

ExposureNode::~ExposureNode()
{
    RCLCPP_INFO(this->get_logger(), "exposure node is destroyed");
}

bool ExposureNode::RawImageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr & img)
{

}

}