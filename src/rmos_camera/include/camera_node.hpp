#ifndef SENSOR_NODE_HPP_
#define SENSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/publisher.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "camera_info_manager/camera_info_manager.hpp"

#include "rmos_interfaces/msg/CameraInfo.msg"
#include "daheng.hpp"

namespace rmos_sensor
{

class CameraNode : public rclcpp::Node
{
public:
    CameraNode(const std::string & node_name, const rclcpp::NodeOptions & options);
    ~CameraNode();



};




}

#endif // SENSOR_NODE_HPP_