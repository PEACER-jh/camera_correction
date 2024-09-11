#ifndef RMOS_DAHENG_HPP
#define RMOS_DAHENG_HPP

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>

#include <opencv2/opencv.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include "image_transport/image_transport.hpp"
// #include "image_transport/publisher.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "std_msgs/msg/int8.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include "camera_info_manager/camera_info_manager.hpp"

#include "driver/DxImageProc.h"
#include "driver/GxIAPI.h"

namespace rmos_camera
{
enum class CamParamsEnum
{
    Width,                  // 图像宽度
    Height,                 // 图像高度 
    AutoExposure,           // 自动曝光
    Exposure,               // 曝光时间 
    Brightness,             // 亮度
    AutoWhiteBalance,       // 自动白平衡
    WhiteBalance,           // 白平衡
    Gain,                   // 增益
    RGain,                  // 红增益
    GGain,                  // 绿增益
    BGain,                  // 蓝增益
    Gamma,                  // 伽马
    Contrast,               // 对比度
    Saturation,             // 饱和度
    Hue,                    // 色调
    Fps                     // 帧率
};

typedef struct CamParamsStruct
{
    size_t  width;
    size_t  height;

    bool    auto_exposure;
    int16_t exposure;
    int16_t brightness;

    bool    auto_white_balance;
    int16_t white_balance;
    int16_t gain;
    int16_t r_gain;
    int16_t g_gain;
    int16_t b_gain;

    int16_t gamma;
    int16_t contrast;
    int16_t saturation;
    int16_t hue;
    int16_t fps;
}CamParamsStruct;

class DahengCamera
{
public:
    DahengCamera();
    ~DahengCamera();

    bool SensorInit();
    void SensorRun();
    bool SensorDeinit();

private:
    bool isOpen();
    bool isRun();

    void setParams();
    void resetParams();


    


};

}

#endif // RMOS_DAHENG_HPP