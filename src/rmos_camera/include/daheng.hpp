#ifndef RMOS_DAHENG_HPP
#define RMOS_DAHENG_HPP

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "driver/DxImageProc.h"
#include "driver/GxIAPI.h"

namespace rmos_camera
{
#define ACQ_TRANSFER_SIZE (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB 64
#define ACQ_BUFFER_NUM 3


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
    int exposure;
    float brightness;

    bool    auto_white_balance;
    float white_balance;
    float gain;
    float r_gain;
    float g_gain;
    float b_gain;

    float gamma;
    float contrast;
    float saturation;
    float hue;
    int fps;
}CamParamsStruct;

class DahengCamera
{
public:
    DahengCamera(const std::string camer_sn);
    ~DahengCamera();

    bool SensorOpen();
    bool SensorInit();
    bool SensorRun(cv::Mat & image);
    bool SensorShut();

public: 
    bool AutoExposure();
    bool AutoWhiteBalance();

    template<typename T>
    bool SetParam(CamParamsEnum param, T value);
    template<typename T>
    bool GetParam(CamParamsEnum param, T &value);

private:
    bool isOpen();
    bool isInit();
    bool isRun();

private:
    CamParamsStruct cam_params_;
    bool is_open_;
    bool is_init_;
    bool is_run_;
    
    GX_DEV_HANDLE device_;          // 设备权柄
    PGX_FRAME_BUFFER pFrameBuffer_; // raw 图像的buffer
    uint8_t *rgbImagebuf_;          // rgb 图像的buffer
    std::string error_message_;     // 错误消息，对外传输
    std::string camera_sn_;         // 相机sn号

};

}

#endif // RMOS_DAHENG_HPP