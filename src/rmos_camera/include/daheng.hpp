#ifndef RMOS_DAHENG_HPP
#define RMOS_DAHENG_HPP

#include <iostream>
#include <string>
#include <vector>

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
    CamParamsStruct cam_params_;
    bool AutoExposure(double exposure_factor);
    bool AutoWhiteBalance(std::vector<double> white_balance_factor);

private:
    bool isOpen();
    bool isInit();
    bool isRun();

public:
    bool is_open_;
    bool is_init_;
    bool is_run_;
    
    GX_DEV_HANDLE device_;          // 设备权柄
    PGX_FRAME_BUFFER pFrameBuffer_; // raw 图像的buffer
    uint8_t *rgbImagebuf_;          // rgb 图像的buffer
    std::string error_message_;     // 错误消息，对外传输
    std::string camera_sn_;         // 相机sn号

public:
    template<typename T>
    T GetParam(CamParamsEnum type, T & value) 
    {
        switch(type)
        {
            case CamParamsEnum::Width :
            {
                value = this->cam_params_.width;
                return this->cam_params_.width;
            }
            case CamParamsEnum::Height : 
            {
                value = this->cam_params_.height;
                return this->cam_params_.height;
            }
            case CamParamsEnum::AutoExposure : 
            {
                value = this->cam_params_.auto_exposure;
                return this->cam_params_.auto_exposure;
            }
            case CamParamsEnum::Exposure : 
            {
                value = this->cam_params_.exposure;
                return this->cam_params_.exposure;
            }
            case CamParamsEnum::AutoWhiteBalance : 
            {
                value = this->cam_params_.auto_white_balance;
                return this->cam_params_.auto_white_balance;
            }
            case CamParamsEnum::Gain :
            {
                value = this->cam_params_.gain;
                return this->cam_params_.gain;
            }
            case CamParamsEnum::RGain :
            {
                value = this->cam_params_.r_gain;
                return this->cam_params_.r_gain;
            }
            case CamParamsEnum::GGain :
            {
                value = this->cam_params_.g_gain;
                return this->cam_params_.g_gain;
            }
            case CamParamsEnum::BGain :
            {
                value = this->cam_params_.b_gain;
                return this->cam_params_.b_gain;
            }
            case CamParamsEnum::Gamma : 
            {
                value = this->cam_params_.gamma;
                return this->cam_params_.gamma;
            }
            default : 
            {
                this->error_message_ = "get camera param error ";
                return false;
            }
        }

        return false;
    }

    template<typename T>
    bool SetParam(CamParamsEnum type, T value) 
    {
        switch(type)
        {
            case CamParamsEnum::Exposure :
            {
                status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, this->cam_params_.exposure - 200);
                status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, this->cam_params_.exposure + 200);
                status = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, this->cam_params_.exposure);
                return true;
            }
            case CamParamsEnum::Gain :
            case CamParamsEnum::RGain :
            case CamParamsEnum::GGain :
            case CamParamsEnum::BGain :
            {
                status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
                status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, this->cam_params_.r_gain / 10.0);
                status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
                status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, this->cam_params_.g_gain / 10.0);
                status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
                status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, this->cam_params_.b_gain / 10.0);
                return true;
            }
            default :
            {
                this->error_message_ = "set camera param error ";
                return false;
            }
        }

        return false;
    }

};

}

#endif // RMOS_DAHENG_HPP