#include "include/daheng.hpp"

namespace rmos_camera
{
DahengCamera::DahengCamera(const std::string camer_sn) :
    camera_sn_(camer_sn), device_(nullptr), pFrameBuffer_(nullptr)
{
    this->cam_params_.width  = 1920;
    this->cam_params_.height = 1200;
    this->cam_params_.auto_exposure = false;
    this->cam_params_.exposure = 1000;
    this->cam_params_.brightness = 0.0;
    this->cam_params_.auto_white_balance = false;
    this->cam_params_.white_balance = 0.0;
    this->cam_params_.gain = 0.0;
    this->cam_params_.r_gain = 19.8;
    this->cam_params_.g_gain = 19.8;
    this->cam_params_.b_gain = 19.8;
    this->cam_params_.gamma = 0.0;
    this->cam_params_.contrast = 0.0;
    this->cam_params_.saturation = 0.0;
    this->cam_params_.hue = 0.0;
    this->cam_params_.fps = 100;

    this->is_open_ = false;
    this->is_init_ = false;
    this->is_run_  = false;
    this->error_message_ = "";

}

DahengCamera::~DahengCamera() 
{
    if(is_open_){
        this->is_run_ = false;
        this->is_init_ = false;
        this->is_open_ = !this->SensorShut();
    }
}

bool DahengCamera::isOpen(){
    return is_open_;
}
bool DahengCamera::isRun(){
    return is_run_;
}
bool DahengCamera::isInit(){
    return is_init_;
}

bool DahengCamera::SensorOpen()
{
    if(this->is_open_)
        return true;
    if(!this->SensorInit())
        return false;
    
    rgbImagebuf_ = new uint8_t[this->cam_params_.height * this->cam_params_.width * 3];
    GX_STATUS status;
    status = GXStreamOn(device_);
    if(status != GX_STATUS_SUCCESS)
    {
        this->is_open_ = !this->SensorShut();
        this->error_message_ = "DahengCam open failed ";
        return false;
    }
    this->is_open_ = true;
    return true;
}

bool DahengCamera::SensorInit()
{
    GX_STATUS status;
    // init the lib of DahengCam
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        GXCloseLib();
        this->error_message_ = "DahengCam initsdk InitLib failed ";
        return false;
    }

    // get device lists
    uint32_t ui32DeviceNum = 0;
    status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if (status != GX_STATUS_SUCCESS || ui32DeviceNum <= 0)
    {
        GXCloseLib();
        this->error_message_ = "DahengCam initsdk getDeviceList failed ";
        return false;
    }

    // open device by sn
    status = GXOpenDeviceByIndex(1, &device_); //  open the  device

    if (status != GX_STATUS_SUCCESS)
    {
        GXCloseLib();
        this->error_message_ = "DahengCam initsdk OpentDevice failed ";
        return false;
    }

    size_t nSize = 0;
    // get  vendor name
    GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    char *pszVendorName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    delete[] pszVendorName;
    pszVendorName = NULL;

    // get nodel_name
    GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    char *pszModelName = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    delete[] pszModelName;
    pszModelName = NULL;

    // get serial_number
    GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    char *pszSerialNumber = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    // get version
    GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
    char *pszDeviceVersion = new char[nSize];
    GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;

    // set the model and the buffer num
    status = GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    status = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    status = GXSetAcqusitionBufferNumber(device_, ACQ_BUFFER_NUM);

    // set exposure of camera
    status = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, this->cam_params_.exposure - 200);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, this->cam_params_.exposure + 200);
    status = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME, this->cam_params_.exposure);

    // set white balance of camera
    status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, this->cam_params_.r_gain / 10.0);
    status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, this->cam_params_.g_gain / 10.0);
    status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO, this->cam_params_.b_gain / 10.0);

    // set gamma of camera
    status = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
    status = GXSetFloat(device_, GX_FLOAT_GAIN, this->cam_params_.gamma);

    if(status != GX_STATUS_SUCCESS)
    {
        this->error_message_ = "DahengCam set init failed ";
        return false;
    }
   
    return true;
}

bool DahengCamera::SensorRun(cv::Mat & image)
{
    GX_STATUS status;
    status = GXDQBuf(device_, &pFrameBuffer_, 1000);
    if(status != GX_STATUS_SUCCESS)
    {
        this->error_message_ = "DahengCam get buffer failed ";
        return false;
    }

    VxInt32 DXstatus = DxRaw8toRGB24(pFrameBuffer_->pImgBuf, 
                                    rgbImagebuf_, pFrameBuffer_->nWidth, 
                                    pFrameBuffer_->nHeight, 
                                    RAW2RGB_NEIGHBOUR, 
                                    DX_PIXEL_COLOR_FILTER(4), 
                                    false);
    if (DXstatus != DX_OK)
    {
        this->error_message_ = "DahengCam RawtoRGB24 failed ";
        return false;
    }
    auto speed_test_start_begin_time = std::chrono::steady_clock::now();
    image.create(pFrameBuffer_->nHeight, pFrameBuffer_->nWidth, CV_8UC3);
    memcpy(image.data, rgbImagebuf_, pFrameBuffer_->nHeight * pFrameBuffer_->nWidth * 3);
    // 重新回采
    status = GXQBuf(device_, pFrameBuffer_);
    if (status != GX_STATUS_SUCCESS)
    {
        this->error_message_ = "DahengCam get buffer failed ";
        return false;
    }
    auto cost = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count();
    
    return true;
}

bool DahengCamera::SensorShut()
{
    GX_STATUS status;
    status = GXStreamOff(device_);
    if(this->rgbImagebuf_ != NULL)
    {
        delete [] this->rgbImagebuf_;
        this->rgbImagebuf_ = NULL;
    }
    status = GXCloseDevice(device_);
    status = GXCloseLib();
    if(status != GX_STATUS_SUCCESS)
    {
        this->error_message_ = "DahengCam shut down failed ";
        return false;
    }

    this->is_open_ = false;
    return true;
}

bool DahengCamera::AutoExposure(double exposure_factor)
{
    if(!cam_params_.auto_exposure){
        return false;
    }
}

bool DahengCamera::AutoWhiteBalance(std::vector<double> white_balance_factor)
{
    if(cam_params_.auto_white_balance){
        return false;
    }
}

}
