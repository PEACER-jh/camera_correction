#include "include/daheng.hpp"

namespace rmos_camera
{
DahengCamera::DahengCamera(const std::string camer_sn) :
    camera_sn_(camer_sn), device_(nullptr), pFrameBuffer_(nullptr)
{
    this->cam_params_.height = 1200;
    this->cam_params_.width  = 1920;
    this->cam_params_.auto_exposure = false;
    this->cam_params_.exposure = 1000;
    this->cam_params_.brightness = 0;
    this->cam_params_.auto_white_balance = false;
    this->cam_params_.white_balance = 0;
    this->cam_params_.gain = 0;
    this->cam_params_.r_gain = 0;
    this->cam_params_.g_gain = 0;
    this->cam_params_.b_gain = 0;
    this->cam_params_.gamma = 0;
    this->cam_params_.contrast = 0;
    this->cam_params_.hue = 0;
    this->cam_params_.saturation = 0;
    this->cam_params_.hue = 0;
    this->cam_params_.fps = 0;

}

DahengCamera::~DahengCamera() {}




}
