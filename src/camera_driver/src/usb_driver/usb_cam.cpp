/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:03:26
 * @LastEditTime: 2022-09-28 17:26:36
 * @FilePath: /tup_2023/src/camera_driver/src/usb_cam.cpp
 */
#include "../../include/usb_driver/usb_cam.hpp"

namespace camera_driver
{
    usb_cam::usb_cam(usb_cam_params usb_params)
    {
        this->usb_cam_params_ = usb_params;
        // init();
    }

    usb_cam::~usb_cam()
    {

    }

    void usb_cam::init()
    {
        // this->usb_cam_params_.camera_id = device;
        cap.open(this->usb_cam_params_.camera_id);
        if(cap.isOpened())
        {
            this->is_open = true;
        }
    }

    // bool usb_cam::get_frame(cv::Mat &src)
    // {
    //     this->cap >> src;
    //     if(src.size().empty())
    //     {
    //         printf("grab image failed!");
    //         return false;
    //     }
    //     return true;
    // }

    // bool set_stream_on()
    // {
    //     return false;
    // }

    // bool set_resolution(int width, int height)
    // {
    //     return false;
    // }
    
    // bool set_exposure_time(float exposure_time)
    // {
    //     return false;
    // }
    // bool set_gain(int value, int exp_gain)
    // {
    //     return false;
    // }
    // bool set_auto_balance()
    // {
    //     return false;
    // }
    // bool set_balance(int value, unsigned int value_num)
    // {
    //     return false;
    // }
    // bool set_gamma(bool set_status, double gamma_param)
    // {
    //     return false;
    // }
    // bool color_correct(bool value)
    // {
    //     return false;
    // }
    // bool set_contrast(bool set_status, int contrast_param)
    // {
    //     return false;
    // }    
} //namespace usb_cam
