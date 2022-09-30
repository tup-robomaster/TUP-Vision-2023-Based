/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:03:26
 * @LastEditTime: 2022-09-28 17:26:36
 * @FilePath: /tup_2023/src/camera_driver/src/usb_cam.cpp
 */
#include "../include/usb_cam.hpp"

namespace camera_driver
{
    usb_cam::usb_cam(std::string id)
    {
        this->device_path = id;
        init(this->device_path);
    }

    usb_cam::~usb_cam()
    {

    }

    void usb_cam::init(std::string device)
    {
        this->device_path = device;
        cap.open(this->device_path);
        if(cap.isOpened())
        {
            this->is_open = true;
        }
    }

    bool usb_cam::get_frame(cv::Mat &src)
    {
        this->cap >> src;
        if(src.size().empty())
        {
            printf("grab image failed!");
            return false;
        }
        return true;
    }

    bool set_stream_on()
    {
        return false;
    }

    bool set_resolution(int width, int height)
    {
        return false;
    }
    
    bool set_exposure_time(float exposure_time)
    {
        return false;
    }
    bool set_gain(int value, int exp_gain)
    {
        return false;
    }
    bool set_auto_balance()
    {
        return false;
    }
    bool set_balance(int value, unsigned int value_num)
    {
        return false;
    }
    bool set_gamma(bool set_status, double gamma_param)
    {
        return false;
    }
    bool color_correct(bool value)
    {
        return false;
    }
    bool set_contrast(bool set_status, int contrast_param)
    {
        return false;
    }    
} //namespace usb_cam
