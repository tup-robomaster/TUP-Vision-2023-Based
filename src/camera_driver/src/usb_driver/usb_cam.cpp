/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:03:26
 * @LastEditTime: 2023-04-14 02:50:54
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam.cpp
 */
#include "../../include/usb_driver/usb_cam.hpp"

namespace camera_driver
{
    UsbCam::UsbCam(UsbCamParam usb_params)
    : logger_(rclcpp::get_logger("usb_driver"))
    {
        this->usb_cam_params_ = usb_params;
        // init();
    }

    UsbCam::UsbCam()
    : logger_(rclcpp::get_logger("usb_driver"))
    {

    }

    UsbCam::~UsbCam()
    {

    }

    void UsbCam::init()
    {
        // this->usb_cam_params_.camera_id = device;
        cap_.open(this->usb_cam_params_.camera_id);
        RCLCPP_INFO(logger_, "[USB CAMERA] ID:%d", this->usb_cam_params_.camera_id);
        if(cap_.isOpened())
        {
            this->is_open_ = true;
        }
    }

    // bool UsbCam::get_frame(cv::Mat &src)
    // {
    //     this->cap_ >> src;
    //     if(src.size().empty())
    //     {
    //         printf("grab image failed!");
    //         return false;
    //     }
    //     return true;
    // }   
} //namespace UsbCam
