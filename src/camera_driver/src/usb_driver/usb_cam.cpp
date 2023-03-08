/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:03:26
 * @LastEditTime: 2023-02-26 13:54:44
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam.cpp
 */
#include "../../include/usb_driver/usb_cam.hpp"

namespace camera_driver
{
    UsbCam::UsbCam(const CameraParam& usb_params)
    : logger_(rclcpp::get_logger("usb_driver"))
    {
        this->usb_cam_params_ = usb_params;
    }

    UsbCam::UsbCam()
    : logger_(rclcpp::get_logger("usb_driver"))
    {

    }

    UsbCam::~UsbCam()
    {

    }

    bool UsbCam::open()
    {
        if(!this->usb_cam_params_.using_video)
        {
            cap.open(this->usb_cam_params_.cam_id);
            RCLCPP_INFO(logger_, "[USB CAMERA] ID:%d", this->usb_cam_params_.cam_id);
            if(cap.isOpened())
            {
                this->is_open = true;
                return true;
            }
            else
                return false;
        }
        else
        {
            cap.open(this->usb_cam_params_.video_path);
            RCLCPP_INFO(logger_, "[Video path:] %s", this->usb_cam_params_.video_path);
            if(cap.isOpened())
            {
                this->is_open = true;
                return true;
            }
            else
                return false;
        }
    }

    bool UsbCam::get_frame(cv::Mat &src, sensor_msgs::msg::Image& image_msg)
    {
        cap >> src;
        if(src.empty())
            return false;
        else
            return true;
    }
} //namespace UsbCam
