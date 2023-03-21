/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:03:26
 * @LastEditTime: 2023-03-10 20:44:40
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam.cpp
 */
#include "../../include/usb_driver/usb_cam.hpp"

namespace camera_driver
{
    UsbCam::UsbCam(const CameraParam& usb_params)
    : logger_(rclcpp::get_logger("usb_driver"))
    {
        // cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
        // cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_height);

        this->usb_cam_params_ = usb_params;
    }

    UsbCam::UsbCam()
    : logger_(rclcpp::get_logger("usb_driver"))
    {
        // cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
        // cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_height);
    }

    UsbCam::~UsbCam()
    {

    }

    bool UsbCam::open()
    {
        if (!this->usb_cam_params_.using_video)
        {
            cap.open(this->usb_cam_params_.cam_id);
            RCLCPP_INFO(logger_, "[USB CAMERA] ID:%d", this->usb_cam_params_.cam_id);
            if (cap.isOpened())
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
            cout << "Video path:" << this->usb_cam_params_.video_path << endl;
            // RCLCPP_INFO(logger_, "[Video path:] %s", this->usb_cam_params_.video_path);
            if (cap.isOpened())
            {
                cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
                cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_height);
                this->is_open = true;
                return true;
            }
            else
            {
                return false;
            }
            // return true;
        }
    }

    bool UsbCam::get_frame(cv::Mat src, sensor_msgs::msg::Image& image_msg)
    {
        // if (!cap.isOpened())
        // {
        //     cap.open(this->usb_cam_params_.video_path);
        //     cout << "Video path:" << this->usb_cam_params_.video_path << endl;
        //     // RCLCPP_INFO(logger_, "[Video path:] %s", this->usb_cam_params_.video_path);
        //     if (cap.isOpened())
        //     {
        //         this->is_open = true;
        //         // return true;
        //     }
        // }

        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            return false;
        image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);  
        image_msg.is_bigendian = false;
        image_msg.data.assign(frame.datastart, frame.dataend);
        return true;
    }
} //namespace UsbCam
