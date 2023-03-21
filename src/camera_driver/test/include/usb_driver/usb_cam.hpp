/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 16:51:24
 * @LastEditTime: 2023-03-10 19:53:52
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam.hpp
 */
//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

//c++
#include <iostream>

//opencv
#include <opencv2/opencv.hpp>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace camera_driver
{
    class UsbCam
    {
    private:     
        rclcpp::Logger logger_;

    public:
        cv::Mat src;
        bool is_open;
        cv::VideoCapture cap;
        CameraParam usb_cam_params_;

    public:
        UsbCam();
        UsbCam(const CameraParam& usb_params);
        ~UsbCam();

        bool open();
        bool get_frame(cv::Mat src, sensor_msgs::msg::Image& image_msg);
    }; // usb_cam
} // camera_driver