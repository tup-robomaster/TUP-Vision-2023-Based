/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 16:51:24
 * @LastEditTime: 2023-03-07 19:14:32
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam.hpp
 */
//ros
#include <rclcpp/rclcpp.hpp>

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
        bool init();
        bool close();
        bool get_frame(cv::Mat &src);
    }; // usb_cam
} // camera_driver