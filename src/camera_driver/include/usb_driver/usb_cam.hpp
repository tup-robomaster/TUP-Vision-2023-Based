/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 16:51:24
 * @LastEditTime: 2023-04-14 02:49:20
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam.hpp
 */
//ros
#include <rclcpp/rclcpp.hpp>

//c++
#include <iostream>

//opencv
#include <opencv2/opencv.hpp>

namespace camera_driver
{
    struct UsbCamParam
    {
        std::string frame_id;
        int camera_id;
        int image_width;
        int image_height;
        int fps;
    };
    
    class UsbCam
    {
    public:
        UsbCam();
        UsbCam(UsbCamParam usb_params);
        ~UsbCam();

        void init();

        // void get_params();
        // int start_device(int serial_num);
        // bool get_frame(cv::Mat &src);

    private:     
        // std::string device_path;
        UsbCamParam usb_cam_params_;
        rclcpp::Logger logger_;

    public:
        cv::VideoCapture cap_;
        bool is_open_;
        cv::Mat src_;
    }; // usb_cam
} // camera_driver