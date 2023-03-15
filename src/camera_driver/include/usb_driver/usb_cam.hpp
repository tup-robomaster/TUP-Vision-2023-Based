/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 16:51:24
 * @LastEditTime: 2023-03-15 10:00:00
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam.hpp
 */
//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

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
    private:     
        UsbCamParam usb_cam_params_;
        rclcpp::Logger logger_;

    public:
        cv::VideoCapture cap;
        bool is_open;
        cv::Mat src;

    public:
        UsbCam();
        UsbCam(UsbCamParam usb_params);
        ~UsbCam();

        // bool open();
        bool get_frame(cv::Mat &src);
        void init();
    }; // usb_cam
} // camera_driver