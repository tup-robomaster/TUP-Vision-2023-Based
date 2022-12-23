/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 16:51:24
 * @LastEditTime: 2022-12-23 22:02:52
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam.hpp
 */
// #include "camera_driver/camera_driver.hpp"
#include "global_user/global_user.hpp"
#include "global_interface/msg/gimbal.hpp"

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
        // std::string device_path;
        UsbCamParam usb_cam_params_;

    public:
        cv::VideoCapture cap;
        bool is_open;
        cv::Mat src;

    public:
        UsbCam();
        UsbCam(UsbCamParam usb_params);
        ~UsbCam();

        void init();

        // void get_params();
        // int start_device(int serial_num);
        // bool get_frame(cv::Mat &src);
    }; // usb_cam
} // camera_driver