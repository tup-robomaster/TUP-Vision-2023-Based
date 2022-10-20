/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 16:51:24
 * @LastEditTime: 2022-09-29 00:02:39
 * @FilePath: /tup_2023/src/camera_driver/include/usb_cam.hpp
 */
// #include "camera_driver/camera_driver.hpp"
#include "global_user/global_user.hpp"
#include "global_interface/msg/gimbal.hpp"

namespace camera_driver
{
    struct usb_cam_params
    {
        std::string frame_id;
        int camera_id;
        int image_width;
        int image_height;
        int fps;
    };
    
    class usb_cam
    {
    public:     
        // std::string device_path;
        usb_cam_params usb_cam_params_;

    public:
        cv::VideoCapture cap;
        bool is_open;
        cv::Mat src;

    public:
        // usb_cam(){}
        usb_cam(usb_cam_params usb_params);
        ~usb_cam();

        // void get_params();
        void init();
        // int start_device(int serial_num) override;
        // bool update_timestamp(std::chrono::_V2::steady_clock::time_point time_start) override;
        // bool get_frame(cv::Mat &src) override;
        // int get_timestamp() override;

        // bool set_stream_on() override;
        // bool set_resolution(int width, int height) override;
        // bool set_exposure_time(float exposure_time) override;
        // bool set_gain(int value, int exp_gain) override;
        // bool set_auto_balance() override;
        // bool set_balance(int value, unsigned int value_num) override;
        // bool set_gamma(bool set_status, double gamma_param) override;
        // bool color_correct(bool value) override;
        // bool set_contrast(bool set_status, int contrast_param) override;    
    }; // usb_cam
} // camera_driver