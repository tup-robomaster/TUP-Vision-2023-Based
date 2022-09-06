/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 00:29:49
 * @LastEditTime: 2022-09-06 18:03:35
 * @FilePath: /tup_2023/src/camera_driver/include/camera_driver/camera_driver.hpp
 */
#include "global_user/include/global_user/global_user.hpp"
#include "rmoss_master/rmoss_core/rmoss_cam/include/rmoss_cam/cam_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camera_driver
{
    class camera_driver : public rmoss_cam::CamInterface
    {
    public:
        camera_driver();
        virtual ~camera_driver() = 0;

        virtual int start_device(int serial_num) = 0;
        virtual bool set_stream_on() = 0;
        virtual bool set_resolution(int width, int height) = 0;
        virtual bool set_exposure_time(float exposure_time) = 0;
        virtual bool set_gain(int value, int exp_gain) = 0;
        virtual bool set_auto_balance() = 0;
        virtual bool set_balance(int value, unsigned int value_num) = 0;
        virtual bool set_gamma(bool set_status, double gamma_param) = 0;
        virtual bool color_correct(bool value) = 0;
        virtual bool set_contrast(bool set_status, int contrast_param) = 0;
        virtual bool update_timestamp(std::chrono::_V2::steady_clock::time_point time_start) = 0;
        virtual bool get_frame(cv::Mat &src) = 0;
        virtual int get_timestamp() = 0;
    
    public:
        cv::Mat src;
        int timestamp_offset = 0;
        std::chrono::_V2::steady_clock::time_point time_start;
    };
};
