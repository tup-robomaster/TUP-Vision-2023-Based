/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:13:49
 * @LastEditTime: 2022-09-06 18:04:28
 * @FilePath: /tup_2023/src/camera_driver/include/hik_camera.h
 */
#include "camera_driver/camera_driver.hpp"
#include "../dependencies/hik_sdk/include/MvCameraControl.h"

#define nRetError(status, ...)          \
    if(status != MV_OK)                 \
    {                                   \
        RCLCPP_FATAL(                   \
            this->node->get_logger(),   \
            ##__VA_ARGS__);             \
        return false;                   \
    }                                   

namespace camera_driver
{
    typedef enum _GAIN_MODE_
    {
        R_CHANNEL,
        G_CHANNEL,
        B_CHANNEL
    } GAIN_MODE;

    class hik_camera : public camera_driver
    {
    public:
        hik_camera(rclcpp::Node::SharedPtr node);
        ~hik_camera();

        bool open() override;
        bool close() override;
        bool is_open() override;

        int start_device(int serial_number) override;
        bool set_stream_on() override;
        bool set_resolution(int width, int height) override;
        bool set_exposure_time(float exposure_time) override;
        bool set_gain(int value, int exp_gain) override;
        bool set_auto_balance() override;
        bool set_balance(int value, unsigned int value_num) override;
        bool set_gamma(bool set_status, double gamma_param) override;
        bool color_correct(bool value) override;
        bool set_contrast(bool set_status, int contrast_param) override;
        bool update_timestamp(std::chrono::_V2::steady_clock::time_point time_start) override;
        bool get_frame(cv::Mat &src) override;
        int get_timestamp() override;
    
    public:
        bool _is_open; 
        rclcpp::Node::SharedPtr node; // 相机节点

        int nRet = MV_OK;
        void* handle = NULL;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_FRAME_OUT pFrame = {0};
        bool g_bExit = false;
        unsigned int g_nPayloadSize = 0;

    }; //hik_camera
} //camera_driver
