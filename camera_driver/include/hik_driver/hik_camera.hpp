/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:13:49
 * @LastEditTime: 2022-09-30 10:23:29
 * @FilePath: /tup_2023/src/camera_driver/include/hik_driver/hik_camera.hpp
 */
// #include "../camera_driver/camera_driver.hpp"

#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <thread>
#include <memory>
#include <string>
#include <iterator>
#include <unistd.h>
#include <string>

#include <fstream>
#include <yaml-cpp/yaml.h>
// #include <fmt/format.h>
// #include <fmt/color.h>
#include <glog/logging.h>

// #include <exception/exception.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../dependencies/hik_sdk/include/MvCameraControl.h"                         
#include "../../dependencies/hik_sdk/include/MvISPErrorDefine.h"                         
#include "../../dependencies/hik_sdk/include/MvErrorDefine.h"                         
#include "../../dependencies/hik_sdk/include/CameraParams.h"                         

namespace camera_driver
{
    typedef enum _GAIN_MODE_
    {
        R_CHANNEL,
        G_CHANNEL,
        B_CHANNEL
    } GAIN_MODE;

    class hik_camera
    {
    public:
        hik_camera();
        ~hik_camera();

        bool open();
        bool close();
        bool is_open();

        void start_device(int serial_number);
        bool set_stream_on();
        bool set_resolution(int width, int height);
        bool set_exposure_time(float exposure_time);
        bool set_gain(int value, int exp_gain);
        bool set_auto_balance();
        bool set_balance(int value, unsigned int value_num);
        bool set_gamma(bool set_status, double gamma_param);
        bool color_correct(bool value);
        bool set_contrast(bool set_status, int contrast_param);
        bool update_timestamp(std::chrono::_V2::steady_clock::time_point time_start);
        bool get_frame(cv::Mat &src);
        int get_timestamp();
    
    public:
        bool _is_open; 
        int timestamp_offset = 0;
        std::chrono::_V2::steady_clock::time_point time_start;

        int nRet = MV_OK;
        void* handle = NULL;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
        MV_FRAME_OUT_INFO_EX stImageInfo;
        MV_FRAME_OUT pFrame;
        bool g_bExit = false;
        unsigned int g_nPayloadSize;

    }; //hik_camera
} //camera_driver