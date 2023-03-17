/*
 * @Description is a ros-based project!
 * @AuthorBiao
 * @Date-09-05 03:13:49
 * @LastEditTime: 2023-03-14 20:23:26
 * @FilePath_2023/src/camera_driver/include/hik_driver/HikCamera.hpp
 */
//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

//c++
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>

#include <fstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

//opencv
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Dense>
#include <Eigen/Core>

//hik
#include "../../dependencies/hik_sdk/include/MvCameraControl.h"                         
#include "../../dependencies/hik_sdk/include/MvISPErrorDefine.h"                         
#include "../../dependencies/hik_sdk/include/MvErrorDefine.h"                         
#include "../../dependencies/hik_sdk/include/CameraParams.h"                         

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace camera_driver
{
    typedef enum _GAIN_MODE_
    {
        R_CHANNEL,
        G_CHANNEL,
        B_CHANNEL
    } GAIN_MODE;
    
    class HikCamera
    {
    public:
        HikCamera();
        HikCamera(const CameraParam& cam_params);
        ~HikCamera();

        bool init();
        bool open();
        bool close();
        bool is_open();

        bool get_frame(cv::Mat &src, sensor_msgs::msg::Image& image_msg);
        bool set_gain(int value, int exp_gain);
        bool set_exposure_time(float exposure_time);
        bool set_balance(int value, unsigned int value_num);
        bool deviceReset();
    
    private:
        void start_device(int serial_number);
        bool set_stream_on();
        bool set_resolution(int width, int height);
        bool set_auto_balance();
        bool set_gamma(bool set_status, double gamma_param);
        bool color_correct(bool value);
        bool set_contrast(bool set_status, int contrast_param);
        bool update_timestamp(rclcpp::Time time_start);
        int get_timestamp();

        bool set_trigger_mode(unsigned int acquisition_mode = 2,
            const char* acquisition_start = "AcquisitionStart",
            const char* acquisition_stop = "AcquisitionStop",
            unsigned int acquisition_burst_frame_count = 1,
            unsigned int trigger_selector = 10,
            unsigned int trigger_mode = MV_TRIGGER_MODE_OFF, 
            unsigned int trigger_source_line = MV_TRIGGER_SOURCE_LINE2,
            unsigned int trigger_activation = 0,
            unsigned int trigger_delay = 0.0);

        bool set_digital_io_control(unsigned int line_selector = 2,
            unsigned int line_mode = 8,
            bool line_status = false,
            unsigned int trigger_selector = 10);
    
    public:
        // Camera params.
        CameraParam cam_param_;
    
    private:
        bool _is_open; 
        double timestamp_offset = 0;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Time time_start_;
        rclcpp::Logger logger_;

    protected:
        int nRet = MV_OK;
        void* handle = NULL;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
        MV_FRAME_OUT_INFO_EX stImageInfo;
        MV_FRAME_OUT pFrame;
        bool g_bExit = false;
        unsigned int g_nPayloadSize;

        MVCC_FLOATVALUE frame_rate;
        MV_IMAGE_BASIC_INFO stFrameInfo;
    }; //HikCamera
} //camera_driver