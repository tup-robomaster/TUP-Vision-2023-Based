/*
 * @Description is a ros-based project!
 * @AuthorBiao
 * @Date-09-05 03:13:49
 * @LastEditTime: 2023-04-14 02:40:37
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

    struct TriggerSetting
    {
        unsigned int acquisition_mode;
        const char* acquisition_start;
        const char* acquisition_stop;
        unsigned int acquisition_burst_frame_count;
        unsigned int trigger_selector;
        unsigned int trigger_mode;
        unsigned int trigger_source_line;
        unsigned int trigger_activation;
        unsigned int trigger_delay;

        TriggerSetting()
        {
            acquisition_mode = 2;
            acquisition_start = "AcquisitionStart";
            acquisition_stop = "AcquisitionStop";
            acquisition_burst_frame_count = 1;
            trigger_selector = 10;
            trigger_mode = MV_TRIGGER_MODE_OFF; 
            trigger_source_line = MV_TRIGGER_SOURCE_LINE2;
            trigger_activation = 0;
            trigger_delay = 0.0;
        }
    };

    struct IoControlSetting
    {
        unsigned int line_selector;
        unsigned int line_mode;
        bool line_status;
        unsigned int trigger_selector;
        IoControlSetting()
        {
            line_selector = 2;
            line_mode = 8;
            line_status = false;
            trigger_selector = 10;
        }
    };
    
    class HikCamera
    {
    public:
        HikCamera();
        HikCamera(const CameraParam& cam_params);
        ~HikCamera();

        bool init();
        bool open();
        bool close();
        bool isOpen();

        bool getImage(cv::Mat &src, sensor_msgs::msg::Image& image_msg);
        bool setGain(int value, int exp_gain);
        bool setExposureTime(float exposure_time);
        bool setBalance(int value, unsigned int value_num);
        bool deviceReset();
    
    private:
        void startDevice(int serial_number);
        bool setStreamOn();
        bool setResolution(int width, int height);
        bool setAutoBalance();
        bool setGamma(bool set_status, double gamma_param);
        bool colorCorrect(bool value);
        bool setContrast(bool set_status, int contrast_param);
        bool updateTimestamp(rclcpp::Time time_start);
        int getTimestamp();
        bool setTriggerMode(TriggerSetting trigger_setting = TriggerSetting());
        bool setDigitalIoControl(IoControlSetting io_control_setting = IoControlSetting());
    
    public:
        // Camera params.
        CameraParam cam_param_;
    
    private:
        bool is_open_; 
        double timestamp_offset_ = 0;
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