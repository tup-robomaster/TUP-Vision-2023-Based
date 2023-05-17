/*
 * @Description: This is a ros-based project!
 * @Author: Leo
 * @Date: 2023-04-29 12:02:22
 * @LastEditTime: 2023-05-03 16:08:59
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/mvs_driver/mvs_camera.hpp
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

//mvs
#include "../../dependencies/mvs_sdk/include/CameraApi.h"

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
using namespace cv;

namespace camera_driver
{
    class MvsCamera {
    public:
        MvsCamera();
        MvsCamera(const CameraParam& cam_params);
        ~MvsCamera();

        bool init();
        bool open();
        bool close();
        bool isOpen();
        bool getImage(cv::Mat &src, sensor_msgs::msg::Image& image_msg);
        
        void setGain(int value, int exp_gain);
        void setExposureTime(float exposure_time);
        void setBalance(int value, unsigned int value_num);
        void setResolution(int width, int height); // TODO
        void deviceReset();

    public:
        int iCameraCounts = 1;
        int hCamera;
        tSdkCameraDevInfo tCameraEnumList;
        tSdkCameraCapbility tCapability;
        tSdkFrameHead sFrameInfo;
        BYTE* pbyBuffer;
        BYTE* g_pRgbBuffer;     //处理后数据缓存区
        CameraSdkStatus status;
        // BYTE* pFrameBuffer;
        // Mat frame;

    public:
        // Camera params.
        CameraParam cam_param_;
        bool is_camera_initialized_ = false;

    private:
        bool is_open_; 
        double timestamp_offset_ = 0;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Time time_start_;
        rclcpp::Logger logger_;
    };
}