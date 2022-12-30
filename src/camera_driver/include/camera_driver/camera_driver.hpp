/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 00:29:49
 * @LastEditTime: 2022-12-30 21:37:17
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/camera_driver/camera_driver.hpp
 */
#ifndef CAMERA_DRIVER_HPP_
#define CAMERA_DRIVER_HPP_

//ros
#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

//opencv
#include <opencv2/opencv.hpp>

//c++
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>

// #include <fstream>
// #include <yaml-cpp/yaml.h>
// #include <fmt/format.h>
// #include <fmt/color.h>
// #include <glog/logging.h>
// #include <Eigen/Dense>
// #include <Eigen/Core>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace camera_driver
{
    class CameraBase
    {
    public:
        CameraBase();
        ~CameraBase();

        virtual void start_device(int serial_num) = 0;
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
        // cv::Mat src;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        int timestamp_offset_ = 0;
        rclcpp::Time time_start_;
    };

    template<class T>
    class CameraBaseNode : public rclcpp::Node
    {
    public:
        explicit CameraBaseNode(const rclcpp::NodeOptions& options);
        ~CameraBaseNode();
    
    public:
        void image_callback();
        std::unique_ptr<ImageMsg> convert_frame_to_msg(cv::Mat frame);

        std::unique_ptr<T> cam_driver_;
        virtual std::unique_ptr<T> init_cam_driver()
        {
            return std::make_unique<T>();
        }
    
    public:
        std::map<std::string, int> param_map_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    private:    
        bool using_shared_memory_;   //图像数据内存共享
        SharedMemoryParam shared_memory_param_;   
        std::thread memory_write_thread_;
    };
    
} //namespace camera_driver
#endif