/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 22:57:12
 * @LastEditTime: 2023-02-06 01:09:01
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/buff_detector_node.hpp
 */
#ifndef BUFF_DETECTOR_NODE_HPP_
#define BUFF_DETECTOR_NODE_HPP_

#include "./buff_detector/buff_detector.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>

//c++
#include <atomic>
#include <thread>
#include <mutex>

//custom message
#include "global_interface/msg/buff.hpp"
#include "global_interface/msg/serial.hpp"

using namespace global_user;
using namespace coordsolver;
namespace buff_detector
{
    class BuffDetectorNode : public rclcpp::Node
    {
        typedef global_interface::msg::Buff BuffMsg;
        typedef global_interface::msg::Serial SerialMsg;

    public:
        BuffDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~BuffDetectorNode();
    
    private:
        BuffParam buff_param_;
        PathParam path_param_;
        DebugParam debug_param_;

        std::unique_ptr<Detector> detector_;
        std::unique_ptr<Detector> init_detector();
    
    private:
        std::string transport_;
        // Subscribe images from camera node.
        std::shared_ptr<image_transport::Subscriber> img_sub_; 
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);

        Mutex mutex_;
        SerialMsg serial_msg_;
        rclcpp::Subscription<SerialMsg>::SharedPtr serial_msg_sub_;
        void sensorMsgCallback(const SerialMsg& serial_msg);
    private:
        rclcpp::Time time_start_;
        // rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Publisher<BuffMsg>::SharedPtr buff_info_pub_; //buff msgs pub.
    
    protected:
        // params callback.
        std::map<std::string, int> param_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(const rclcpp::Parameter& param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);

    private:
        // Shared memory.
        bool using_shared_memory_;          
        SharedMemoryParam shared_memory_param_;
        std::thread read_memory_thread_;    //共享内存读线程
    };
} // namespace buff_detector

#endif