/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 22:57:12
 * @LastEditTime: 2023-06-06 21:08:36
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
#include <ament_index_cpp/get_package_share_directory.hpp>
//c++
#include <atomic>
#include <thread>
#include <mutex>

//custom message
#include "global_interface/msg/buff.hpp"
#include "global_interface/msg/serial.hpp"

using namespace global_user;
using namespace coordsolver;
using namespace ament_index_cpp;
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
        Mutex param_mutex_;
        BuffParam buff_param_;
        PathParam path_param_;
        DebugParam debug_param_;

        std::unique_ptr<Detector> detector_;
        std::unique_ptr<Detector> initDetector();
    
    private:
        // Subscribe images from camera node.
        std::shared_ptr<image_transport::Subscriber> img_msg_sub_; 
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        Mutex serial_mutex_;
        SerialMsg serial_msg_;
        rclcpp::Subscription<SerialMsg>::SharedPtr serial_msg_sub_;
        void sensorMsgCallback(const SerialMsg& serial_msg);

        // Buff msgs pub.
        rclcpp::Publisher<BuffMsg>::SharedPtr buff_msg_pub_; 
        int mode_ = 1;

        Eigen::Vector3d last_center3d_ = {0.0, 0.0, 0.0};
        Eigen::Vector3d last_point3d_cam_ = {0.0, 0.0, 0.0};
        Eigen::Vector3d last_point3d_world_ = {0.0, 0.0, 0.0};
        double last_rotate_speed_ = 0.0;

    protected:
        // Params callback.
        bool updateParam();
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };
} // namespace buff_detector

#endif