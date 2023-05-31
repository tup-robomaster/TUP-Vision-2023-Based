/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 22:57:12
 * @LastEditTime: 2023-05-31 20:04:40
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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

        // visualization_msgs::Marker
        void pubMarkerArray(vector<geometry_msgs::msg::Transform> armor3d_transform_vec, int flag, rclcpp::Time stamp);
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
        uint64 shape_ = visualization_msgs::msg::Marker::CUBE;
    
    protected:
        // Params callback.
        bool updateParam();
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };
} // namespace buff_detector

#endif