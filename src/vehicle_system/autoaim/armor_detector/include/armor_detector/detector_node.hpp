/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 16:49:59
 * @LastEditTime: 2022-10-28 10:00:57
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/armor_detector/detector_node.hpp
 */
#include "../../global_user/include/global_user/global_user.hpp"
#include "./detector.hpp"

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
//tf2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <kdl/frames.hpp> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//
#include <iostream>
#include <chrono>
#include <string>
#include <memory>

//custom message
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/armor.hpp"
#include "global_interface/msg/armors.hpp"
#include "global_interface/msg/target.hpp"

typedef std::chrono::duration<int> SecondsType;

namespace armor_detector
{
    class detector_node : public rclcpp::Node
    {
    public:
        detector_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~detector_node();
        
    private:
        //订阅图像  
        std::shared_ptr<image_transport::Subscriber> img_sub_;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;

        // Image subscriptions transport type
        std::string transport_;
        std::chrono::_V2::steady_clock::time_point time_start_;
        
        // std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info;
        // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;

        //发布装甲信息
        global_interface::msg::Armors armors_info_;
        rclcpp::Publisher<global_interface::msg::Target>::SharedPtr armors_pub_;

        // message_filters::Publisher<geometry_msgs::msg::PointStamped> armor_point_pub_;

    public:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        // std::vector<Armor> detect_armors(const sensor_msgs::msg::Image::SharedPtr& img);
  
    public:
        // rclcpp::Node handle;
        // image_transport::ImageTransport it;
        std::unique_ptr<detector> detector_;
        std::unique_ptr<detector> init_detector();  
    
    private:
        //tf2
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::string target_frame_;
        std::shared_ptr<tf2_ros::MessageFilter<global_interface::msg::Target>> tf2_filter_;
    };
} //namespace detector