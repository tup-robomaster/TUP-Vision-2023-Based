/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 16:49:59
 * @LastEditTime: 2022-11-08 21:56:08
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

//custom message
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/armor.hpp"
#include "global_interface/msg/armors.hpp"
#include "global_interface/msg/target.hpp"

namespace armor_detector
{
    class detector_node : public rclcpp::Node
    {
        typedef std::chrono::_V2::steady_clock::time_point TimePoint;
        typedef global_interface::msg::Armors ArmorsMsg;
        typedef global_interface::msg::Target TargetMsg;

    public:
        detector_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~detector_node();
        
    private:
        //订阅图像  
        std::shared_ptr<image_transport::Subscriber> img_sub;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;

        // Image subscriptions transport type
        std::string transport_;
        TimePoint time_start;
        
        // std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info;
        // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;

        //发布装甲信息
        ArmorsMsg armors_info;
        rclcpp::Publisher<TargetMsg>::SharedPtr armors_pub;
    
    private:    
        //params callback
        rclcpp::TimerBase::SharedPtr param_timer_;
        void param_callback();
        
    public:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        // std::vector<Armor> detect_armors(const sensor_msgs::msg::Image::SharedPtr& img);
  
    public:
        detector_params detector_params_;
        debug_params debug_;
        gyro_params gyro_params_;
        void getParameters();

        // rclcpp::Node handle;
        // image_transport::ImageTransport it;
        std::unique_ptr<detector> detector_;
        std::unique_ptr<detector> init_detector();  
    };
} //namespace detector