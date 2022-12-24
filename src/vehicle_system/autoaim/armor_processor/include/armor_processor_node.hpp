/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:56:35
 * @LastEditTime: 2022-12-24 19:05:08
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor_node.hpp
 */
#ifndef ARMOR_PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR_NODE_HPP_

#include "./armor_processor/armor_processor.hpp"

#include "../../global_user/include/global_user/global_user.hpp"
#include "global_interface/msg/spin_info.hpp"
#include "global_interface/msg/gimbal.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>

//std
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace global_user;
using namespace coordsolver;
namespace armor_processor
{
    class ArmorProcessorNode : public rclcpp::Node 
    {
        typedef global_interface::msg::Target TargetMsg;
        typedef global_interface::msg::SpinInfo SpinMsg;
        typedef global_interface::msg::Gimbal GimbalMsg;

    public:
        explicit ArmorProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ArmorProcessorNode();

    private:
        // rclcpp::Subscription<SpinInfo>::SharedPtr spin_info_sub;
        // void spin_info_callback(const SpinMsg::SharedPtr msg) const;
        
        // message_filters::Subscriber<TargetMsg> target_info_sub;
        rclcpp::Subscription<TargetMsg>::SharedPtr target_info_sub_;
        message_filters::Subscriber<TargetMsg> target_point_sub_; 
        void target_info_callback(const TargetMsg& target_info);

        // bool draw_predict;
        Eigen::Vector3d last_predict_point_;
        Eigen::Vector3d predict_point_;
        cv::Point2f apex2d[4];
        // coordsolver::coordsolver coordsolver_;
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_info_pub_;
    
    private:
        std::unique_ptr<Processor> processor_;
        std::unique_ptr<Processor> init_armor_processor();

    private:
        //tf2 transformation
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;

        void msg_callback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr);
    
    protected:
        // sub image.
        std::shared_ptr<image_transport::Subscriber> img_sub_;
        // Image subscriptions transport type.
        std::string transport_;
        int image_width;
        int image_height;
        
        // image callback.
        void img_callback();
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
    
    public:
        PredictParam predict_param_;
        SingerModel singer_model_param_;
        DebugParam debug_param_;
        std::string filter_param_path_;
        std::string coord_param_path_;
        std::string coord_param_name_;

    private:
        std::map<std::string, int> params_map_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        
        // std::shared_ptr<ParamSubcriber> cb_;
        // std::shared_ptr<ParamCbHandle> param_cb_;
    protected:
        // 共享图像数据内存
        bool using_shared_memory;
        SharedMemoryParam shared_memory_param_;
        std::thread read_memory_thread_; //共享内存读线程
    };
} //armor_processor

#endif