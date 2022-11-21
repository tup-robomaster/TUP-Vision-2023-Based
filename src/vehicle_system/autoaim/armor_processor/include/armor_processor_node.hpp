/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:56:35
 * @LastEditTime: 2022-11-21 11:04:33
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor_node.hpp
 */
#ifndef ARMOR_PROCESSOR_NODE_HPP
#define ARMOR_PROCESSOR_NODE_HPP

#include "./armor_processor/armor_processor.hpp"
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

//
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>

//std
#include <memory>
#include <string>
#include <vector>

//linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define DAHENG_IMAGE_WIDTH 1280
#define DAHENG_IMAGE_HEIGHT 1024

typedef std::chrono::duration<int> SecondsType;

namespace armor_processor
{
    class ArmorProcessorNode : public rclcpp::Node 
    {
    public:
        explicit ArmorProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ArmorProcessorNode();

    private:
        // rclcpp::Subscription<global_interface::msg::SpinInfo>::SharedPtr spin_info_sub;
        // void spin_info_callback(const global_interface::msg::SpinInfo::SharedPtr msg) const;
        
        // message_filters::Subscriber<global_interface::msg::Target> target_info_sub;
        rclcpp::Subscription<global_interface::msg::Target>::SharedPtr target_info_sub_;
        message_filters::Subscriber<global_interface::msg::Target> target_point_sub_; 
        void target_info_callback(const global_interface::msg::Target& target_info);

        // bool draw_predict;
        Eigen::Vector3d last_predict_point_;
        Eigen::Vector3d predict_point_;
        // coordsolver::coordsolver coordsolver_;
        rclcpp::Publisher<global_interface::msg::Gimbal>::SharedPtr gimbal_info_pub_;
    
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
        // 订阅图像  
        std::shared_ptr<image_transport::Subscriber> img_sub_;
        // Image subscriptions transport type
        std::string transport_;
        
        //
        void img_callback();
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
    
    public:
        PredictParam predict_param_;
        // SingerModelParam singer_model_param_;
        SingerModel singer_model_param_;
        DebugParam debug_param_;
        std::string filter_param_path_;
        std::string coord_param_path_;
        std::string coord_param_name_;

    private:
        /**
         * @brief 动态调参
         * @param 参数服务器参数
         * @return 是否修改参数成功
         */
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

        // std::shared_ptr<ParamSubcriber> cb_;
        // std::shared_ptr<ParamCbHandle> param_cb_;
    
    protected:
        /**
         * @brief 共享图像数据内存
         * 
         */
        //
        bool using_shared_memory;
        
        //生成key键
        key_t key_;

        //获取共享内存id
        int shared_memory_id_;

        //映射共享内存，得到虚拟地址
        void* shared_memory_ptr_ = nullptr;

        //共享内存读线程
        std::thread read_memory_thread_;

    };
} //armor_processor

#endif