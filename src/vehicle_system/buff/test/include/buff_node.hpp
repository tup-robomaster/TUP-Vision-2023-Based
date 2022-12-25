/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-11 11:30:51
 * @LastEditTime: 2022-12-24 18:32:23
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/test/include/buff_node.hpp
 */
#include "./buff/buff.hpp"

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

using namespace global_user;
using namespace coordsolver;
namespace buff
{
    class BuffNode : public rclcpp::Node
    {
        typedef global_interface::msg::Target TargetMsg;
        typedef global_interface::msg::Armors ArmorsMsg;
        typedef global_interface::msg::Gimbal GimbalMsg;

    public:
        BuffNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~BuffNode();
    
    private:
        std::unique_ptr<Buff> buff_;
        std::unique_ptr<Buff> init_buff();
    
    private:
        std::string transport_;
        std::shared_ptr<image_transport::Subscriber> img_sub_; //订阅图像  
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
    
    private:
        rclcpp::Time time_start_;

        // gimbal msgs pub
        rclcpp::Publisher<TargetMsg>::SharedPtr buff_info_pub_;
        rclcpp::Publisher<TargetMsg>::SharedPtr predict_info_pub_;
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_info_pub_;
    
    protected:
        BuffParam buff_param_;
        PredictorParam predictor_param_;
        PathParam path_param_;
        DebugParam debug_param_;

        //params callback
        rclcpp::TimerBase::SharedPtr param_timer_;
        void param_callback();
    
    private:
        /**
         * @brief 共享图像数据内存
         * 
         */
        void run();                         //
        bool using_shared_memory_;          //
        key_t key_;                         //生成key键
        int shared_memory_id_;              //获取共享内存id
        void* shared_memory_ptr_ = nullptr; //映射共享内存，得到虚拟地址
        std::thread read_memory_thread_;    //共享内存读线程
    };
} //namespace buff