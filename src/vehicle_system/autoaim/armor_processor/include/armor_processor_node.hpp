/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:56:35
 * @LastEditTime: 2023-04-14 03:11:33
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor_node.hpp
 */
#ifndef ARMOR_PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR_NODE_HPP_

//ros
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

//std
#include <mutex>
#include <atomic>
#include <thread>

#include "./armor_processor/armor_processor.hpp"
#include "../../global_user/include/global_user/global_user.hpp"

#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/car_pos.hpp"
#include "global_interface/msg/obj_hp.hpp"
#include "global_interface/msg/game_info.hpp"

using namespace global_user;
using namespace coordsolver;
using namespace message_filters;
using namespace ament_index_cpp;
namespace armor_processor
{
    class ArmorProcessorNode : public rclcpp::Node 
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef global_interface::msg::Gimbal GimbalMsg;
        typedef global_interface::msg::ObjHP ObjHPMsg;
        typedef global_interface::msg::CarPos CarPosMsg;
        typedef global_interface::msg::GameInfo GameMsg;
        typedef sync_policies::ApproximateTime<sensor_msgs::msg::Image, AutoaimMsg> MySyncPolicy;

    public:
        explicit ArmorProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ArmorProcessorNode();

    private:
        rclcpp::Subscription<AutoaimMsg>::SharedPtr target_info_sub_;
        void targetMsgCallback(const AutoaimMsg& target_info);
        bool processTargetMsg(const AutoaimMsg& target_info, cv::Mat* src = nullptr);

        mutex debug_mutex_;
        mutex image_mutex_;
        cv::Mat src_;
        atomic<bool> flag_ = false;
        bool is_aimed_ = false;
        bool is_pred_ = false;
        bool is_pred_failed_ = false;
        int count_ = 0;
        
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_info_pub_;
        rclcpp::Publisher<GimbalMsg>::SharedPtr tracking_info_pub_;
        rclcpp::Publisher<AutoaimMsg>::SharedPtr predict_info_pub_;
        
        // message_filter
        MySyncPolicy my_sync_policy_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img_msg_sync_sub_;
        std::shared_ptr<message_filters::Subscriber<AutoaimMsg>> target_msg_sync_sub_;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
        void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const AutoaimMsg::ConstSharedPtr& target_msg);
        bool sync_transport_ = false;

    private:
        std::unique_ptr<Processor> processor_;
        std::unique_ptr<Processor> initArmorProcessor();

    protected:
        ImageSize image_size_;
        ImageInfo image_info_;

        // Image callback.
        void imageProcessor(cv::Mat& img);
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        
        // Sub image.
        std::shared_ptr<image_transport::Subscriber> img_sub_;
    
    public:
        mutex param_mutex_;
        bool debug_;
        PredictParam predict_param_;
        vector<double> singer_param_;
        DebugParam debug_param_;
        PathParam path_param_;

    private:
        bool updateParam();
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };
} //armor_processor

#endif