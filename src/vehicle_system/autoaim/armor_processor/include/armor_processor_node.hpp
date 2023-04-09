/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:56:35
 * @LastEditTime: 2023-04-08 20:02:17
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor_node.hpp
 */
#ifndef ARMOR_PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR_NODE_HPP_

//ros
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

//std
#include <mutex>
#include <atomic>
#include <thread>

#include "./armor_processor/armor_processor.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/gimbal.hpp"

using namespace global_user;
using namespace coordsolver;
using namespace ament_index_cpp;
namespace armor_processor
{
    class ArmorProcessorNode : public rclcpp::Node 
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef global_interface::msg::Gimbal GimbalMsg;

    public:
        explicit ArmorProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~ArmorProcessorNode();

    private:
        rclcpp::Subscription<AutoaimMsg>::SharedPtr target_info_sub_;
        void targetMsgCallback(const AutoaimMsg& target_info);

        mutex debug_mutex_;
        atomic<bool> flag_;
        cv::Point2f apex2d[4];
        Eigen::Vector3d predict_point_;
        
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_info_pub_;
        rclcpp::Publisher<GimbalMsg>::SharedPtr tracking_info_pub_;
        rclcpp::Publisher<AutoaimMsg>::SharedPtr predict_info_pub_;
    
    private:
        std::unique_ptr<Processor> processor_;
        std::unique_ptr<Processor> initArmorProcessor();

    protected:
        ImageSize image_size_;
        ImageInfo image_info_;
        mutex image_mutex_;
        cv::Mat src_;
        
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
        
        // std::shared_ptr<ParamSubcriber> cb_;
        // std::shared_ptr<ParamCbHandle> param_cb_;
    protected:
        // 共享图像数据内存
        bool using_shared_memory_;
        SharedMemoryParam shared_memory_param_;
        std::thread read_memory_thread_; //共享内存读线程
        void imgCallbackThread();
    };
} //armor_processor

#endif