/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:56:35
 * @LastEditTime: 2023-05-17 15:12:06
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
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        void targetMsgCallback(const AutoaimMsg& autoaim_msg);
        rclcpp::Subscription<AutoaimMsg>::SharedPtr autoaim_msg_sub_;

        cv::Mat src_;
        mutex debug_mutex_;
        mutex image_mutex_;
        bool is_aimed_ = false;
        bool is_pred_ = false;
        map<int, string> state_map_;
        atomic<bool> image_flag_ = false;
        
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_info_pub_;
        rclcpp::Publisher<GimbalMsg>::SharedPtr tracking_info_pub_;
        rclcpp::Publisher<AutoaimMsg>::SharedPtr predict_info_pub_;
        
        // visualization_msgs::Marker
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
        uint64 shape_ = visualization_msgs::msg::Marker::SPHERE;
        bool show_marker_ = false;
        int count_ = 0;
        bool shoot_flag_ = false;
        void pubMarkerArray(vector<Eigen::Vector4d> armor3d_vec, bool is_clockwise, int flag);

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