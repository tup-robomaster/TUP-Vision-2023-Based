/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:10:59
 * @LastEditTime: 2023-06-06 21:21:15
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/include/buff_processor_node.hpp
 */
#ifndef BUFF_PROCESSOR_NODE_HPP_
#define BUFF_PROCESSOR_NODE_HPP_

#include "./buff_processor/buff_processor.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
//c++
#include <mutex>
#include <thread>
#include <atomic>

#include <matplotlibcpp.h>

//opencv
#include <opencv2/opencv.hpp>

#include "global_interface/msg/buff.hpp"
#include "global_interface/msg/gimbal.hpp"

namespace plt = matplotlibcpp;

using namespace global_user;
using namespace coordsolver;
using namespace ament_index_cpp;
using namespace plt;
namespace buff_processor
{
    class BuffProcessorNode : public rclcpp::Node
    {
        typedef global_interface::msg::Buff BuffMsg;
        typedef global_interface::msg::Gimbal GimbalMsg;
        typedef sensor_msgs::msg::Image ImageMsg;

    public:
        explicit BuffProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~BuffProcessorNode();
    
    private:
        rclcpp::Subscription<BuffMsg>::SharedPtr buff_msg_sub_;
        void predictorCallback(const BuffMsg& buff_msg);

        // 云台偏转角度（pitch、yaw）
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_msg_pub_;

        // 预测点位置发布
        rclcpp::Publisher<BuffMsg>::SharedPtr predict_msg_pub_;
    
    private:
        std::unique_ptr<Processor> buff_processor_;
        std::unique_ptr<Processor> initBuffProcessor();
    
    private:
        Mutex image_mutex_;
        std::shared_ptr<image_transport::Subscriber> img_msg_sub_;
        rclcpp::TimerBase::SharedPtr draw_curve_callback_timer_;
        cv::Mat src_;
        rclcpp::Time start_time_;
        Mutex plot_mutex_;
        
        Eigen::Vector3d last_pred_point3d_cam_ = {0.0, 0.0, 0.0};
        Eigen::Vector3d last_pred_point3d_world_ = {0.0, 0.0, 0.0};

        Eigen::Vector3d last_meas_point3d_cam_ = {0.0, 0.0, 0.0};
        Eigen::Vector3d last_meas_point3d_world_ = {0.0, 0.0, 0.0};
        double last_pred_angle_ = 0.0;
        double last_meas_angle_ = 0.0;
        Eigen::Vector2d last_gimabl_angle_ = {0.0, 0.0};
        
        void imageCallback(const ImageMsg::ConstSharedPtr &img_msg);
        void drawCurve();

    public:
        Mutex param_mutex_;
        PredictorParam predict_param_;
        PathParam path_param_;
        DebugParam debug_param_;
        std::string filter_param_path_;
    
    private:
        // params callback.
        bool updateParam();
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };
} // namespace buff_processor

#endif