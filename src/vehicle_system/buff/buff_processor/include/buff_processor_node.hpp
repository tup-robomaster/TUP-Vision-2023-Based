/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:10:59
 * @LastEditTime: 2022-12-22 23:32:19
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/include/buff_processor_node.hpp
 */
#ifndef BUFF_PROCESSOR_NODE_HPP_
#define BUFF_PROCESSOR_NODE_HPP_

#include "./buff_processor/buff_processor.hpp"
#include "global_interface/include/global_interface/msg/gimbal.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

namespace buff_processor
{
    class BuffProcessorNode : public rclcpp::Node
    {
        typedef global_interface::msg::Buff BuffMsg;
        typedef global_interface::msg::Gimbal GimbalMsg;

    public:
        explicit BuffProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~BuffProcessorNode();
    
    private:
        // message_filters::Subscriber<BuffMsg> target_info_sub;
        rclcpp::Subscription<BuffMsg>::SharedPtr target_info_sub_;
        // message_filters::Subscriber<BuffMsg> target_point_sub_; 
        void target_info_callback(const BuffMsg& target_info);

        // 云台偏转角度（pitch、yaw）
        rclcpp::Publisher<GimbalMsg>::SharedPtr gimbal_info_pub_;

        // 预测点位置发布
        rclcpp::Publisher<BuffMsg>::SharedPtr predict_info_pub_;
    
    private:
        std::unique_ptr<Processor> buff_processor_;
        std::unique_ptr<Processor> init_buff_processor();

    public:
        PredictorParam predict_param_;
        PathParam path_param_;
        DebugParam debug_param_;
        std::string filter_param_path_;
    
    private:
        // params callback.
        std::map<std::string, int> param_map_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };
} // namespace buff_processor

#endif