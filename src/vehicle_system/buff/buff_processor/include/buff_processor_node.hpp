/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:10:59
 * @LastEditTime: 2022-12-19 23:19:31
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/include/buff_processor_node.hpp
 */
#include "./buff_processor/buff_processor.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

namespace buff_processor
{
    class BuffProcessorNode : public rclcpp::Node
    {
    public:
        explicit BuffProcessorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~BuffProcessorNode();
    
    private:
        // message_filters::Subscriber<global_interface::msg::Target> target_info_sub;
        rclcpp::Subscription<global_interface::msg::Target>::SharedPtr target_info_sub_;
        message_filters::Subscriber<global_interface::msg::Target> target_point_sub_; 
        void target_info_callback(const global_interface::msg::Target& target_info);
    
    private:
        std::unique_ptr<Processor> buff_processor_;
        std::unique_ptr<Processor> init_buff_processor();

    public:
        PredictParam predict_param_;
        DebugParam debug_param_;
        std::string filter_param_path_;
        std::string coord_param_path_;
        std::string coord_param_name_;
    
    private:
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    };
}