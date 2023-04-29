/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 14:30:38
 * @LastEditTime: 2023-04-14 02:38:20
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/hik_driver/hik_cam_node.cpp
 */
#include "../../include/hik_driver/hik_cam_node.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace camera_driver
{
    HikCamNode::HikCamNode(const rclcpp::NodeOptions &options)
    : CameraBaseNode("hik_driver", options)
    {
        bool debug_;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&HikCamNode::paramsCallback, this, _1));
            RCLCPP_INFO(this->get_logger(), "Hik camera debug...");
        }
    }

    HikCamNode::~HikCamNode()
    {
    }

    bool HikCamNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            cam_driver_->setExposureTime(param.as_double());
            break;
        case 1:
            cam_driver_->setGain(3, param.as_int());
            break;
        case 2:
            cam_driver_->setBalance(0, param.as_int());
            break;
        case 3:
            cam_driver_->setBalance(1, param.as_int());
            break;
        case 4:
            cam_driver_->setBalance(2, param.as_int());
            break;
        default:
            break;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult HikCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            result.successful = setParam(param);
        }
        return result;
    }
} //namespace camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::HikCamNode)

