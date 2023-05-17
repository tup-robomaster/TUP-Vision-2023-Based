/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-02-25 18:52:43
 * @LastEditTime: 2023-04-14 02:47:53
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/daheng_driver/daheng_cam_node.cpp
 */
#include "../../include/daheng_driver/daheng_cam_node.hpp"

namespace camera_driver
{
    DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options)
    : CameraBaseNode<DaHengCam>("daheng_driver", options)
    {
        bool debug_;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {   
            RCLCPP_INFO(this->get_logger(), "Camera debug...");
            
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DahengCamNode::paramsCallback, this, _1));
        }
    }

    DahengCamNode::~DahengCamNode()
    {
        
    }

    /**
     * @brief 相机参数修改
     * 
     * @param param 参数量
     * @return true 
     * @return false 
     */
    bool DahengCamNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->cam_driver_->setExposureTime(param.as_int());
            RCLCPP_INFO(this->get_logger(), "Set daheng camera exposure time: %ldus", param.as_int());
            break;
        case 1:
            this->cam_driver_->setGain(3, param.as_int());
            RCLCPP_INFO(this->get_logger(), "Set daheng camera exposure gain: %ld", param.as_int());
            break;
        case 2:
            this->cam_driver_->setBalance(0, param.as_double());
            RCLCPP_INFO(this->get_logger(), "Set daheng camera balance B channel: %lf", param.as_double());
            break;
        case 3:
            this->cam_driver_->setBalance(1, param.as_double());
            RCLCPP_INFO(this->get_logger(), "Set daheng camera balance G channel: %lf", param.as_double());
            break;
        case 4:
            this->cam_driver_->setBalance(2, param.as_double());
            RCLCPP_INFO(this->get_logger(), "Set daheng camera balance R channel: %lf", param.as_double());
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "No relative param to set...");
            break;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult DahengCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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
} //camera_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver::DahengCamNode>());
    rclcpp::shutdown();
    
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::DahengCamNode)
