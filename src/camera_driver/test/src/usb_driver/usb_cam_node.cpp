/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:53
 * @LastEditTime: 2023-02-26 13:50:56
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam_node.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace camera_driver
{
    UsbCamNode::UsbCamNode(const rclcpp::NodeOptions& option)
    : CameraBaseNode("usb_driver", option), is_filpped(false)
    {
        bool debug_;
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            RCLCPP_INFO(this->get_logger(), "Usb camera debug...");

            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&UsbCamNode::paramsCallback, this, _1));
        }
    }

    UsbCamNode::~UsbCamNode()
    {
    }

    bool UsbCamNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            cam_driver_->usb_cam_params_.image_width = param.as_int();
            break;
        case 1:
            cam_driver_->usb_cam_params_.image_height = param.as_int();
            break;
        case 2:
            cam_driver_->usb_cam_params_.fps = param.as_int();
            break;
        default:
            break;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult UsbCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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

// int main(int argc, char** argv)
// {
//     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//     rclcpp::init(argc, argv);
//     rclcpp::executors::SingleThreadedExecutor exec;
//     const rclcpp::NodeOptions options;
//     auto usb_cam_node = std::make_shared<camera_driver::UsbCamNode>(options);
//     exec.add_node(usb_cam_node);
//     exec.spin();
//     rclcpp::shutdown();

//     return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::UsbCamNode)