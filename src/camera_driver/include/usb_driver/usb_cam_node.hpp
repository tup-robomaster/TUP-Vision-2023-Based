/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:40
 * @LastEditTime: 2023-02-26 13:52:58
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam_node.hpp
 */
#ifndef USB_CAM_NODE_HPP_
#define USB_CAM_NODE_HPP_

#include <memory>

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

//ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "../../global_user/include/global_user/global_user.hpp"
#include "../camera_driver/camera_driver_node.hpp"

using namespace global_user;
namespace camera_driver
{
    class UsbCamNode : public CameraBaseNode<UsbCam>
    {
    public:
        UsbCamNode(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        ~UsbCamNode();
    
    private:
        bool is_filpped;
        
        // Update params.
        bool setParam(rclcpp::Parameter);

        // Params callback.
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    }; // UsbCamNode
} //namespace camera_driver

#endif