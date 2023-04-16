/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:03:45
 * @LastEditTime: 2023-02-26 12:35:58
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/hik_driver/hik_cam_node.hpp
 */

#ifndef HIK_CAM_NODE_HPP_
#define HIK_CAM_NODE_HPP_

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include "../../global_user/include/global_user/global_user.hpp"
#include "../camera_driver/camera_driver_node.hpp"

using namespace std; 
using namespace global_user;
namespace camera_driver
{
    class HikCamNode : public CameraBaseNode<HikCamera>
    {
    public:
        HikCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~HikCamNode();
    
    private:
        // Update params.
        bool setParam(rclcpp::Parameter);

        // Params callback.
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    };
} // namespace camera_driver

#endif // HIK_CAM_NODE_HPP_
