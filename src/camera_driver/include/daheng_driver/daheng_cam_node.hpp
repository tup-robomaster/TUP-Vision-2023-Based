/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-02-25 18:41:51
 * @LastEditTime: 2023-02-26 12:35:01
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/daheng_driver/daheng_cam_node.hpp
 */
#ifndef DAHENG_CAM_NODE_HPP_
#define DAHENG_CAM_NODE_HPP_

//ros
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>

#include "../../global_user/include/global_user/global_user.hpp"
#include "../camera_driver/camera_driver_node.hpp"

using namespace std;
namespace camera_driver
{
    class DahengCamNode : public CameraBaseNode<DaHengCam>
    {
    public:
        DahengCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~DahengCamNode();

    private:
        // Update params.
        bool setParam(rclcpp::Parameter);

        // Params callback.
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    };
} //namespace camera_driver

#endif