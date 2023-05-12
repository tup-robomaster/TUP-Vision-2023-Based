/*
 * @Description: This is a ros-based project!
 * @Author: Leo
 * @Date: 2023-04-29 12:02:22
 * @LastEditTime: 2023-04-29 12:02:22
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/hik_driver/hik_cam_node.hpp
 */

#ifndef MVS_CAM_NODE_HPP_
#define MVS_CAM_NODE_HPP_

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
    class MvsCamNode : public CameraBaseNode<MvsCamera>
    {
        public:
            MvsCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
            ~MvsCamNode();
    
        private:
            // Update params.
            bool setParam(rclcpp::Parameter);

            // Params callback.
            rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    };
}

#endif