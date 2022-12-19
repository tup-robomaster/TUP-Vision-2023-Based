/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:08:00
 * @LastEditTime: 2022-12-19 23:10:18
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/src/buff_detector_node.cpp
 */
#include "../include/buff_detector_node.hpp"

namespace buff_detector
{
    BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("buff_detector", options)
    {
        RCLCPP_INFO("buff detector node...");

    }

    BuffDetectorNode::~BuffDetectorNode()
    {
        
    }
    
}