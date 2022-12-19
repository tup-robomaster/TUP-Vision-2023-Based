/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:11:19
 * @LastEditTime: 2022-12-19 23:22:26
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/buff_processor_node.cpp
 */
#include "../include/buff_processor_node.hpp"

namespace buff_processor
{
    BuffProcessorNode::BuffProcessorNode(const rclcpp::NodeOptions& options)
    : Node("buff_processor", options)
    {
        
    }

    BuffProcessorNode::~BuffProcessorNode()
    {
        
    }
    
} //namespace buff_processor