/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:15:03
 * @LastEditTime: 2022-09-28 16:47:47
 * @FilePath: /tup_2023/src/serialport/include/serialport/serialport_node.hpp
 */
#ifndef SERIALPORT_NODE_HPP
#define SERIALPORT_NODE_HPP


#pragma once 

#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp/rclcpp.hpp"

#include "serialport.hpp"
// #include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "global_interface/msg/gimbal.hpp"

namespace serialport
{
    class serial_driver : public rclcpp::Node
    {
        rclcpp::Node::SharedPtr node;
        std::thread thread_watch;
        std::thread thread_read;
        std::string device_name;
        int baud;
        void* buffer;
        int len;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
        rclcpp::Subscription<global_interface::msg::Gimbal>::SharedPtr gimbal_motion_sub;
        
        std::unique_ptr<serialport> serial_port;
         
    public:
        explicit serial_driver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~serial_driver();
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node->get_node_base_interface();
        }

        void receive_data();
        void send_data(global_interface::msg::Gimbal::SharedPtr msg);

    }; //serial_driver
} //serialport

#endif
