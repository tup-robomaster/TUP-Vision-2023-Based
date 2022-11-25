/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:15:03
 * @LastEditTime: 2022-11-24 21:52:45
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport/serialport_node.hpp
 */
#ifndef SERIALPORT_NODE_HPP
#define SERIALPORT_NODE_HPP
#pragma once 

//ros
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "./serialport_old.hpp"
#include "./serialport.hpp"

#include "../../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/imu.hpp"
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/target.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace serialport
{
    class serial_driver : public rclcpp::Node
    {
    public:
        serial_driver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~serial_driver();
        // rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        // {
        //     return node_->get_node_base_interface();
        // }

    public:
        void run();
        void receive_data();
        void send_data(global_interface::msg::Gimbal::SharedPtr msg);
    
    private:
        bool debug_without_port;
        // rclcpp::Node::SharedPtr node_;
        // std::thread thread_watch_;
        std::thread receive_thread_;
        std::string device_name_;
        int baud_;
        void* buffer_;
        int len_;

        Eigen::Quaterniond quat_;
        Eigen::Matrix3d rmat_imu_;
        // global_interface::msg::Target target_info_;
        coordsolver::coordsolver coordsolver_;

        //tf2
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        message_filters::Subscriber<global_interface::msg::Gimbal> gimbal_info_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<global_interface::msg::Gimbal> tf2_filter_;

    public:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<global_interface::msg::Gimbal>::SharedPtr gimbal_motion_sub_;
    
    private:
        std::unique_ptr<serialport> serial_port_;
        std::unique_ptr<serialport> init_serial_port();

        std::unique_ptr<SerialPort> serial_port_old_;
        std::unique_ptr<SerialPort> init_serial_port_old();

    public:
        //tf2
        rclcpp::Subscription<global_interface::msg::Imu>::SharedPtr imu_data_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string base_frame_;
        std::string imu_frame_;
        std::string camera_frame_;

        void handle_imu_data(const std::shared_ptr<global_interface::msg::Imu>& msg);

        void send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg) const;

        void send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg,
            const std::string& header_frame_id, const std::string& child_frame_id) const;

        void send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg,
            const std::string& header_frame_id,
            const std::string& child_frame_id,
            const rclcpp::Time& time) const;

        std::vector<double> static_transform_;

    }; //serial_driver
} //serialport

#endif
