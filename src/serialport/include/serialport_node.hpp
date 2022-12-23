/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:15:03
 * @LastEditTime: 2022-12-23 19:20:26
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport_node.hpp
 */
#ifndef SERIALPORT_NODE_HPP_
#define SERIALPORT_NODE_HPP_

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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "./serialport/serialport.hpp"

#include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/imu.hpp"
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/target.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace global_user;
using namespace coordsolver;

namespace serialport
{
    class SerialPortNode : public rclcpp::Node
    {
        typedef global_interface::msg::Gimbal GimbalMsg;
        typedef global_interface::msg::Target TargetMsg;
        typedef global_interface::msg::Imu ImuMsg;

    public:
        SerialPortNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SerialPortNode();
        // rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        // {
        //     return node_->get_node_base_interface();
        // }

    public:
        void run();
        void receive_data();
        void send_data(GimbalMsg::SharedPtr msg);
    
    private:
        // void* buffer_;
        // std::thread thread_watch_;
        // rclcpp::Node::SharedPtr node_;
        bool debug_without_port_;
        std::thread receive_thread_;
        std::string device_name_;
        int baud_;
        // int len_;

        // Eigen::Quaterniond quat_;
        // Eigen::Matrix3d rmat_imu_;
        // TargetMsg target_info_;
        CoordSolver coordsolver_;

        //tf2
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        message_filters::Subscriber<GimbalMsg> gimbal_info_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<GimbalMsg>> tf2_filter_;

    public:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr gimbal_motion_sub_;
        rclcpp::Publisher<ImuMsg>::SharedPtr imu_data_pub_;
    
    private:
        std::unique_ptr<SerialPort> serial_port_;
        std::unique_ptr<SerialPort> init_serial_port();

    public:
        // tf2
        rclcpp::Subscription<global_interface::msg::Imu>::SharedPtr imu_data_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string base_frame_;
        std::string imu_frame_;
        std::string camera_frame_;

        std::vector<double> static_transform_;
        void handle_imu_data(const std::shared_ptr<global_interface::msg::Imu>& msg);
        void send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg);
        void send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg,
            const std::string& header_frame_id, const std::string& child_frame_id);
        void send_tf2_transforms(const TargetMsg::SharedPtr msg,
            const std::string& header_frame_id,
            const std::string& child_frame_id,
            const rclcpp::Time& time);
    
    private:
        std::map<std::string, int> params_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    }; //SerialPortNode

} //serialport

#endif
