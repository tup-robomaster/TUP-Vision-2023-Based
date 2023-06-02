/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:15:03
 * @LastEditTime: 2023-05-31 17:46:44
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport_node.hpp
 */
#ifndef SERIALPORT_NODE_HPP_
#define SERIALPORT_NODE_HPP_

#pragma once 

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "./serialport/serialport.hpp"
#include "./data_processor/data_transform.hpp"

#include "global_interface/msg/serial.hpp"
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/sentry.hpp"
#include "global_interface/msg/car_pos.hpp"
#include "global_interface/msg/obj_hp.hpp"
#include "global_interface/msg/game_info.hpp"
#include "global_interface/msg/decision.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/decision.hpp"

using namespace global_user;
using namespace coordsolver;
namespace serialport
{
    class SerialPortNode : public rclcpp::Node
    {
        typedef global_interface::msg::Gimbal GimbalMsg;
        typedef global_interface::msg::Serial SerialMsg;

    public:
        SerialPortNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SerialPortNode();

    public:
        void receiveData();
        bool sendData(GimbalMsg::SharedPtr msg);
        void armorMsgCallback(GimbalMsg::SharedPtr msg);
        void buffMsgCallback(GimbalMsg::SharedPtr msg);
        
        void serialWatcher();
        rclcpp::Publisher<SerialMsg>::SharedPtr serial_msg_pub_;

    private:
        int baud_;
        std::string id_;
        std::string device_name_;
        CoordSolver coordsolver_;
        bool print_serial_info_;
        bool print_referee_info_;
        std::unique_ptr<std::thread> receive_thread_;
        
        mutex mutex_;
        bool using_port_;
        bool tracking_target_;
        atomic<int> mode_;
        atomic<bool> flag_;
        rclcpp::TimerBase::SharedPtr watch_timer_;
        rclcpp::TimerBase::SharedPtr send_timer_;
        
        //tf2
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        
    public:
        rclcpp::Subscription<GimbalMsg>::SharedPtr autoaim_info_sub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr autoaim_tracking_sub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr buff_info_sub_;

    private:
        std::unique_ptr<SerialPort> serial_port_;
        std::unique_ptr<SerialPort> initSerialPort();

        std::unique_ptr<DataTransform> data_transform_;
        std::unique_ptr<DataTransform> initDataTransform();
    
    private:
        std::map<std::string, int> params_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    }; 
} //namespace serialport

#endif
