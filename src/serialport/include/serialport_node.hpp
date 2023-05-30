/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:15:03
 * @LastEditTime: 2023-05-14 16:04:52
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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "./serialport/serialport.hpp"
#include "./data_processor/data_transform.hpp"

#include "global_interface/msg/serial.hpp"
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/sentry.hpp"
#include "global_interface/msg/car_pos.hpp"
#include "global_interface/msg/obj_hp.hpp"
#include "global_interface/msg/game_info.hpp"
#include "global_interface/msg/decision.hpp"
#include "global_interface/msg/mode_set.hpp"
#include "../../global_user/include/coordsolver.hpp"

using namespace global_user;
using namespace coordsolver;
namespace serialport
{
    class SerialPortNode : public rclcpp::Node
    {
        typedef global_interface::msg::Gimbal GimbalMsg;
        typedef global_interface::msg::Serial SerialMsg;
        typedef global_interface::msg::Sentry SentryMsg;
        typedef global_interface::msg::ObjHP ObjHPMsg;
        typedef global_interface::msg::CarPos CarPosMsg;
        typedef global_interface::msg::GameInfo GameMsg;
        typedef global_interface::msg::Decision DecisionMsg;
        typedef global_interface::msg::ModeSet ModeSetMsg;

    public:
        SerialPortNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SerialPortNode();

    public:
        void receiveData();
        // void sendingData();
        bool sendData(GimbalMsg::SharedPtr msg);
        void armorMsgCallback(GimbalMsg::SharedPtr msg);
        void buffMsgCallback(GimbalMsg::SharedPtr msg);
        void sentryNavCallback(geometry_msgs::msg::Twist::SharedPtr msg);
        void serialWatcher();

        void decisionMsgCallback(DecisionMsg::SharedPtr msg);
        DecisionMsg decision_msg_;
        mutex decision_mutex_;
    private:
        int baud_;
        std::string id_;
        std::string device_name_;
        CoordSolver coordsolver_;
        bool print_serial_info_;
        bool print_referee_info_;
        std::unique_ptr<std::thread> receive_thread_;
        
        mutex mutex_;
        bool debug_without_decision_msg_;
        bool using_port_;
        bool tracking_target_;
        atomic<int> mode_;
        atomic<bool> flag_;
        
        mutex tf_lock_;
        tf2::Transform virtual_heading_;
        geometry_msgs::msg::TwistStamped last_twist_;
        // VisionData vision_data_;
        rclcpp::TimerBase::SharedPtr watch_timer_;
        rclcpp::TimerBase::SharedPtr send_timer_;
        // rclcpp::TimerBase::SharedPtr receive_timer_;
        queue<VisionAimData> vision_data_queue_;

        rclcpp::Time stamp_;

        //tf2    
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
    public:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sentry_twist_sub_;
        rclcpp::Publisher<CarPosMsg>::SharedPtr car_pos_pub_;
        rclcpp::Publisher<ObjHPMsg>::SharedPtr obj_hp_pub_;
        rclcpp::Publisher<GameMsg>::SharedPtr game_msg_pub_;
        rclcpp::Publisher<ModeSetMsg>::SharedPtr mode_set_msg_pub_;

        rclcpp::Subscription<GimbalMsg>::SharedPtr autoaim_info_sub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr autoaim_tracking_sub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr buff_info_sub_;
        rclcpp::Subscription<DecisionMsg>::SharedPtr decision_msg_sub_;
        
        rclcpp::Publisher<SerialMsg>::SharedPtr serial_msg_pub_;
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
