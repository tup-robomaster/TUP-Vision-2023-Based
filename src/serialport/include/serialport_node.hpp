/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:15:03
 * @LastEditTime: 2023-03-02 17:47:39
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

#include "./serialport/serialport.hpp"
#include "./data_processor/data_transform.hpp"

#include "global_interface/msg/serial.hpp"
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/sentry.hpp"
#include "global_interface/msg/car_pos.hpp"
#include "global_interface/msg/car_hp.hpp"
#include "global_interface/msg/game_info.hpp"
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
        typedef global_interface::msg::CarHP CarHPMsg;
        typedef global_interface::msg::CarPos CarPosMsg;
        typedef global_interface::msg::GameInfo GameMsg;

    public:
        SerialPortNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SerialPortNode();

    public:
        void receiveData();
        void sendingData();
        bool sendData(GimbalMsg::SharedPtr msg);
        void armorMsgSub(GimbalMsg::SharedPtr msg);
        void buffMsgSub(GimbalMsg::SharedPtr msg);
        void sentryMsgSub(SentryMsg::SharedPtr msg);
        void serialWatcher();
    
    private:
        int baud_;
        std::string id_;
        std::string device_name_;
        std::thread receive_thread_;
        CoordSolver coordsolver_;
        bool print_serial_info_;
        bool print_referee_info_;
        
        mutex mutex_;
        bool using_port_;
        bool tracking_target_;
        bool is_sentry_;
        atomic<int> mode_;
        atomic<bool> flag_;
        // VisionData vision_data_;
        rclcpp::TimerBase::SharedPtr watch_timer_;
        rclcpp::TimerBase::SharedPtr send_timer_;
        queue<VisionData> vision_data_queue_;
        
    public:
        /**
         * @brief 哨兵和其他车辆的msg不同，此处订阅者和发布者视兵种而定
         * 
         */
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<SentryMsg>::SharedPtr sentry_msg_sub_;
        rclcpp::Publisher<CarPosMsg>::SharedPtr car_pos_pub_;
        rclcpp::Publisher<CarHPMsg>::SharedPtr car_hp_pub_;
        rclcpp::Publisher<GameMsg>::SharedPtr game_msg_pub_;

        // 其他兵种
        rclcpp::Subscription<GimbalMsg>::SharedPtr autoaim_info_sub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr autoaim_tracking_sub_;
        rclcpp::Subscription<GimbalMsg>::SharedPtr buff_info_sub_;

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
