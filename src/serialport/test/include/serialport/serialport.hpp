/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:59:06
 * @LastEditTime: 2022-11-23 10:06:37
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport/serialport.hpp
 */
#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#pragma once 

#include "packet.hpp"
#include "crc_check.hpp"

// #include "global_user/global_user.hpp"
#include <rclcpp/rclcpp.hpp>
// #include "rmoss_master/rmoss_core/rmoss_base/include/rmoss_base/transporter_interface.hpp"

namespace serialport 
{

    #define MAX_ITER 5
    const std::vector<std::string> DEFAULT_PORT = {"ttyUSB", "ttyACM"};
    
    class serialport
    {
    public:
        explicit serialport();
        explicit serialport(const std::string id, const int baud, bool debug_without_com);
        ~serialport();

        void init();
        bool open(const std::string id, const int baud);
        void close();
        void read(std::vector<uint8_t>& buffer);
        void write(std::vector<uint8_t>& buffer);
        std::string error_message();

        std::vector<Device> list_ports();
        Device get_device_info(std::string path);
        Device select_id(std::vector<Device> devices);
        Device getDeviceInfo(std::string path);
        
        bool debug_without_port();
        bool set_brate();
        int set_bit();

        bool get_quat(std::vector<uint8_t>& data);
        bool get_gyro(std::vector<uint8_t>& data);
        bool get_acc(std::vector<uint8_t>& data);

        bool debug_without_com_;
        bool is_open;
    private:    

        rclcpp::Node::SharedPtr node;
        Device device;
        std::string serial_id;
        
        int fd;
        int last_fd;
        int speed;
        int device_baud;
        int databits;
        int stopbits;
        int parity;
        int bytes;
        int result;
        char *name;

    public:    
        SendPacket send_packet;
        ReceivePacket receive_packet;
    };
} //serialport

#endif //SERIALPORT_HPP