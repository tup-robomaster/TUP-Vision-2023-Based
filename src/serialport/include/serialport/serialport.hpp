/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-15 22:01:49
 * @LastEditTime: 2023-03-14 21:29:21
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport/serialport.hpp
 */
#ifndef SERIALPORT_HPP_
#define SERIALPORT_HPP_

//linux
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>

//c++
#include <iostream>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>

#include "./crc_check.hpp"
#include "../../../global_user/include/global_user/global_user.hpp"

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL) //C_lflag
const std::vector<std::string> DEFAULT_PORT = {"ttyUSB", "ttyACM"}; //默认串口名
constexpr int MAX_ITER = 20; //默认串口最大编号

using namespace std;
using namespace global_user;
namespace serialport
{
    typedef struct
    {
        string id;
        string alias;
        string path;
    } Device;

    struct SerialData
    {
        bool is_initialized;
        Device device;
        int fd;       // 当前串口号
        int last_fd;  // 上一次串口号
        int speed;
        int baud;
        int databits;
        int stopbits;
        int parity;
        unsigned char rdata[64]; // raw_data
    };

    /**
     *@class  SerialPort
     *@brief  Set serialport, recieve and send data.
     *@param  int fd
    */
    class SerialPort
    {
    public:
        SerialPort();
        SerialPort(const string ID, const int BUAD, bool debug_without_com);
        ~SerialPort();

        //收发数据
        bool receiveData(int lens = 64);
        void sendData(int bytes_num = 64);

        //开闭串口
        bool openPort();
        void closePort();
        
        uchar Tdata[64] = {0x00}; 
        SerialData serial_data_;
    private:
        string serial_id_;
        CrcCheck crc_check_;
        
        bool setBit();
        void setBrate();
        std::vector<Device> listPorts();
        Device getDeviceInfo(string path);
        Device setDeviceByID(std::vector<Device> devices);

        bool withoutSerialPort();
        
    private:
        rclcpp::Logger logger_;
        rclcpp::Time timestamp_;

    public:
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        bool using_port_; //是否无串口调试
    };
} // namespace serialport

#endif // SERIALPORT_HPP_
