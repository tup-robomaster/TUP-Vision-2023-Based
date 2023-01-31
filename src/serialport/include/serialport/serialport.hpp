/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-15 22:01:49
 * @LastEditTime: 2023-01-31 23:57:45
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
constexpr int MAX_ITER = 3; //默认串口最大编号
#define TRUE 1
#define FALSE 0

// 模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //自瞄
#define CmdID2 0x02; //英雄吊射
#define CmdID3 0x03; //小符
#define CmdID4 0x04; //大符

using namespace std;
using namespace global_user;
namespace serialport
{
    //字节数为4的结构体
    typedef union
    {
        float f;
        unsigned char c[4];
    } float2uchar;

    typedef struct
    {
        string id;
        string alias;
        string path;
    } Device;

    // 字节数为2的uchar数据类型
    // typedef union
    // {
    //     int16_t d;
    //     unsigned char c[2];
    // } int16uchar;

    //用于保存目标相关角度和距离信息及瞄准情况
    typedef struct
    {
        float2uchar pitch_angle; //偏航角
        float2uchar yaw_angle;   //俯仰角
        // float2uchar yaw_angle;//偏航角
        // float2uchar pitch_angle;//俯仰角
        float2uchar dis;  //目标距离
        int isSwitched;   //目标是否发生切换
        int isFindTarget; //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
        int isSpinning;   //目标是否处于陀螺状态
        int ismiddle;     //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
    } VisionData;

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
        unsigned char rdata[255]; // raw_data
    };

    /**
     *@class  SerialPort
     *@brief  Set serialport, recieve and send.
     *@param  int fd
    */
    class SerialPort
    {
    public:
        SerialPort();
        SerialPort(const string ID, const int BUAD, bool debug_without_com, bool is_sentry_mode);
        ~SerialPort();
    
        SerialData serial_data_;              
        bool get_Mode(int bytes);
        bool initSerialPort();
        void transformData(const VisionData &data); //主要方案
        void send();
    private:
        bool withoutSerialPort();
        Device getDeviceInfo(string path);
        Device setDeviceByID(std::vector<Device> devices);
        std::vector<Device> listPorts();
        void set_Brate();
        int set_Bit();
        void closePort();
    private:
        bool is_sentry_mode_;
        CrcCheck crc_check_;
        string serial_id_;
        rclcpp::Time timestamp_;

        unsigned char Tdata[30];                  // transfrom data
        float exchange_data(unsigned char *data); // 将4个uchar合并成一个float
        bool getQuat(unsigned char *data);
        bool getGyro(unsigned char *data);
        bool getAcc(unsigned char *data);
        bool getSpeed(unsigned char *data);
    
        rclcpp::Logger logger_;
    public:
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        bool debug_without_com_; //是否无串口调试
        bool print_imu_data_; 
        int mode;       
        float bullet_speed_;
        float quat[4]; //四元数
        float acc[3];  //加速度
        float gyro[3]; //角速度
    };
} // namespace serialport

#endif // SERIALPORT_HPP_
