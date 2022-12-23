/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 16:19:57
 * @LastEditTime: 2022-12-23 11:57:00
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport/packet.hpp
 */
#ifndef PACKET_H
#define PACKET_H
 
#pragma once

#include "crc_check.hpp"

namespace serialport
{
    #define CmdID0 0x00; //关闭视觉
    #define CmdID1 0x01; //自瞄
    #define CmdID2 0x02; //小符
    #define CmdID3 0x03; //大符

    // typedef union
    // {
    //     float f;
    //     unsigned char c[4];
    // } float2uchar;
    
    typedef struct 
    {
        uint8_t header = 0xA5;

        bool isSwitched : 1; //1,位域长度
        bool isFindTarget : 1;
        bool isSpinning : 1;
        bool isMiddle : 1;
        
        float pitch_angle;
        float yaw_angle;
        float dis;

        uint16_t check_sum = 0;
    } SendPacket;

    typedef struct
    {
        std::string id;
        std::string path;
        std::string alias;
    } Device;

    typedef struct
    {
        uint8_t header = 0xA5;
        uint8_t mode : 2;
        float joint_pitch;
        float joint_yaw;

        float quat[4];
        float acc[3];
        float gyro[3];
        float bullet_speed = 28.0;

        uint16_t check_sum = 0;
    } ReceivePacket;

    inline ReceivePacket vec_to_packet(const std::vector<uint8_t>& data)
    {
        ReceivePacket receive_packet;
        std::copy(data.begin(), data.end(),
            reinterpret_cast<uint8_t*>(&(receive_packet)));
        return receive_packet;
    }

    inline std::vector<uint8_t> packet_to_vec(const SendPacket& send_packet)
    {
        std::vector<uint8_t> data(sizeof(SendPacket));
        std::copy(reinterpret_cast<const uint8_t*>(&send_packet),
            reinterpret_cast<const uint8_t*>(&send_packet) + sizeof(SendPacket),
            data.begin());
        return data;
    }
} //serialport

#endif