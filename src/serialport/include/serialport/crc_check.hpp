/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-15 22:01:49
 * @LastEditTime: 2022-12-23 12:16:54
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/serialport/crc_check.hpp
 */
#ifndef _CRC_CHECK_HPP_
#define _CRC_CHECK_HPP_

#include <iostream>
#include <stdint.h>

namespace serialport
{
    class CrcCheck
    {
    public:
        CrcCheck();
        ~CrcCheck();

        unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
        unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
        void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
        uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
        uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
        void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
    };
} //namespace serialport

#endif
