/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 16:02:36
 * @LastEditTime: 2022-09-27 20:58:51
 * @FilePath: /tup_2023/src/serialport/src/serialport/crc_check.cpp
 */
#include "../../include/serialport/crc_check.hpp"

namespace serialport
{
    unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
    {
        unsigned char ucIndex;

        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*pchMessage++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }

        return (ucCRC8);
    }

    unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
    {
        unsigned char ucExpected = 0;

        if ((pchMessage == 0) || (dwLength <= 2)) return 0;

        ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
        return (ucExpected == pchMessage[dwLength - 1]);
    }

    void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
    {
        unsigned char ucCRC = 0;

        if ((pchMessage == 0) || (dwLength <= 2)) return;

        ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
        pchMessage[dwLength - 1] = ucCRC;
    }

    uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
    {
        uint8_t chData;

        if (pchMessage == NULL)
        {
            return 0xFFFF;
        }

        while (dwLength--)
        {
            chData = *pchMessage++;
            (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
        }

	    return wCRC;
    }

    uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
    {
        uint16_t wExpected = 0;

        if ((pchMessage == NULL) || (dwLength <= 2))
        {
            return 0;
        }

        wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
        return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
    }

    void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
    {
        uint16_t wCRC = 0;

        if ((pchMessage == NULL) || (dwLength <= 2))
        {
            return;
        }

        wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC16_INIT);
        pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
        pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
    }
}; //serialport
