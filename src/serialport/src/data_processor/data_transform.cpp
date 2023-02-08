/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-02-07 02:02:10
 * @LastEditTime: 2023-02-08 15:37:38
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/data_processor/data_transform.cpp
 */
#include "../../include/data_processor/data_transform.hpp"

namespace serialport
{
    DataTransform::DataTransform()
    {
    }

    DataTransform::~DataTransform()
    {
    }

    /**
    *@brief 转换数据并发送
    *@param data 类型 VisionData(union)  包含pitch,yaw,distance
    *@param flag 类型 char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
    */
    void DataTransform::transformData(int mode, const VisionData &vision_data, uchar* trans_data)
    {
        if(mode == AUTOAIM || mode == SMALL_BUFF || mode == BIG_BUFF)
        {
            trans_data[0] = 0xA5;
            trans_data[1] = mode;
            crc_check_.Append_CRC8_Check_Sum(trans_data, 3);

            float float_data[] = {vision_data.pitch_angle, vision_data.yaw_angle, vision_data.distance};
            float2UcharRawArray(float_data, 3, &trans_data[3]);

            trans_data[15] = vision_data.isSwitched;
            trans_data[16] = vision_data.isFindTarget;
            trans_data[17] = vision_data.isSpinning;
            trans_data[18] = vision_data.ismiddle;
            
            trans_data[19] = 0x00;
            crc_check_.Append_CRC16_Check_Sum(trans_data, 22);
        }
        else if(mode == SENTRY_RECV_NORMAL)
        {
            
        }
    }

    void DataTransform::getQuatData(uchar* raw_data, vector<float>& quat)
    {
        ucharRaw2FloatVector(raw_data, 16, quat);
    }

    void DataTransform::getGyroData(uchar* raw_data, vector<float>& gyro)
    {
        ucharRaw2FloatVector(raw_data, 12, gyro);
    }

    void DataTransform::getAccData(uchar* raw_data, vector<float>& acc)
    {
        ucharRaw2FloatVector(raw_data, 12, acc);
    }

    void DataTransform::getBulletSpeed(uchar* raw_data, float& bullet_speed)
    {
        bullet_speed = ucharRaw2Float(raw_data);
    }

    void DataTransform::getThetaAngle(uchar* raw_data, float& theta)
    {
        theta = ucharRaw2Float(raw_data);
    }

    /**
     * @brief 将4个uchar转换为float
     * @param data data首地址指针
     * @return
     */
    float DataTransform::ucharRaw2Float(uchar *data)
    {
        float float_data;
        float_data = *((float*)data);
        return float_data;
    };

    /**
     * @brief float转uchar
     * 
     * @param float_data float型数据
     * @return uchar* 返回uchar指针
     */
    uchar* DataTransform::float2UcharRaw(float float_data)
    {
        uchar* raw_data = nullptr;
        raw_data = (uchar*)(&float_data); 
        return std::move(raw_data);
    }   

    /**
     * @brief uchar原始数据转换为float vector
     * @param data 首地址指针
     * @param bytes 字节数
     * @param vec float vector地址
     */
    bool DataTransform::ucharRaw2FloatVector(uchar *data, int bytes, std::vector<float> &vec)
    {
        std::vector<uchar*> pts;
        assert(bytes % 4 == 0);
        for (int i = 0; i < bytes; i+=4)
        {
            vec.push_back(ucharRaw2Float(&data[i]));
        }
        return true;
    }
    
    /**
     * @brief float转uchar数组
     * 
     * @param float_data float型数据
     * @param num float型数组长度
     * @param raw_data uchar指针
     * @return true 
     * @return false 
     */
    bool DataTransform::float2UcharRawArray(float float_data[], int num, uchar* raw_data)
    {
        // memcpy(raw_data, float_data, sizeof(float) * num);
        for(int ii = 0; ii < num; ++ii)
        {
            uchar* data = float2UcharRaw(float_data[ii]);
            for(int jj = 0; jj < 4; ++jj)
            {
                raw_data[ii * 4 + jj] = data[jj];
            }
        }
        return true;
    }
} //namespace serialport