/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-02-07 01:45:19
 * @LastEditTime: 2023-02-23 18:39:12
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/include/data_processor/data_transform.hpp
 */
#ifndef DATA_TRANSFORM_HPP_
#define DATA_TRANSFORM_HPP_

#include <string.h>

//c++
#include <iostream>
#include <vector>
#include <string>

//opencv
#include <opencv2/opencv.hpp>

#include "../serialport/crc_check.hpp"

using namespace std;
namespace serialport
{
    /**
     * @brief 模式选择（取消视觉，自瞄，英雄吊射，小符，大符，哨兵）
     * 
     */
    enum MODE
    {
        CLOSE_VISION,
        AUTOAIM,
        HERO_SLING,
        SMALL_BUFF,
        BIG_BUFF,
        SENTRY_RECV_NORMAL,
        OUTPOST_ROTATION_MODE
    };
    
    // 字节数为4的结构体
    // typedef union
    // {
    //     float f;
    //     unsigned char c[4];
    // } float2uchar;

    // typedef union
    // {
    //     int16_t d;
    //     unsigned char c[2];
    // } int16uchar;

    //TODO:待改
    typedef struct VisionData
    {
        double timestamp;
        float pitch_angle; //俯仰角
        float yaw_angle;   //偏航角
        float distance;  //目标距离
        int isSwitched;   //目标是否发生切换
        int isFindTarget; //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
        int isSpinning;   //目标是否处于陀螺状态
        int ismiddle;     //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
    } VisionData;
    
    class DataTransform
    {
    public:
        DataTransform();
        ~DataTransform();

        void transformData(int mode, const VisionData &data, uchar* trans_data); 
        void getQuatData(uchar* raw_data, vector<float>& quat);
        void getGyroData(uchar* raw_data, vector<float>& gyro);
        void getAccData(uchar* raw_data, vector<float>& acc);
        void getBulletSpeed(uchar* raw_data, float& bullet_speed);
        void getThetaAngle(uchar* raw_data, float& theta);

        int mode_;       
    private:
        CrcCheck crc_check_;
        float ucharRaw2Float(uchar *data); // 将4个uchar合并成一个float
        bool ucharRaw2FloatVector(uchar *data, int bytes, std::vector<float> &vec);
        uchar* float2UcharRaw(float float_data);
        bool float2UcharRawArray(float float_data[], int num, uchar* raw_data);
    };
} //namespace serialport

#endif
