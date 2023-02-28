/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-02-07 01:45:19
 * @LastEditTime: 2023-02-25 12:15:49
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

//ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

//eigen
#include <Eigen/Core>

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

    typedef struct VisionData
    {
        double timestamp;
        float pitch_angle;                //俯仰角
        float yaw_angle;                  //偏航角
        float distance;                   //目标距离
        int isSwitched;                   //目标是否发生切换
        int isFindTarget;                 //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
        int isSpinning;                   //目标是否处于陀螺状态
        int ismiddle;                     //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
        Eigen::Vector3f linear_velocity;  //线速度
        Eigen::Vector3f angular_velocity; //角速度
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
        void getPosInfo(uchar flag, uchar* raw_data, vector<float>& pos);
        void getHPInfo(uchar flag, uchar* raw_data, vector<uint16_t>& hp);
        void getGameInfo(uchar flag, uchar* raw_data, uint16_t& timestamp);

        int mode_;     
        rclcpp::Logger logger_;  
    private:
        CrcCheck crc_check_;
        float ucharRaw2Float(uchar *data); // 将4个uchar合并成一个float
        bool ucharRaw2FloatVector(uchar *data, int bytes, vector<float> &vec);
        uchar* float2UcharRaw(float float_data);
        bool float2UcharRawArray(float float_data[], int num, uchar* raw_data);
        uint16_t ucharRaw2Int16(uchar *data);
        bool ucharRaw2Int16Vector(uchar *data, int bytes, vector<uint16_t>& vec);
    };
} //namespace serialport

#endif
