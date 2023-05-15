/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-02-07 01:45:19
 * @LastEditTime: 2023-05-14 16:10:42
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

#include "../../global_user/include/global_user/global_user.hpp"
#include "../serialport/crc_check.hpp"

using namespace std;
using namespace global_user;
namespace serialport
{
    /**
     * @brief 模式选择（取消视觉，自瞄，英雄吊射，小符，大符，哨兵, 前哨站旋转模式）
     * 
     */
    typedef struct VisionAimData
    {
        double timestamp;
        float pitch_angle;                //俯仰角
        float yaw_angle;                  //偏航角
        float distance;                   //目标距离
        int isSwitched;                   //目标是否发生切换
        int isFindTarget;                 //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
        int isSpinning;                   //目标是否处于陀螺状态
        int isPrediction;                 //当前预测器是否处于预测状态
        int isShooting;                   //是否开火（自动开火控制）
        Eigen::Vector3d meas_tracking_point;
        Eigen::Vector3d pred_aiming_point;
    } VisionAimData;

    typedef struct VisionNavData
    {
        Eigen::Vector3f linear_velocity;  //线速度
        Eigen::Vector3f angular_velocity; //角速度
    } VisionNavData;
    typedef struct VisionDecisionData
    {
        int mode;               //车辆设定模式
        float theta_gimbal;    //指定云台所需转动的相对角度
        float theta_chassis;   //指定底盘所需转动的相对角度
    } VisionDecisionData;

    class DataTransform
    {
    public:
        DataTransform();
        ~DataTransform();

        void transformData(int mode, const VisionAimData &data, uchar* trans_data);
        void transformData(int mode, const VisionNavData &data, uchar* trans_data);
        void transformData(int mode, const VisionDecisionData &data, uchar* trans_data); 
        void getQuatData(uchar* raw_data, vector<float>& quat);
        void getGyroData(uchar* raw_data, vector<float>& gyro);
        void getAccData(uchar* raw_data, vector<float>& acc);
        void getBulletSpeed(uchar* raw_data, float& bullet_speed);
        void getThetaAngle(uchar* raw_data, float& theta);
        void getPosInfo(uchar flag, uchar* raw_data, vector<float>& pos);
        void getHPInfo(uchar flag, uchar* raw_data, vector<ushort>& hp);
        void getGameInfo(uchar flag, uchar* raw_data, ushort& timestamp);
        void getYawAngle(uchar flag, uchar* raw_data, float& yaw_angle);
        void getPitchAngle(uchar flag, uchar* raw_data, float& pitch_angle);
        void getShootDelay(uchar* raw_data, float& shoot_delay);

        int mode_;     
        rclcpp::Logger logger_;  
    private:
        CrcCheck crc_check_;
        float ucharRaw2Float(uchar *data); // 将4个uchar合并成一个float
        bool ucharRaw2FloatVector(uchar *data, int bytes, vector<float> &vec);
        uchar* float2UcharRaw(float float_data);
        bool float2UcharRawArray(float float_data[], int num, uchar* raw_data);
        ushort ucharRaw2Int16(uchar *data);
        bool ucharRaw2Int16Vector(uchar *data, int bytes, vector<ushort>& vec);
    };
} //namespace serialport

#endif
