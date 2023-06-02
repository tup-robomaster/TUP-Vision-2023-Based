/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:48:07
 * @LastEditTime: 2023-05-11 21:55:35
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/armor_tracker/armor_tracker.hpp
 */
#ifndef ARMOR_TRACKER_HPP_
#define ARMOR_TRACKER_HPP_

#pragma once 

//c++
#include <iostream>
#include <future>
#include <vector>

//eigen
#include <Eigen/Core>

//opencv
#include <opencv2/opencv.hpp>

#include "../param_struct/param_struct.hpp"
#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
using namespace cv;
using namespace std;

using namespace global_user;
namespace armor_detector
{
    struct Armor : ObjectBase
    {
        int area;
        Rect roi;
        Rect rect;
        Point2f apex2d[4];
        RotatedRect rrect;
        cv::Point2f center2d;
        TargetType type;
        bool is_front;
        double rangle;

        Armor()
        {
            is_front = false;
        }
    };

    class ArmorTracker
    {
    public:
        ArmorTracker();
        ArmorTracker(Armor src, int64_t src_timestamp);
        bool update(Armor new_armor, int64_t new_timestamp);
        bool calcTargetScore();

    public:
        string key;
        bool is_initialized;                    //是否完成初始化
        double hit_score;                       //该tracker可能作为目标的分数,由装甲板旋转角度,距离,面积大小决定
        Armor last_armor;                       //上一次装甲板
        Armor new_armor;                       //本次装甲板
        int gray_armor_cnt_ = 0;
        bool is_dead_ = false;
        int max_gray_armor_cnt_ = 20;
        
        int64_t now = 0.0;                       //本次装甲板时间戳
        int64_t last_timestamp = 0.0;            //上次装甲板时间戳
        int64_t last_selected_timestamp = 0.0;   //该Tracker上次被选为目标tracker时间戳
        
        int selected_cnt = 0;                   //该Tracker被选为目标tracker次数和
        int max_history_len = 20;          //历史信息队列最大长度
        int history_type_sum;                   //历史次数之和
        std::deque<Armor> history_info_;    //目标队列
        Eigen::Vector3d rotation_center;
        double relative_angle;
        
        // double radius;
        // double velocity;
        // double last_yaw_diff_;
        // double last_pitch_diff_;
        // int64_t hop_timestamp_;
        // SpinHeading spin_status_;
        // int flag_;
        // int normal_gyro_status_counter_;
        // int switch_gyro_status_counter_;
    };
}

#endif