/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:48:07
 * @LastEditTime: 2023-03-31 18:35:41
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
        cv::Point2d center2d;
        TargetType type;
    };

    class ArmorTracker
    {
    public:
        Armor last_armor;                       //上一次装甲板
        Armor new_armor;                       //本次装甲板
        bool is_initialized;                    //是否完成初始化
        int64_t last_selected_timestamp = 0.0;   //该Tracker上次被选为目标tracker时间戳
        int64_t last_timestamp = 0.0;            //上次装甲板时间戳
        int64_t now = 0.0;                       //本次装甲板时间戳
        int history_type_sum;                   //历史次数之和
        int selected_cnt = 0;                   //该Tracker被选为目标tracker次数和
        const int max_history_len = 4;          //历史信息队列最大长度
        double hit_score;                       //该tracker可能作为目标的分数,由装甲板旋转角度,距离,面积大小决定
        double velocity;
        double radius;
        string key;

        double last_yaw_diff_;
        double last_pitch_diff_;
        double hop_timestamp_;
        // SpinHeading spin_status_;
        // int flag_;
        // int normal_gyro_status_counter_;
        // int switch_gyro_status_counter_;
        std::deque<Armor> history_info_;    //目标队列

        ArmorTracker();
        ArmorTracker(Armor src, double src_timestamp);
        bool update(Armor new_armor, double new_timestamp);
        bool calcTargetScore();
    };
}

#endif