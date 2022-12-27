/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:53:56
 * @LastEditTime: 2022-12-27 18:30:57
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/fan_tracker/fan_tracker.hpp
 */
#ifndef FAN_TRACKER_HPP_
#define FAN_TRACKER_HPP_

#include <iostream>
#include <future>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
using namespace cv;
using namespace std;

namespace buff_detector
{
    struct Fan : ObjectBase
    {
        Point2f apex2d[5];
        Eigen::Vector3d centerR3d_cam;
        Eigen::Vector3d centerR3d_world;
    };

    // struct FanTracker
    // {
    //     Fan last_fan;           //上一次扇叶
    //     bool is_last_fan_exists;//是否存在上一次扇叶
    //     double rotate_speed;    //角速度
    //     int last_timestamp;     //上次扇叶时间戳
    // };

    class FanTracker
    {
    public:
        Fan prev_fan;                           //上一次装甲板
        Fan last_fan;                           //本次装甲板
        bool is_last_fan_exists;                //是否存在上一次扇叶
        bool is_initialized;                    //是否完成初始化
        double rotate_speed;                    //角速度
        int max_history_len = 2;          //队列长度
        int prev_timestamp;                     //上次装甲板时间戳
        int last_timestamp;                     //本次装甲板时间戳

        std::deque<Fan> history_info;//目标队列

        FanTracker(Fan src,int src_timestamp);
        bool update(Fan new_fan, int new_timestamp);
    };

} //namespace buff_detector

#endif