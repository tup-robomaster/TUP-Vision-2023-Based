/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:53:56
 * @LastEditTime: 2023-01-11 22:20:17
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
        double dx;
        double dz;
        double angle;
        Point2f apex2d[5];
        Eigen::Vector3d centerR3d_cam;
        Eigen::Vector3d centerR3d_world;
    };

    class FanTracker
    {
    public:
        Fan last_fan_;                 //上一次装甲板
        Fan new_fan_;                  //本次装甲板
        bool is_last_fan_exists_;      //是否存在上一次扇叶
        bool is_initialized_;          //是否完成初始化
        double rotate_speed_;          //角速度
        double delta_angle_;
        int max_history_len_ = 2;      //队列长度
        
        uint64_t now_;                 //本次装甲板时间戳
        uint64_t last_timestamp_;      //上次装甲板时间戳

        std::deque<Fan> history_info_; //目标队列

        FanTracker(Fan new_fan, uint64_t now);
        bool update(Fan new_fan, uint64_t now);
    };

} //namespace buff_detector

#endif