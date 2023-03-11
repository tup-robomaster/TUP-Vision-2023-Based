/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:54:32
 * @LastEditTime: 2023-01-03 21:15:35
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/src/fan_tracker/fan_tracker.cpp
 */
#include "../../include/fan_tracker/fan_tracker.hpp"

namespace buff_detector
{
    /**
     * @brief 构造一个ArmorTracker对象
     * 
     * @param src Armor对象
     */
    FanTracker::FanTracker(Fan src, double src_timestamp)
    {
        last_fan = src;
        last_timestamp = src_timestamp;
        is_initialized = false;
        is_last_fan_exists = false;
        history_info.push_back(src);
    }

    bool FanTracker::update(Fan new_fan, double new_timestamp)
    {
        is_last_fan_exists = true;
        if (history_info.size() < max_history_len)
        {
            history_info.push_back(new_fan);
        }
        else
        {
            is_initialized = true;
            history_info.pop_front();
            history_info.push_back(new_fan);
        }

        prev_fan = last_fan;
        prev_timestamp = last_timestamp;
        
        last_fan = new_fan;
        last_timestamp = new_timestamp;

        return true;
    }
} //namespace buff_detector