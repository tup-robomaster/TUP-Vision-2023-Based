/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:54:32
 * @LastEditTime: 2023-05-28 21:07:15
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
    FanTracker::FanTracker(Fan new_buff, uint64_t now)
    {
        new_fan_ = new_buff;
        now_ = now;
        is_initialized_ = false;
        is_last_fan_exists_ = false;
        history_info_.push_back(new_buff);
    }

    bool FanTracker::update(Fan new_fan, uint64_t now)
    {
        is_last_fan_exists_ = true;
        if (history_info_.size() < max_history_len_)
        {
            history_info_.push_back(new_fan);
        }
        else
        {
            is_initialized_ = true;
            history_info_.pop_front();
            history_info_.push_back(new_fan);
        }

        last_fan_ = new_fan_;
        last_timestamp_ = now_;

        new_fan_ = new_fan;
        now_ = now;
        return true;
    }
} //namespace buff_detector