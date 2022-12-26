/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:48:07
 * @LastEditTime: 2022-12-26 21:50:31
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/armor_tracker/armor_tracker.cpp
 */
#include "../../include/armor_tracker/armor_tracker.hpp"

namespace armor_detector
{
    ArmorTracker::ArmorTracker()
    {
        
    }
    ArmorTracker::ArmorTracker(Armor src, double src_timestamp)
    {
        last_armor = src;
        last_timestamp = src_timestamp;
        key = src.key;
        is_initialized = false;
        hit_score = 0;
        history_info_.push_back(src);
        calcTargetScore();
    }

    bool ArmorTracker::update(Armor new_armor, double new_timestamp)
    {
        if (history_info_.size() <= max_history_len)
        {   // 若历史队列装甲板信息小于给定阈值，直接将当前目标信息放入队列
            history_info_.push_back(new_armor);
        }
        else
        {   // 若大于给定阈值，则删除掉过旧信息，添加目标当前信息
            history_info_.pop_front();
            history_info_.push_back(new_armor);
        }

        is_initialized = true;
        prev_armor = last_armor;         //上一帧目标装甲板信息
        prev_timestamp = last_timestamp; //上一帧目标装甲板对应的时间戳信息
        last_armor = new_armor;          //当前装甲板信息
        last_timestamp = new_timestamp;  //当前装甲板对应的时间戳信息

        calcTargetScore();  //计算装甲板分数，作为打击目标切换判据，防止随意切换造成云台乱抖
        return true;
    }

    bool ArmorTracker::calcTargetScore()
    {
        vector<Point2f> points;
        float rotate_angle;
        // auto horizonal_dist_to_center = abs(last_armor.center2d.x - 640);

        RotatedRect rotated_rect = last_armor.rrect;
        //调整角度至0-90度(越水平角度越小)
        if (rotated_rect.size.width > rotated_rect.size.height)
            rotate_angle = rotated_rect.angle;
        else
            rotate_angle = 90 - rotated_rect.angle;
        
        // 计算分数
        // 使用log函数压缩角度权值范围
        hit_score = log(0.15 * (90 - rotate_angle) + 10) * (last_armor.area);
        // cout << "hit_socre: " <<rotate_angle<<" "<<" : "<<last_armor.area<<" "<< hit_score << endl;
        return true;
    }
} //namespace armor_detector