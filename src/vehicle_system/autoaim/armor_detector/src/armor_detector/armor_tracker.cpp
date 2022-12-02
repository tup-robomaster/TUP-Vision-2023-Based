/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:48:07
 * @LastEditTime: 2022-10-15 13:31:13
 * @FilePath: /tup_2023-10-16/src/vehicle_system/autoaim/armor_detector/src/armor_tracker.cpp
 */
#include "../../include/armor_detector/armor_tracker.h"

namespace armor_detector
{
    ArmorTracker::ArmorTracker()
    {
        
    }
    ArmorTracker::ArmorTracker(Armor src, int src_timestamp)
    {
        last_armor = src;
        last_timestamp = src_timestamp;
        key = src.key;
        is_initialized = false;
        hit_score = 0;
        history_info.push_back(src);
        calcTargetScore();
    }

    bool ArmorTracker::update(Armor new_armor, int new_timestamp)
    {
        if (history_info.size() <= max_history_len)
        {
            history_info.push_back(new_armor);
        }
        else
        {
            history_info.pop_front();
            history_info.push_back(new_armor);
        }

        is_initialized = true;
        prev_armor = last_armor;
        prev_timestamp = last_timestamp;
        last_armor = new_armor;
        last_timestamp = new_timestamp;

        calcTargetScore();
        
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
        
        //计算分数
        //使用log函数压缩角度权值范围
        hit_score = log(0.15 * (90 - rotate_angle) + 10) * (last_armor.area);
        // cout << "hit_socre: " <<rotate_angle<<" "<<" : "<<last_armor.area<<" "<< hit_score << endl;
        return true;
    }
}