/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:48:07
 * @LastEditTime: 2023-04-14 02:25:36
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/armor_tracker/armor_tracker.cpp
 */
#include "../../include/armor_tracker/armor_tracker.hpp"

namespace armor_detector
{
    ArmorTracker::ArmorTracker()
    {
        relative_angle = 0.0;
        this->now = 0;
        this->last_timestamp = 0;
        this->last_selected_timestamp = 0;
    }

    /**
     * @brief Construct a new Armor Tracker:: Armor Tracker object
     * 
     * @param armor 装甲板对象
     * @param now_timestamp 本帧对应的时间戳
     */
    ArmorTracker::ArmorTracker(Armor armor, int64_t now_timestamp)
    {
        this->key = armor.key;
        this->last_timestamp = 0;
        this->last_selected_timestamp = 0;
        this->now = now_timestamp;
        this->last_armor = Armor();
        this->new_armor = armor;
        this->hit_score = 0.0;
        this->relative_angle = 0.0;
        this->history_info_.push_back(armor);
        this->calcTargetScore();
        
        this->is_initialized = false;
    }

    /**
     * @brief 更新追踪器
     * 
     * @param new_armor 本帧检测得到的装甲板信息
     * @param new_timestamp 本帧对应的时间戳
     * @return true 
     * @return false 
     */
    bool ArmorTracker::update(Armor new_add_armor, int64_t new_timestamp)
    {
        if ((int)history_info_.size() <= max_history_len)
        {   // 若历史队列装甲板信息小于给定阈值，直接将当前目标信息放入队列
            history_info_.push_back(new_add_armor);
        }
        else
        {   // 若大于给定阈值，则删除掉过旧信息，添加目标当前信息
            history_info_.pop_front();
            history_info_.push_back(new_add_armor);
        }

        this->last_timestamp = this->now;   //上一帧目标装甲板对应的时间戳信息
        this->now = new_timestamp;          //当前装甲板对应的时间戳信息
        this->last_armor = this->new_armor; //上一帧目标装甲板信息
        this->new_armor = new_add_armor;    //当前装甲板信息

        if (new_add_armor.color == GRAY_SMALL || new_add_armor.color == GRAY_BIG)
        {
            ++gray_armor_cnt_;
            cout << "gray_cnt:" <<gray_armor_cnt_ << endl;
            if (gray_armor_cnt_ >= max_gray_armor_cnt_)
            {
                is_dead_ = true;
            }
        }

        this->calcTargetScore();  //计算装甲板分数，作为打击目标切换判据，防止随意切换造成云台乱抖
        this->is_initialized = true;
        return true;
    }

    /**
     * @brief 计算目标装甲板分数
     * 用于进行目标切换的判定依据
     * @return true 
     * @return false 
     */
    bool ArmorTracker::calcTargetScore()
    {
        vector<Point2f> points;
        float rotate_angle;
        // auto horizonal_dist_to_center = abs(last_armor.center2d.x - 640);

        RotatedRect rotated_rect = new_armor.rrect;
        //调整角度至0-90度(越水平角度越小)
        if (rotated_rect.size.width > rotated_rect.size.height)
            rotate_angle = rotated_rect.angle;
        else
            rotate_angle = 90 - rotated_rect.angle;
        
        // 计算分数
        // 使用log函数压缩角度权值范围
        hit_score = log(0.15 * (90 - rotate_angle) + 10) * (new_armor.area);
        // cout << "hit_socre: " <<rotate_angle<<" "<<" : "<<last_armor.area<<" "<< hit_score << endl;
        return true;
    }
} //namespace armor_detector