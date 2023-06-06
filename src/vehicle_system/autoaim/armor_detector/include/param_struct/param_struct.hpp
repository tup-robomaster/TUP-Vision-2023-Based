/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-10 15:53:36
 * @LastEditTime: 2023-06-02 22:08:06
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/param_struct/param_struct.hpp
 */
#ifndef PARAM_STRUCT_HPP_
#define PARAM_STRUCT_HPP_

//c++
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <atomic>
#include <future>
#include <vector>

//angle
#include <angles/angles.h>

//eigen
#include <Eigen/Core>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace armor_detector
{
    enum ArmorType
    {
        BLUE_SMALL,
        BLUE_BIG,
        RED_SMALL,
        RED_BIG,
        GRAY_SMALL,
        GRAY_BIG,
        PURPLE_SMALL,
        PURPLE_BIG
    };

    struct SpinState
    {
        int64_t switch_timestamp;
        SpinHeading spin_state;
        SpinState()
        {
            switch_timestamp = 0;
            spin_state = UNKNOWN;
        }
    };

    enum SwitchStatus
    {
        NONE,
        SINGER,
        DOUBLE
    };
    
    struct GyroParam
    {
        int max_dead_buffer;  //允许因击打暂时熄灭的装甲板的出现次数  
        double max_delta_dist;   //连续目标距离变化范围
        double max_delta_t;      //tracker未更新的时间上限，过久则删除
        double switch_max_dt;    //目标陀螺状态未更新的时间上限，过久则删除
        double hero_danger_zone; //英雄危险距离

        double anti_spin_judge_high_thresh; //大于该阈值认为该车已开启陀螺
        double anti_spin_judge_low_thresh;  //小于该阈值认为该车已关闭陀螺
        double anti_spin_max_r_multiple;

        double delta_x_3d_high_thresh;
        double delta_x_3d_higher_thresh;
        double delta_x_3d_low_thresh;
        double delta_x_3d_lower_thresh;

        double delta_y_3d_high_thresh;
        double delta_y_3d_higher_thresh;
        double delta_y_3d_low_thresh;
        double delta_y_3d_lower_thresh;

        double max_rotation_angle;
        double min_rotation_angle;
        double max_hop_period;
        double max_conf_dis;
        GyroParam()
        {
            max_delta_t = 100;
            switch_max_dt = 2000.0;
            delta_x_3d_high_thresh = 0.18;
            delta_x_3d_higher_thresh = 0.25;
            delta_x_3d_low_thresh = 0.10;
            delta_x_3d_lower_thresh = 0.05;

            delta_y_3d_high_thresh = 0.18;
            delta_y_3d_higher_thresh = 0.25;
            delta_y_3d_low_thresh = 0.10;
            delta_y_3d_lower_thresh = 0.05;

            max_rotation_angle = 10.0 * (M_PI / 180);
            min_rotation_angle = 3.0 * (M_PI / 180);
            max_hop_period = 3.0;
            max_conf_dis = 3.5;
        }
    };

    struct TimeInfo
    {
        double last_timestamp;
        double new_timestamp;
    };

    /**
     * @brief 装甲板切换时的信息（同时存在两个装甲板，一个新增，一个先前存在）
     * 
     */
    struct GyroInfo
    {
        double last_x_font;
        double last_x_back;
        double new_x_font;
        double new_x_back;    
        double last_timestamp;
        double new_timestamp;
        double last_y_font;
        double last_y_back;
        double new_y_font;
        double new_y_back;    
        Eigen::Matrix3d last_rmat;
        Eigen::Matrix3d new_rmat;
    };

    struct DetectorInfo
    {
        double last_x;
        double new_x;
        double last_y;
        double new_y;
        double last_add_tracker_timestamp;
        double new_add_tracker_timestamp;
    };

    struct SpinCounter
    {
        int flag;
        int normal_gyro_status_counter;
        int switch_gyro_status_counter;
        SpinCounter()
        {
            normal_gyro_status_counter = 0;
            switch_gyro_status_counter = 0;
            flag = 0;
        }
    };

    struct SpinningMap
    {
        std::map<std::string, SpinState> spin_status_map;    //反小陀螺，记录该车小陀螺状态
        std::map<std::string, SpinCounter> spin_counter_map; //记录装甲板旋转帧数，大于0为逆时针旋转，小于0为顺时针

        std::multimap<std::string, TimeInfo> spinning_time_map;
        std::multimap<std::string, GyroInfo> spinning_x_map;
    };

    struct DetectorParam
    {
        int color;
        int max_lost_cnt;        //最大丢失目标帧数
        int max_armors_cnt;    //视野中最多装甲板数

        double armor_type_wh_high_thresh; //大小装甲板长宽比高阈值
        double armor_type_wh_low_thresh;  //大小装甲板长宽比低阈值
        double armor_type_wh_thresh;  //大小装甲板长宽比阈值

        double no_crop_thresh; //禁用ROI裁剪的装甲板占图像面积最大面积比值
        double hero_danger_zone; //英雄危险距离阈值，检测到有小于该距离的英雄直接开始攻击
        double no_crop_ratio;
        double full_crop_ratio;
        double fire_zone;

        // double max_delta_dist;
        double armor_roi_expand_ratio_width;
        double armor_roi_expand_ratio_height;
        double armor_conf_high_thresh;

        DetectorParam()
        {
            color = 1; //(Red:1/Blue:0)
            max_lost_cnt = 5;
            max_armors_cnt = 8;
            
            armor_type_wh_thresh = 3.0;
            armor_type_wh_high_thresh = 3.0;
            armor_type_wh_low_thresh = 2.5;  
             
            no_crop_thresh = 2e-3;
            no_crop_ratio = 2e-3;
            full_crop_ratio = 1e-4;

            hero_danger_zone = 4.0;
            fire_zone = 5.0;
            armor_roi_expand_ratio_width = 1.1;
            armor_roi_expand_ratio_height = 1.5;
            armor_conf_high_thresh = 0.82;
        }
    };

    struct DebugParam
    {
        bool use_imu;
        bool use_roi;
        bool show_aim_cross;
        bool show_img;
        bool show_crop_img;
        bool show_all_armors;
        bool show_fps;
        bool print_latency;
        bool print_target_info;
        bool save_data;
        bool save_dataset;

        DebugParam()
        {
            use_imu = true;
            use_roi = false;
            show_img = false;
            show_fps = false;
            save_data = false;
            save_dataset = false;
            show_crop_img = false;
            print_latency = false;
            show_aim_cross = false;
            show_all_armors = false;
            print_target_info = false; 
        }
    };
    
    struct PathParam
    {
        std::string camera_name;
        std::string camera_param_path;
        std::string network_path;
        std::string save_path;
    };

} //namespace armor_detector

#endif