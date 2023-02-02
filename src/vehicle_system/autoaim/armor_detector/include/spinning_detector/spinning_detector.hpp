/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-15 11:25:33
 * @LastEditTime: 2023-02-02 16:15:06
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/spinning_detector/spinning_detector.hpp
 */
#ifndef SPINNING_DETECTOR_HPP_
#define SPINNING_DETECTOR_HPP_

#include "../../global_user/include/global_user/global_user.hpp"
#include "../armor_tracker/armor_tracker.hpp"

//c++
#include <future>
#include <vector>

//eigen
#include <Eigen/Core>

//ros
#include <rclcpp/rclcpp.hpp>

namespace armor_detector
{
    enum SpinHeading
    {
        UNKNOWN, 
        CLOCKWISE, //
        COUNTER_CLOCKWISE
    };

    enum Color 
    {
        BLUE,
        RED
    };

    struct GyroParam
    {
        int max_dead_buffer;  //允许因击打暂时熄灭的装甲板的出现次数  
        double max_delta_dist;   //连续目标距离变化范围
        double max_delta_t;      //
        double hero_danger_zone; //英雄危险距离

        double anti_spin_judge_high_thres; //大于该阈值认为该车已开启陀螺
        double anti_spin_judge_low_thres;  //小于该阈值认为该车已关闭陀螺
        double anti_spin_max_r_multiple;

        double delta_x_3d_high_thresh;
        double delta_x_3d_higher_thresh;
        double delta_x_3d_low_thresh;
        double delta_x_3d_lower_thresh;

        double delta_x_2d_high_thresh;
        double delta_x_2d_higher_thresh;
        double delta_x_2d_low_thresh;
        double delta_x_2d_lower_thresh;
        GyroParam()
        {
            delta_x_3d_high_thresh = 0.37;
            delta_x_3d_higher_thresh = 0.44;
            delta_x_3d_low_thresh = 0.23;
            delta_x_3d_lower_thresh = 0.15;

            delta_x_2d_high_thresh = 65;
            delta_x_2d_higher_thresh = 85;
            delta_x_2d_low_thresh = 35;
            delta_x_2d_lower_thresh = 24;
        }
    };

    struct TimeInfo
    {
        double last_timestamp;
        double new_timestamp;
    };

    struct GyroInfo
    {
        double last_x_font;
        double last_x_back;
        double last_timestamp;
        double new_x_font;
        double new_x_back;    
        double new_timestamp;
        int last_x_font_2d;
        int last_x_back_2d;
        int new_x_font_2d;
        int new_x_back_2d;   
        Eigen::Matrix3d last_rmat;
        Eigen::Matrix3d new_rmat;
    };

    struct DetectorInfo
    {
        double last_x;
        double new_x;
        double last_add_tracker_timestamp;
        double new_add_tracker_timestamp;
    };

    struct SpinningMap
    {
        std::map<std::string, SpinHeading> spin_status_map; //反小陀螺，记录该车小陀螺状态
        std::map<std::string, double> spin_score_map;       //反小陀螺，记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
        std::multimap<std::string, TimeInfo> spinning_time_map;
        std::multimap<std::string, GyroInfo> spinning_x_map;
    };
    
    class SpinningDetector
    {
    private:
        Color detect_color;
        Armor last_armor;

        // std::vector<armor_detector::Armor> armors;
        // std::multimap<std::string, ArmorTracker> trackers_map;
        // std::map<std::string, int> new_armors_cnt_map;
    public:
        SpinningDetector();
        SpinningDetector(Color color, GyroParam gyro_params);
        ~SpinningDetector();

        bool updateSpinScore();
        void createArmorTracker(std::multimap<std::string, ArmorTracker>& trackers_map, std::vector<Armor>& armors, std::map<std::string, int>& new_armors_cnt_map, double timestamp, int dead_buffer_cnt);
        bool isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, std::map<std::string, int>& new_armors_cnt_map, double timestamp);
        
        // 记录小陀螺运动下前后两次新增tracker时的时间戳，用以计算小陀螺旋转周期
        
        // 记录小陀螺运动下新增tracker时两个追踪器新增装甲板的x坐标
        // armor_detector::ArmorTracker* chooseTargetTracker(vector<armor_detector::ArmorTracker*> trackers, int timestamp, int prev_timestamp);
        // int chooseTargetID(vector<armor_detector::Armor> &armors, int timestamp, int prev_timestamp);
        SpinningMap spinning_map_;
        DetectorInfo detector_info_;
        GyroParam gyro_params_;
        
        rclcpp::Logger logger_;
    };
} //namespace detector

#endif


