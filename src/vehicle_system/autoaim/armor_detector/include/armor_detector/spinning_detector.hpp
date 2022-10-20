/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-15 11:25:33
 * @LastEditTime: 2022-10-17 13:53:45
 * @FilePath: /tup_2023-10-16/src/vehicle_system/autoaim/armor_detector/include/armor_detector/spinning_detector.hpp
 */
#include "../../global_user/include/global_user/global_user.hpp"
#include "global_interface/msg/armors.hpp"
#include "./armor_tracker.h"

#include <future>
#include <vector>

#include <Eigen/Core>

namespace armor_detector
{
    enum SpinHeading
    {
        UNKNOWN, 
        CLOCKWISE, 
        COUNTER_CLOCKWISE
    };

    enum Color 
    {
        BLUE,
        RED
    };

    struct gyro_params
    {
        int max_dead_buffer;  //允许因击打暂时熄灭的装甲板的出现次数  
        int max_delta_dist;   //连续目标距离变化范围
        int max_delta_t;      //
        int hero_danger_zone; //英雄危险距离

        int anti_spin_judge_high_thres; //大于该阈值认为该车已开启陀螺
        int anti_spin_judge_low_thres;  //小于该阈值认为该车已关闭陀螺
        int anti_spin_max_r_multiple;
    };

    class spinning_detector
    {
    public: 
        std::map<std::string, SpinHeading> spin_status_map; //反小陀螺，记录该车小陀螺状态
        std::map<std::string, double> spin_score_map;       //反小陀螺，记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
    
    private:
        gyro_params gyro_params_;
        
        Color detect_color;

        armor_detector::Armor last_armor;

        std::vector<armor_detector::Armor> armors;
        std::multimap<std::string, armor_detector::ArmorTracker> trackers_map;
        std::map<std::string, int> new_armors_cnt_map;
     
    public:
        spinning_detector();
        spinning_detector(Color color, gyro_params gyro_params_);
        ~spinning_detector();

        bool update_spin_score();
        void create_armor_tracker(int timestamp, int dead_buffer_cnt);
        bool is_spinning();

        // armor_detector::ArmorTracker* chooseTargetTracker(vector<armor_detector::ArmorTracker*> trackers, int timestamp, int prev_timestamp);
        // int chooseTargetID(vector<armor_detector::Armor> &armors, int timestamp, int prev_timestamp);
    };
} //namespace detector




