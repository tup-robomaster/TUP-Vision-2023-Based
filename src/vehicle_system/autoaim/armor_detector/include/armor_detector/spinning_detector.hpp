/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-15 11:25:33
 * @LastEditTime: 2022-12-04 17:10:55
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/armor_detector/spinning_detector.hpp
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
        double max_delta_dist;   //连续目标距离变化范围
        int max_delta_t;      //
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

        gyro_params()
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

    typedef struct XCoord
    {
        double last_x_font;
        double last_x_back;
        double new_x_font;
        double new_x_back; 
        int last_timestamp;
        int new_timestamp;
        int last_x_font_2d;
        int last_x_back_2d;
        int new_x_font_2d;
        int new_x_back_2d;   
    } XCoord;

    class spinning_detector
    {
    public: 
        std::map<std::string, SpinHeading> spin_status_map; //反小陀螺，记录该车小陀螺状态
        std::map<std::string, double> spin_score_map;       //反小陀螺，记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转
    
        gyro_params gyro_params_;
    private:
        Color detect_color;
        Armor last_armor;

        // std::vector<armor_detector::Armor> armors;
        // std::multimap<std::string, ArmorTracker> trackers_map;

        // std::map<std::string, int> new_armors_cnt_map;
     
    public:
        spinning_detector();
        spinning_detector(Color color, gyro_params gyro_params_);
        ~spinning_detector();

        bool update_spin_score();
        void create_armor_tracker(std::multimap<std::string, ArmorTracker>& trackers_map, std::vector<armor_detector::Armor>& armors, std::map<std::string, int>& new_armors_cnt_map, int timestamp, int dead_buffer_cnt);
        bool is_spinning(std::multimap<std::string, ArmorTracker>& trackers_map, std::map<std::string, int>& new_armors_cnt_map, int timestamp);
        
        // 记录小陀螺运动下新增tracker时两个追踪器新增装甲板的x坐标
        std::multimap<std::string, XCoord> spinning_x_map;

        // armor_detector::ArmorTracker* chooseTargetTracker(vector<armor_detector::ArmorTracker*> trackers, int timestamp, int prev_timestamp);
        // int chooseTargetID(vector<armor_detector::Armor> &armors, int timestamp, int prev_timestamp);
    };
} //namespace detector




