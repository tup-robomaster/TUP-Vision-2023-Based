/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-15 11:25:33
 * @LastEditTime: 2023-05-05 03:29:18
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/spinning_detector/spinning_detector.hpp
 */
#ifndef SPINNING_DETECTOR_HPP_
#define SPINNING_DETECTOR_HPP_

#include "../../global_user/include/global_user/global_user.hpp"
#include "../armor_tracker/armor_tracker.hpp"
#include "../param_struct/param_struct.hpp"
#include "../param_struct/param_struct.hpp"

//ros
#include <rclcpp/rclcpp.hpp>

namespace armor_detector
{
    class SpinningDetector
    {
    private:
        Armor last_armor_;
        Color detect_color_;
        rclcpp::Logger logger_;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        DetectorInfo detector_info_;

    public:
        SpinningDetector();
        SpinningDetector(Color color, GyroParam gyro_params);
        ~SpinningDetector();

        // bool updateSpinScore();
        void createArmorTracker(std::multimap<std::string, ArmorTracker>& trackers_map,
            std::vector<Armor>& armors, std::map<std::string, int>& new_armors_cnt_map, int64_t timestamp, bool is_last_exists);
        bool isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, int64_t now);
        bool isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, std::map<std::string, int>& new_armors_cnt_map, int64_t now);
        
        int dead_buffer_cnt_ = 0;
        bool is_gray_exists_ = false;
        int gray_id_;
        bool is_dead_;
        
        double max_hop_period_;
        double last_timestamp_;
       
        GyroParam gyro_params_;
        SpinningMap spinning_map_;

        //history info
        ArmorTracker last_armor_tracker_;
        ArmorTracker cur_armor_tracker_;
        
        // queue<Armor> armors_queue_; 
        // map<int64_t, Armor> last_armors_map_;
        // map<int64_t, Armor> cur_armors_map_;
    };
} //namespace detector

#endif


