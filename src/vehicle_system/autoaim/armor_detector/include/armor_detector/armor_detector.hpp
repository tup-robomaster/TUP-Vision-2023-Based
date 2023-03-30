/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:51:58
 * @LastEditTime: 2023-03-13 10:10:30
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/armor_detector/armor_detector.hpp
 */
//ros
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "../param_struct/param_struct.hpp"
#include "../inference/inference_api2.hpp"
#include "../armor_tracker/armor_tracker.hpp"
#include "../spinning_detector/spinning_detector.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/detection.hpp"
#include "global_interface/msg/detection_array.hpp"
#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/obj_hp.hpp"

using namespace global_user;
using namespace coordsolver;
namespace armor_detector
{
    class Detector
    {
        typedef global_interface::msg::ObjHP ObjHPMsg;

    public:
        Detector(const PathParam& path_params, const DetectorParam& detector_params_, const DebugParam& debug_params_, const GyroParam& gyro_params_);
        ~Detector();

        // void run();
        bool armor_detect(TaskData &src, bool& is_target_lost);
        bool gyro_detector(TaskData &src, global_interface::msg::Autoaim& target_info, ObjHPMsg hp = ObjHPMsg());

        Point2i cropImageByROI(Mat &img);
        ArmorTracker* chooseTargetTracker(TaskData& src, vector<ArmorTracker*> trackers, double timestamp);
        int chooseTargetID(TaskData& src, vector<Armor> &armors, double timestamp);
        int chooseTargetID(TaskData& src, std::vector<Armor>& armors, ObjHPMsg hp = ObjHPMsg());

    public:
        CoordSolver coordsolver_;
        ArmorDetector armor_detector_;
        SpinningDetector spinning_detector_;

        std::vector<Armor> armors_;
        std::vector<Armor> last_armors_;
        atomic<int> target_id_ = -1; 

        bool is_init_;
        ofstream data_save_;
        bool is_save_data_;
        atomic<int> mode_;

    private:
        Armor last_armor_;
        std::vector<ArmorObject> objects_;

        std::vector<ArmorTracker> trackers_;
        std::multimap<std::string, ArmorTracker> trackers_map_;
        std::map<string, int> new_armors_cnt_map_;    //装甲板计数map，记录新增装甲板数
        std::map<std::string, int> car_id_map_;
        Eigen::Matrix3d rmat_imu_;

    private:
        ofstream file_;
        bool save_dataset_;
        std::string path_prefix_ = "src/camera_driver/recorder/autoaim_dataset/";

    private:
        int count_;
        rclcpp::Time time_start_;
        rclcpp::Time time_infer_;
        rclcpp::Time time_crop_;
    
    private:
        int lost_cnt_;
        int64_t timestamp_;
        int dead_buffer_cnt_;
        double last_timestamp_; //当前帧时间戳
        double prev_timestamp_; //上一帧时间戳
        bool is_target_switched_;
        double last_target_area_;
        Point2i last_roi_center_;
        double last_bullet_speed_;
        bool is_last_target_exists_;
        Eigen::Vector3d last_aiming_point_;
        
        Point2i roi_offset_;
        Size2i input_size_;

        void showArmors(TaskData& src);
        
    private:
        SwitchStatus last_last_status_;
        SwitchStatus last_status_;
        SwitchStatus cur_status_;
        
        double cur_period_;
        double last_period_;
        deque<double> history_period_;
        deque<double> new_period_deq_;
        double last_ave_period_;
        double cur_ave_period_;

    public:
        //Debug param.
        DebugParam debug_params_;
        PathParam path_params_;
        DetectorParam detector_params_;

        rclcpp::Logger logger_;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    }; 
} //namespace detector