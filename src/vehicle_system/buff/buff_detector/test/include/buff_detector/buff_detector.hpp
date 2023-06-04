/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 15:55:16
 * @LastEditTime: 2023-03-10 15:58:42
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/test/include/buff_detector/buff_detector.hpp
 */
#ifndef BUFF_DETECTOR_HPP_
#define BUFF_DETECTOR_HPP_

#include "../../../include/buff_detector/param_struct.hpp"
#include "../../../include/fan_tracker/fan_tracker.hpp"
#include "../../../include/inference/inference_api2.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace global_user;
using namespace coordsolver;
using namespace std;
namespace buff_detector
{
    class Detector
    {
    public:
        Detector();
        Detector(const BuffParam& buff_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Detector();
    
        bool run(TaskData& src, TargetInfo& target_info, vector<geometry_msgs::msg::Transform>& armor3d_transform_vec, int& flag); //能量机关检测主函数
    
    public:
        BuffParam buff_param_;
        PathParam path_param_;
        DebugParam debug_param_;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        bool is_initialized_;
        BuffDetector buff_detector_;
        CoordSolver coordsolver_;

    private:
        bool is_last_target_exists_;
        int lost_cnt_;
        uint64_t last_last_timestamp_;
        uint64_t last_timestamp_;

        double last_target_area_;
        double last_bullet_speed_;
        Point2i last_roi_center_;
        Point2i roi_offset_;
        Size2d input_size_;
        vector<Fan> fans_;
        // vector<BuffObject> objects_;
        std::vector<FanTracker> trackers_;
        Fan last_fan_;
        
        double last_last_delta_angle_;
        double last_delta_angle_;
        float last_angle_;
        float cur_angle_;
        
        Eigen::Matrix3d rmat_imu_;

        bool chooseTarget(std::vector<Fan> &fans, Fan &target);
        cv::Point2i cropImageByROI(cv::Mat &img); //roi裁剪
        void showFans(TaskData& src);

        vector<double> delta_angle_vec_;
        rclcpp::Logger logger_;
    
    // private:
        // double normalizeAngle(double angle, double dz);
        // double last_angle;
        // deque<Eigen::Vector2d> yaw_pitch_vec_;
        // deque<Eigen::Vector2d> center_yaw_pitch_vec_;
        // deque<double> yaw_vec_;
        // deque<Eigen::Vector3d> armor3d_vec_;
        // deque<Eigen::Vector3d> centerR3d_vec_; 
        // deque<Eigen::Vector3d> rectify3d_vec_;
    };
} // namespace buff_detector

#endif