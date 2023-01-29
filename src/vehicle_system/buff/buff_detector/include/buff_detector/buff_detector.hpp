/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 15:55:16
 * @LastEditTime: 2023-01-29 22:01:09
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/buff_detector/buff_detector.hpp
 */
#ifndef BUFF_DETECTOR_HPP_
#define BUFF_DETECTOR_HPP_

#include "../fan_tracker/fan_tracker.hpp"
#include "../inference/inference_api2.hpp"

#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"

#include <future>
#include <vector>

//eigen
#include <Eigen/Core>

//ros
#include <rclcpp/rclcpp.hpp>

using namespace global_user;
using namespace coordsolver;

namespace buff_detector
{
    struct DebugParam
    {
        bool using_imu;
        bool using_roi;
        bool detect_red;
        bool show_img;
        bool show_all_fans;
        bool show_fps;
        bool print_target_info;
        bool assist_label;
        bool prinf_latency;

        DebugParam()
        {
            using_imu = false;
            using_roi = false;
            detect_red = true;
            show_img = true;
            show_all_fans = true;
            show_fps = true;
            print_target_info = false; 
            prinf_latency = false;
            assist_label = false;
        }
    };

    struct BuffParam
    {
        int max_lost_cnt;           // 最大丢失目标帧数
        double max_v;                  // 最大旋转速度(rad/s)
        double max_delta_t;            // 使用同一预测器的最大时间间隔(ms)
        double fan_length;          // 大符臂长(R字中心至装甲板中心)
        double no_crop_thres;       // 禁用ROI裁剪的装甲板占图像面积最大面积比值

        BuffParam()
        {
            max_lost_cnt = 4; 
            max_v = 4.0;   
            max_delta_t = 100; 
            fan_length = 0.7;
            no_crop_thres = 2e-3;
        }
    };

    struct PathParam
    {
        string camera_name;
        string network_path;
        string camera_param_path;
        string path_prefix;

        PathParam()
        {
            camera_name = "KE0200110075";
            network_path = "src/vehicle_system/buff/model/buff.xml";
            camera_param_path = "src/global_user/config/camera.yaml";
            path_prefix = "src/recorder/buff_dataset";
        }
    };

    struct TargetInfo
    {
        Eigen::Vector3d r_center;
        double rotate_speed;
        double timestamp;
        bool target_switched;
        Eigen::Matrix3d rmat;
        Eigen::Vector3d armor3d_world;
        Eigen::Vector3d armor3d_cam;
        TargetInfo()
        {
            r_center = {0, 0, 0};
            rotate_speed = 0;
            target_switched = false;
            rmat = Eigen::Matrix3d::Identity();
        }
    };

    class Detector
    {
    public:
        Detector();
        Detector(const BuffParam& buff_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Detector();
    
        bool run(TaskData& src, TargetInfo& target_info); //能量机关检测主函数
    private:
        BuffParam buff_param_;
        PathParam path_param_;
        DebugParam debug_param_;
    
    public:
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        bool is_initialized_;
        BuffDetector buff_detector_;
        CoordSolver coordsolver_;

        void setDetectorParam(const double& param, int idx);
        void setDebugParam(const bool& param, int idx);
    private:
        bool is_last_target_exists_;
        int lost_cnt_;
        double last_timestamp_;
        double last_target_area_;
        double last_bullet_speed_;
        Point2i last_roi_center_;
        Point2i roi_offset_;
        Size2d input_size_;
        vector<Fan> fans_;
        // vector<BuffObject> objects_;
        std::vector<FanTracker> trackers_;
        Fan last_fan_;
        Eigen::Matrix3d rmat_imu_;
        float last_angle_;
        float cur_angle_;

        bool chooseTarget(std::vector<Fan> &fans, Fan &target);
        cv::Point2i cropImageByROI(cv::Mat &img); //roi裁剪
        void printTargetInfo(int idx);
        void showFans(TaskData& src);

        rclcpp::Logger logger_;
    };
} // namespace buff_detector

#endif