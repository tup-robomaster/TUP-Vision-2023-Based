/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 15:55:16
 * @LastEditTime: 2022-12-20 21:22:39
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/buff_detector/buff_detector.hpp
 */
#ifndef BUFF_DETECTOR_HPP_
#define BUFF_DETECTOR_HPP_

#include "../fan_tracker/fan_tracker.hpp"
#include "../inference/inference.hpp"

#include "../../../global_user/include/global_user/global_user.hpp"
#include "../../../global_user/include/coordsolver.hpp"

#include <future>
#include <vector>
#include <Eigen/Core>

using namespace global_user;
using namespace coordsolver;

namespace buff_detector
{
    struct DebugParam
    {
        bool using_imu;
        bool using_roi;
        bool detect_red;
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
        int max_v;                  // 最大旋转速度(rad/s)
        int max_delta_t;            // 使用同一预测器的最大时间间隔(ms)
        double fan_length;          // 大符臂长(R字中心至装甲板中心)
        double no_crop_thres;       // 禁用ROI裁剪的装甲板占图像面积最大面积比值

        BuffParam()
        {
            max_lost_cnt = 4; 
            max_v = 4;   
            max_delta_t = 100; 
            fan_length = 0.7;
            no_crop_thres = 2e-3;
        }
    };

    struct PathParam
    {
        string network_path;
        string camera_param_path;
        string camera_name;
        string path_prefix;

        PathParam()
        {
            network_path = "src/vehicle_system/buff/model/buff.xml";
            camera_param_path = "src/global_user/config/camera.yaml";
            camera_name = "KE0200110075";
            path_prefix = "src/vehicle_system/buff/dataset/";
        }
    };

    struct TargetInfo
    {
        Eigen::Vector3d r_center;
        double rotate_speed;
        double timestamp;
        bool target_switched;
        TargetInfo()
        {
            r_center = {0, 0, 0};
            rotate_speed = 0;
            target_switched = false;
        }
    };

    class Detector
    {
    public:
        Detector();
        Detector(const BuffParam& buff_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Detector();
    
    public:
        BuffParam buff_param_;
        PathParam path_param_;
        DebugParam debug_param_;
        bool is_initialized_;
        BuffDetector buff_detector_;
        CoordSolver coordsolver_;
    
        bool run(TaskData& src, TargetInfo& target_info); //能量机关检测主函数
    
    private:
        bool is_last_target_exists_;
        int lost_cnt_;
        double last_timestamp_;
        double last_target_area_;
        double last_bullet_speed_;
        Point2i last_roi_center_;
        Point2i roi_offset_;
        Size2d input_size_;
        std::vector<FanTracker> trackers_;
        Fan last_fan_;
        Eigen::Matrix3d rmat_imu_;

        bool chooseTarget(std::vector<Fan> &fans, Fan &target);
        cv::Point2i cropImageByROI(cv::Mat &img); //roi裁剪
    };
} // namespace buff_detector

#endif