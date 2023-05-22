/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-10 15:35:11
 * @LastEditTime: 2023-03-20 11:38:52
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/test/include/buff_detector/param_struct.hpp
 */
//c++
#include <future>
#include <vector>
//eigen
#include <Eigen/Core>

namespace buff_detector
{
    struct DebugParam
    {
        bool using_imu;
        bool using_roi;
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
        int buff_mode;
        int color;
        int max_lost_cnt;           // 最大丢失目标帧数
        double max_v;                  // 最大旋转速度(rad/s)
        double max_delta_t;            // 使用同一预测器的最大时间间隔(ms)
        double fan_length;          // 大符臂长(R字中心至装甲板中心)
        double no_crop_thres;       // 禁用ROI裁剪的装甲板占图像面积最大面积比值
        double max_angle;

        BuffParam()
        {
            buff_mode = 3;
            color = 1;
            max_lost_cnt = 4; 
            max_v = 4.0;   
            max_delta_t = 100; 
            fan_length = 0.7;
            no_crop_thres = 2e-3;
            max_angle = 0.25;
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
        double timestamp;
        double angle;
        double angle_offset;
        double rotate_speed;
        double delta_angle;
        bool target_switched;
        bool find_target;
        double bullet_speed;
        Eigen::Vector3d r_center;
        Eigen::Vector3d armor3d_cam;
        Eigen::Vector3d armor3d_world;
        Eigen::Matrix3d rmat;
        Eigen::Quaterniond quat_cam;
        cv::Point2f points2d[5];
        TargetInfo()
        {
            r_center = {0, 0, 0};
            rotate_speed = 0.0;
            delta_angle = 0.0;
            target_switched = false;
            find_target = false;
            rmat = Eigen::Matrix3d::Identity();
        }
    };
} //namespace buff_detector