/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-10 15:32:40
 * @LastEditTime: 2023-03-10 15:33:54
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/buff_detector/param_struct.hpp
 */
#ifndef PARAM_STRUCT_HPP_
#define PARAM_STRUCT_HPP_

//c++
#include <future>
#include <vector>
//eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
namespace buff_detector
{
    enum BuffStatus
    {
        UNACTIVATED,
        ACTIVATED
    };

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
        int color;
        int max_lost_cnt;           // 最大丢失目标帧数
        double max_v;                  // 最大旋转速度(rad/s)
        double max_delta_t;            // 使用同一预测器的最大时间间隔(ms)
        double fan_length;          // 大符臂长(R字中心至装甲板中心)
        double no_crop_thres;       // 禁用ROI裁剪的装甲板占图像面积最大面积比值
        double max_angle;

        BuffParam()
        {
            color = 1;
            max_lost_cnt = 10; 
            max_v = 4.0;   
            max_delta_t = 200; 
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
        Eigen::Quaterniond quat_world;
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

#endif