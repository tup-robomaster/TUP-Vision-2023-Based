/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-10 15:32:40
 * @LastEditTime: 2023-03-10 15:33:54
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/buff_detector/param_struct.hpp
 */
#ifndef PARAM_STRUCT_HPP_
#define PARAM_STRUCT_HPP_

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
        int color;
        int buff_mode;
        int max_lost_cnt;           // 最大丢失目标帧数
        double max_v;                  // 最大旋转速度(rad/s)
        double max_delta_t;            // 使用同一预测器的最大时间间隔(ms)
        double fan_length;          // 大符臂长(R字中心至装甲板中心)
        double no_crop_thres;       // 禁用ROI裁剪的装甲板占图像面积最大面积比值

        BuffParam()
        {
            color = 1; // red: 1 / blue: 0
            buff_mode = 3; // small: 3 / big: 4
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
} //namepsace buff_detector

#endif