/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-20 10:24:11
 * @LastEditTime: 2022-12-11 21:13:17
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/include/buff/buff.hpp
 */
#ifndef BUFF_HPP_
#define BUFF_HPP_

#include <future>
#include <vector>

#include <Eigen/Core>

//ros
#include <rclcpp/rclcpp.hpp>

#include "../fan_tracker/fan_tracker.hpp"
#include "../predictor/predictor.hpp"
#include "../detector/inference.hpp"
#include "../../../global_user/include/global_user/global_user.hpp"
#include "../../../global_user/include/coordsolver.hpp"

using namespace global_user;
using namespace coordsolver;

namespace buff
{
    struct DebugParam
    {
        bool debug_without_com;
        bool using_imu;
        bool using_roi;
        bool show_aim_cross;
        bool show_img;
        bool detect_red;
        bool show_all_fans;
        bool show_fps;
        bool print_letency;
        bool print_target_info;
        bool assist_label;
        bool detect_buff_red;
        bool detect_buff_blue;
        bool show_predict;
        bool prinf_latency;

        DebugParam()
        {
            debug_without_com = true;
            using_imu = false;
            using_roi = false;
            show_aim_cross = false;
            show_img = false;
            detect_red = true;
            show_all_fans = true;
            show_fps = true;
            print_target_info = false; 
            assist_label = false;
            detect_buff_red = true;
            detect_buff_blue = false;
            show_predict = false;
            prinf_latency = false;
        }
    };

    struct BuffParam
    {
        int max_lost_cnt;           //最大丢失目标帧数
        int max_v;                  //最大旋转速度(rad/s)
        int max_delta_t;          //使用同一预测器的最大时间间隔(ms)
        double fan_length;        //大符臂长(R字中心至装甲板中心)
        double no_crop_thres;    //禁用ROI裁剪的装甲板占图像面积最大面积比值

        BuffParam()
        {
            max_lost_cnt = 4; 
            max_v = 4;   
            max_delta_t = 100; 
            fan_length = 0.7;
            no_crop_thres = 2e-3;
        }
    };

    typedef enum BuffMode
    {
        SMALL_BUFF,
        BIG_BUFF
    } BuffMode;

    struct TargetInfo
    {
        Eigen::Vector3d xyz;
        double timestamp;
        double dist;
        double period;
        double radius;
        bool target_switched;
        BuffMode buff_mode;
        TargetInfo()
        {
            dist = 0;
            period = 0;
            radius = 0;
            target_switched = false;
            buff_mode = SMALL_BUFF;
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

    class Buff
    {
    public:
        Buff();
        Buff(const BuffParam& buff_param, const PredictorParam& perdictor_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Buff();

        Eigen::Matrix3d rmat_imu_;
        DebugParam debug_param_;
        bool run(TaskData &src, TargetInfo& target_info);  //主函数

    public:
        bool is_init_;
        BuffParam buff_param_;
        PathParam path_param_;
    private:
        bool is_last_target_exists;
        int lost_cnt;
        int last_timestamp;
        double last_target_area;
        double last_bullet_speed;
        Point2i last_roi_center;
        Point2i roi_offset;
        Size2d input_size;
        std::vector<FanTracker> trackers;  //tracker

        Fan last_fan;
        BuffDetector detector;
        BuffPredictor predictor;
        CoordSolver coordsolver;

        bool chooseTarget(vector<Fan> &fans, Fan &target);
        Point2i cropImageByROI(Mat &img);
    };

} //namespace buff

#endif