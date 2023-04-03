/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-09 22:50:31
 * @LastEditTime: 2023-04-02 05:18:17
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/prediction/param_struct.hpp
 */
#ifndef PARAM_STRUCT_HPP_
#define PARAM_STRUCT_HPP_

//eigen/ceres/yaml-cpp
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

//c++
#include <ctime>
#include <future>
#include <random>
#include <vector>

using namespace std;
namespace armor_processor
{
    typedef enum OutpostStatus
    {
        NORMAL,     // 常速旋转
        CONTROLLED, // 速度减半
        STILL       // 静止
    } OutpostStatus;

    typedef enum SpinningStatus
    {
        STILL_SPINNING,
        MOVEMENT_SPINNING
    } SpinningStatus;

    typedef enum SystemModel
    {
        CSMODEL,
        IMMMODEL
    } SystemModel;

    struct TargetInfo
    {
        Eigen::Vector3d xyz;
        double dist;
        int64_t timestamp;
        double period;
        bool is_target_switched;
        bool is_spinning;
        bool spinning_switched;
        bool is_clockwise;
        bool is_outpost_mode;
        SpinningStatus spinning_status;
        OutpostStatus outpost_status;
        SystemModel system_model;
    };

    struct PredictStatus
    {
        bool xyz_status[3];

        PredictStatus()
        {
            xyz_status[0] = false;
            xyz_status[1] = false;
            xyz_status[2] = false;
        }
    };

    struct FilterModelParam
    {
        vector<double> imm_model_trans_prob_params;
        vector<double> imm_model_prob_params;
        vector<double> process_noise_params;
        vector<double> measure_noise_params;
        // vector<double> singer_model_params;
    };

    struct PredictParam
    {
        double bullet_speed;    //弹速
        int max_delta_time;     //最大时间跨度，大于该值则重置预测器
        int max_cost;           //回归函数最大cost
        int max_v;              //
        int min_fitting_lens;   //最短队列长度
        int shoot_delay;        //射击延迟
        int window_size;        //滑窗大小
        KFParam kf_param;       //卡尔曼滤波参数
        FilterModelParam filter_model_param; //滤波模型参数
        SystemModel system_model;
        double reserve_factor;
        double max_offset_value;
        
        PredictParam()
        {
            bullet_speed = 28;    
            max_delta_time = 1000;     
            max_cost = 509;           
            max_v = 8;              
            min_fitting_lens = 10;   
            shoot_delay = 100;       
            window_size = 3;     
            system_model = CSMODEL;   
            reserve_factor = 15.0;
            max_offset_value = 0.25;
        }
    };

    struct DebugParam
    {
        bool show_img;
        bool using_imu;
        bool draw_predict;
        bool show_predict;
        bool print_delay;
        bool x_axis_filter;
        bool y_axis_filter;
        bool z_axis_filter;
        bool disable_filter;
        bool disable_fitting;
        bool show_transformed_info;
        bool show_aim_cross;
        bool show_fps;

        DebugParam()
        {
            show_img = false;
            using_imu = false;
            draw_predict = true;
            show_predict = false;
            print_delay = false;
            x_axis_filter = true;
            y_axis_filter = false;
            z_axis_filter = false;
            disable_filter = false;
            disable_fitting = false;
            show_transformed_info = true;
            show_aim_cross = true;
            show_fps = true;
        }
    };

    struct PathParam
    {
        std::string coord_path; 
        std::string coord_name;
        std::string filter_path;
        PathParam()
        {
            coord_path = "src/global_user/config/camera.yaml";
            coord_name = "KE0200110075";
            filter_path = "src/global_user/config/filter_param.yaml";
        }
    };

    struct PostProcessInfo
    {
        double hp;           //目标血量，作为是否射击目标依据
        double pred_score;   //预测分数，作为是否射击依据
        bool find_target;    //是否找到目标
        bool is_shooting;    //是否射击目标
        bool switch_target;  //根据目标运动情况以及预测结果判断给出是否切换目标建议
        Eigen::Vector3d pred_3d_pos;  //目标预测点信息
        Eigen::Vector3d track_3d_pos; //目标当前位置
        Eigen::Vector2d pred_velocity;//目标移动速度，作为是否射击目标依据
        Eigen::Vector2d meas_velocity;
    };
} //namespace armor_processor
#endif