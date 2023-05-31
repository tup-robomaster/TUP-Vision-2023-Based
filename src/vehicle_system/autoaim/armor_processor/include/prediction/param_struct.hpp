/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-09 22:50:31
 * @LastEditTime: 2023-05-29 22:31:04
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

#include "../../global_user/include/global_user/global_user.hpp"

//IMM Model(CV、CA、CT)
#include "../../../filter/include/model_generator.hpp"

using namespace std;
using namespace global_user;
using namespace filter;
namespace armor_processor
{
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
        double rangle;
        double dist;
        double timestamp;
        double period;
        bool is_target_lost;
        bool is_target_switched;
        bool is_spinning;
        bool is_spinning_switched;
        bool is_clockwise;
        bool is_outpost_mode;
        SpinningStatus spinning_status;
        SystemModel system_model;
    };

    /**
     * @brief 预测器状态
     * 
     */
    enum PredictorState
    {
        TRACKING,   //追踪
        PREDICTING, //预测
        LOSTING,    //丢失预测中
        LOST        //丢失
    };

    enum ShootingMode
    {
        BLOCKING,  // 阻塞
        SINGLE,    // 单发模式
        LIMITED,   // 连发限制模式
        CONTINUOUS // 连发模式
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
        double shoot_delay;     //射击延迟
        double delay_coeff;     //延迟系数（放大时间提前量，缓解云台跟随滞后问题）
        int max_dt;             //最大时间跨度，大于该值则重置预测器(ms)
        int max_cost;           //回归函数最大cost
        int max_v;              //
        int min_fitting_lens;   //最短队列长度
        int window_size;        //滑窗大小
        KFParam kf_param;       //卡尔曼滤波参数
        FilterModelParam filter_model_param; //滤波模型参数
        SystemModel system_model;
        double reserve_factor;
        double max_offset_value;
        
        PredictParam()
        {
            bullet_speed = 28;    
            shoot_delay = 100.0;       
            max_dt = 1000;     
            max_cost = 509;           
            max_v = 8;              
            min_fitting_lens = 10;   
            window_size = 3;     
            system_model = CSMODEL;   
            reserve_factor = 15.0;
            max_offset_value = 0.25;
        }
    };

    struct DebugParam
    {
        bool show_img;
        bool draw_predict;
        bool show_predict;
        bool print_delay;
        bool show_aim_cross;
        bool show_fps;

        DebugParam()
        {
            show_img = false;
            show_fps = false;
            draw_predict = false;
            show_predict = false;
            print_delay = false;
            show_aim_cross = false;
        }
    };

    struct PathParam
    {
        std::string coord_path; 
        std::string coord_name;
        std::string filter_path;
        PathParam()
        {
            coord_path = "/config/camera.yaml";
            coord_name = "KE0200110075";
            filter_path = "/config/filter_param.yaml";
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