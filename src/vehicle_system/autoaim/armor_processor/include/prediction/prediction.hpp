/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 11:28:53
 * @LastEditTime: 2023-04-08 20:11:23
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/prediction/prediction.hpp
 */
#ifndef PREDICTION_HPP_
#define PREDICTION_HPP_

#pragma once

#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "../filter/particle_filter.hpp"
// #include "../filter/kalman_filter.hpp"

//opencv
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//ceres
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

//c++
#include <ctime>
#include <future>
#include <random>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>

// #include <stdio.h>
// #include <lapacke.h> //如果出现找不到此文件夹的错误，注释掉试一试，不行就去官方安装

//运动模型（CV、CA、CTRV、CT、Singer、CS）
// #include "../../../../filter/test/system_model.cpp"
// #include "../../../../filter/test/measurement_model.cpp"

//IMM Model(CV、CA、CT)
#include "../filter/model_generator.hpp"

//Singer Model
// typedef SingerModelState<double> SingerState;
// typedef SingerModelControl<double> SingerControl;
// typedef SingerModel<double> Singer;
// typedef SingerPositionMeasurement<double> SingerPosMeasure;
// typedef SingerPositionMeasurementModel<double> SingerPosModel;

#include "global_interface/msg/autoaim.hpp"

using namespace global_user;
using namespace coordsolver;
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

    typedef struct TargetInfo
    {
        Eigen::Vector3d xyz;
        int dist;
        double timestamp;
        double period;
        bool is_target_switched;
        bool is_spinning;
        bool spinning_switched;
        bool is_sentry_mode;
        bool is_clockwise;
        SpinningStatus spinning_status;
        OutpostStatus outpost_status;
        SystemModel system_model;
    } TargetInfo;

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

        PredictParam()
        {
            bullet_speed = 28;    
            max_delta_time = 1000;     
            max_cost = 509;           
            max_v = 8;              
            min_fitting_lens = 10;   
            shoot_delay = 100;       
            window_size = 3;        
        }
    };

    struct DebugParam
    {
        bool disable_filter;
        bool disable_fitting;
        bool draw_predict;
        bool using_imu;
        bool show_predict;
        bool show_aim_cross;
        bool show_transformed_info;
        bool show_img;

        DebugParam()
        {
            disable_filter = false;
            disable_fitting = false;
            draw_predict = true;
            show_aim_cross = true;
            using_imu = false;
            show_predict = false;
            show_transformed_info = true;
            show_img = true;
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
    
    struct CurveFittingCost
    {
        const bool _axis;
        const double _x, _y, _t, _coeff;

        CurveFittingCost(bool axis, double x, double y, double t, double coeff)
        : _axis(axis), _x (x), _y(y), _t(t), _coeff(coeff) {}

        //计算残差
        template<class T>
        bool operator()(
            const T* const params, // 模型参数，有3维
            T* residual) const     // 残差
        {
            // residual[0] = T (_x) - params[0] * T(_t); // f(x) = a0 + a1 * t + a2 * t^2 
            residual[0] = T (_x) - params[0] * T(_t) - params[1] * T(_t) * T(_t); // f(x) = a0 + a1 * t + a2 * t^2 
            // residual[0] = T (_x) - params[0] * ceres::cos(params[1] * T (_t) + params[2]); // f(x) = a0 + a1 * cos(wt + THETA)
            // residual[0] = T (_x) - params[0] * ceres::cos(params[2] * T (_t)) - params[1] * ceres::sin(params[2] * T (_t)); // f(x) = a0 + a1 * cos(wt) + b1 * sin(wt) 
            return true;
        }

        // 前哨站旋转装甲板轨迹拟合
        template<class T>
        bool operator()
        (
            const T* const x0,
            const T* const y0,
            const T* const theta,
            T* residual
        ) const
        {
            if(!_axis)
            {   // x轴
                residual[0] = x0[0] + 0.2765 * ceres::cos(0.8 * M_PI * _coeff * _t + theta[0]) - _x;
            }
            else
            {   // y轴
                residual[0] = y0[0] + 0.2765 * ceres::sin(0.8 * M_PI * _coeff * _t + theta[0]) - _y;
            }
            return true;
        }

        //小陀螺+左右横移运动轨迹拟合（反陀螺）
        template <class T>
        bool operator()
        (
            const T* const w,
            const T* const theta,
            const T* const V,
            const T* const x0,
            const T* const y0,
            const T* const a,
            const T* const b,
            const T* const phi,
            T* residual
        ) const
        {
            if(!_axis)
            {   //x轴
                // residual[0] = x0[0] + a[0] * ceres::cos(w[0] * T(_t)) + V[0] * T(_t) * ceres::cos(theta[0]) * _coeff - _x;
                residual[0] = x0[0] + a[0] * ceres::cos(w[0] * T(_t) + phi[0]) + V[0] * T(_t) * ceres::cos(theta[0]) * _coeff - _x;
            }
            else
            {   //y轴
                // residual[0] = y0[0] + b[0] * ceres::sin(w[0] * T(_t)) + V[0] * T(_t) * ceres::sin(theta[0]) * _coeff - _y;
                residual[0] = y0[0] + b[0] * ceres::sin(w[0] * T(_t) + phi[0]) + V[0] * T(_t) * ceres::sin(theta[0]) * _coeff - _y;
            }
            return true;
        }

        template <class T>
        bool operator()
        (
            const T* const k,
            const T* const d,
            const T* const a,
            const T* const b,
            const T* const c,
            T* residual
        ) const
        {
            if(!_axis)
            {   //x轴 
                // residual[0] = k[0] * T(_t) + _coeff - _x; //f(t)=kt+x0
                residual[0] = k[0] * T(_t) + d[0] - _x; //f(t)=kt+d
            }  
            else
            {   //f(t)=a*(k^2)*(t^2)+(2kad+kb)*t+a(d^2)+bd+c
                // residual[0] = a[0] * pow(k[0], 2) * pow(T(_t), 2) 
                //             + ((2.0 * (k[0] * a[0] * d[0])) + (k[0] * b[0])) * T(_t) 
                //             + a[0] * pow(d[0], 2) + b[0] * d[0] + c[0] - _y; 

                //f(t)=a*(t^2)+b*t+c
                // residual[0] = a[0] * pow(T(_t), 2) + b[0] * T(_t) + c[0] - _y;

                //f(t)=(a/t) + b
                // residual[0] = b[0] / T(_t) + c[0] - _y;

                //f(t)=(1/t) + b
                residual[0] = (1.0 / T(_t)) + c[0] - _y;
            }

            return true;
        }
    };

    class ArmorPredictor
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        
    public:
        ArmorPredictor();
        ~ArmorPredictor();
        ArmorPredictor(const PredictParam& predict_param, const vector<double>* singer_param, 
            const PathParam& path_param, const DebugParam& debug_param);
    
    private:
        ParticleFilter pf_pos;  //目前坐标粒子滤波
        ParticleFilter pf_v;    //速度粒子滤波

        std::deque<TargetInfo> history_info_; //历史测量信息
        std::deque<TargetInfo> history_pred_; //历史预测信息
        int history_deque_lens_; //历史队列长度

    public:
        YAML::Node config_;
        PredictParam predict_param_;  //滤波先验参数/模型先验参数/调试参数
        vector<double> singer_param_; //cs模型参数 
        DebugParam debug_param_;
        std::string filter_param_path_;
        
        bool setBulletSpeed(double speed);
        void loadParam(std::string filter_param_path);
        Eigen::Vector3d predict(AutoaimMsg& target_msg, double timestamp, double& sleep_time, cv::Mat* src = nullptr);
    
    private:
        std::deque<cv::Point2d> history_pred_info_;
        std::deque<cv::Point2d> history_origin_info_;
        std::deque<double> history_y_info_;
        double last_start_timestamp_;
        double last_end_x_;
        bool is_predicted;
        TargetInfo final_target_;  //最终击打目标信息
        TargetInfo last_pf_target_; //最后一次粒子滤波后的位置结果

    private:
        double evalRMSE(double* params);
        double calcError();
        Eigen::Vector3d shiftWindowFilter(int start_idx);
        PredictStatus predictBasePF(TargetInfo target, Vector3d& result, double timestamp);
        PredictStatus uncoupleFittingPredict(Eigen::Vector3d& result, double timestamp);
        PredictStatus coupleFittingPredict(bool is_still_spinning, TargetInfo target, Eigen::Vector3d& result, double timestamp);

        //移动轨迹拟合预测（小陀螺+横移->旋轮线，若目标处于原地小陀螺状态，则剔除掉模型中的横移项）
        double fitting_params_[5] = {0.1, 0.1, 0.1, 0.1, 0.1};
        double fitting_y_params_[5] = {0.1, 0.1, 0.1, 0.1, 0.1};
    
    private:
        rclcpp::Logger logger_;
        double history_vx_[4] = {0};
        double history_acc_[4] = {0};
        double predict_vx_[4] = {0};
        double predict_acc_[4] = {0};

    private:
        // 卡尔曼滤波
        KalmanFilter kalman_filter_;
        void kfInit(); // 滤波参数初始化（矩阵维度、初始值）

    public:
        bool is_init;
        bool is_ekf_init;
        bool is_imm_init;
        bool fitting_disabled_; // 是否禁用曲线拟合
        bool filter_disabled_;  // 是否禁用滤波
        int error_cnt_ = 0;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    
    private:
        // IMM Model.
        std::shared_ptr<IMM> imm_;
        ModelGenerator model_generator_;
        PredictStatus predictBasedImm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d& target_v, double& ax, double timestamp);
        // CS Model.
        PredictStatus predictBasedSinger(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d target_vel, Eigen::Vector2d target_acc, double timestamp);

        // 前哨站旋转装甲板曲线拟合预测函数    
        PredictStatus spinningPredict(bool is_controlled, TargetInfo& target, Eigen::Vector3d& result, double timestamp);
    };

} //namespace armor_processor

#endif