/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 11:28:53
 * @LastEditTime: 2022-11-17 12:49:55
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/prediction/prediction.h
 */
#ifndef PREDICTION_HPP
#define PREDICTION_HPP

#pragma once

#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "../filter/particle_filter.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

#include <ctime>
#include <future>
#include <random>
#include <vector>

#include <stdio.h>
// #include <lapacke.h> //如果出现找不到此文件夹的错误，注释掉试一试，不行就去官方安装

//运动模型（CV、CA、CTRV、CT、Singer、CS）
#include "src/vehicle_system/filter/test/system_model.cpp"
#include "src/vehicle_system/filter/test/measurement_model.cpp"

//Singer Model
typedef SingerModelState<double> SingerState;
typedef SingerModelControl<double> SingerControl;
typedef SingerModel<double> Singer;
typedef SingerPositionMeasurement<double> SingerPosMeasure;
typedef SingerPositionMeasurementModel<double> SingerPosModel;

namespace armor_processor
{
    typedef struct TargetInfo
    {
        Eigen::Vector3d xyz;
        int dist;
        int timestamp;
    } TargetInfo, *TargetInfoPtr;

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

    struct CurveFittingCost
    {
        const double _x, _y;

        CurveFittingCost(double x, double y)
        : _x (x), _y(y) {}

        //计算残差
        template<class T>
        bool operator()(
            const T* const params, // 模型参数，有3维
            T* residual) const     // 残差
        {
            // residual[0] = T (_y) - params[0] * T(_x); // f(x) = a0 + a1 * x + a2 * x^2 
            residual[0] = T (_y) - params[0] * T(_x) - params[1] * T(_x) * T(_x); // f(x) = a0 + a1 * x + a2 * x^2 
            // residual[0] = T (_y) - params[0] * ceres::cos(params[1] * T (_x) + params[2]); // f(x) = a0 + a1 * cos(wx + THETA)
            // residual[0] = T (_y) - params[0] * ceres::cos(params[2] * T (_x)) - params[1] * ceres::sin(params[2] * T (_x)); // f(x) = a0 + a1 * cos(wx) + b1 * sin(wx) 

            return true;
        }
    };

    struct XAxisFitting
    {
        double _x, _y, _t;
        XAxisFitting(double x, double y, double t) : _x(x), _y(y), _t(t) {}

        template <class T>
        bool operator()
        (
            const T* const w,
            const T* const theta,
            const T* const V,
            const T* const x0,
            const T* const y0,
            T* residual
        ) const
        {
            residual[0] = x0[0] + 0.25 * ceres::cos(w[0] * T(_t)) + V[0] * T(_t) * ceres::cos(theta[0]) - _x;
            return 0;
        }
    };

    struct YAxisFitting
    {
        double _x, _y, _t;
        YAxisFitting(double x, double y, double t) : _x(x), _y(y), _t(t) {}

        template <class T>
        bool operator()
        (
            const T* const w,
            const T* const theta,
            const T* const V,
            const T* const x0,
            const T* const y0,
            T* residual
        ) const
        {
            residual[0] = y0[0] + 0.25 * ceres::sin(w[0] * T(_t)) + V[0] * T(_t) * ceres::sin(theta[0]) - _y;
            return 0;
        }
    };

    struct PredictParam
    {
        double bullet_speed;    //弹速
        int max_time_delta;     //最大时间跨度，大于该值则重置预测器
        int max_cost;           //回归函数最大cost
        int max_v;              //
        int min_fitting_lens;   //最短队列长度
        int shoot_delay;        //射击延迟
        int window_size;        //滑窗大小
        
        PredictParam()
        {
            bullet_speed = 28;    
            max_time_delta = 1000;     
            max_cost = 509;           
            max_v = 8;              
            min_fitting_lens = 10;   
            shoot_delay = 100;       
            window_size = 3;        
        }
    };

    struct DebugParam
    {
        bool disable_fitting;
        bool draw_predict;

        bool using_imu;
        bool show_predict;

        bool show_transformed_info;

        DebugParam()
        {
            disable_fitting = false;
            draw_predict = false;

            using_imu = false;
            show_predict = true;
            show_transformed_info = true;
        }
    };

    class ArmorPredictor
    {
    public:
        ArmorPredictor();
        ~ArmorPredictor();
    
    private:
        bool fitting_disabled_;

        ParticleFilter pf_pos;  //目前坐标粒子滤波
        ParticleFilter pf_v;    //速度粒子滤波

        std::deque<TargetInfo> history_info_;
        int history_deque_lens_; //历史队列长度

        bool is_init;
    public:
        // set const value or default value
        PredictParam predict_param_;
        DebugParam debug_param_;

        std::string filter_param_path_;
        YAML::Node config_;
        
    public:
        TargetInfo final_target_;  //最终击打目标信息
        TargetInfo last_pf_target_; //最后一次粒子滤波后的位置结果

        int cnt;
        cv::Mat pic_x;
        cv::Mat pic_y;
        cv::Mat pic_z;
        
    public:
        ArmorPredictor(const PredictParam& predict_param, DebugParam& debug_param, std::string filter_param_path);
        
        Eigen::Vector3d predict(Eigen::Vector3d xyz, int timestamp);

        bool setBulletSpeed(double speed);
        Eigen::Vector3d shiftWindowFilter(int start_idx);

        PredictStatus predict_pf_run(TargetInfo target, Vector3d& result, int timestamp);
        PredictStatus uncouple_fitting_predict(Eigen::Vector3d& result, int timestamp);

        //移动轨迹拟合预测（小陀螺+横移->旋轮线）
        PredictStatus couple_fitting_predict(Eigen::Vector3d& result, int timestamp);
    
    protected:
        // 控制量
        SingerControl u;
        // Singer模型
        Singer singer;
        // 观测模型
        SingerPosModel pos_model;
        // EKF
        filter::ExtendKalmanFilter<SingerState> ekf;

        //
        bool is_ekf_init;
        PredictStatus predict_ekf_run(TargetInfo target, Eigen::Vector3d& result, int timestamp);
    };


} //namespace armor_processor

#endif