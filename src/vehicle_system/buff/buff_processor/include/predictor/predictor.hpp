/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 17:09:18
 * @LastEditTime: 2022-12-11 20:49:25
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/include/predictor/predictor.hpp
 */
#ifndef PREDICTOR_HPP_
#define PREDICTOR_HPP_

# pragma once
#include <iostream>

#include <ctime>
#include <future>
#include <random>
#include <vector>

#include <ceres/ceres.h>
#include <Eigen/Core>
// #include <matplotlibcpp.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "../../../filter/include/particle_filter.hpp"
#include "../../../global_user/include/global_user/global_user.hpp"

using namespace std;
using namespace cv;
using namespace filter;

namespace buff
{
    struct PredictorParam
    {
        string pf_path;
        double bullet_speed;
        double max_timespan;            //最大时间跨度，大于该时间重置预测器(ms)
        double max_rmse;                //TODO:回归函数最大Cost
        double max_v;                   //设置最大速度,单位rad/s
        double max_a;                   //设置最大角加速度,单位rad/s^2
        int history_deque_len_cos;      //大符全部参数拟合队列长度
        int history_deque_len_phase;    //大符相位参数拟合队列长度
        int history_deque_len_uniform;  //小符转速求解队列长度
        double delay_small;             //小符发弹延迟
        double delay_big;               //大符发弹延迟
        int window_size;                //滑动窗口大小

        PredictorParam()
        {
            pf_path = "src/global_user/config/filter_param.yaml";
            bullet_speed = 28.0;
            max_timespan = 20000;       
            max_rmse = 0.5;
            max_v = 3.0;
            max_a = 8.0;
            history_deque_len_cos = 250;
            history_deque_len_phase = 100;
            history_deque_len_uniform = 100;
            delay_small = 175.0;
            delay_big = 100.0;
            window_size = 2;
        }     
    };


    class BuffPredictor
    {
    private:
        struct CURVE_FITTING_COST
        {
            CURVE_FITTING_COST (double x, double t)
            : _x (x), _t (t) {}
            
            // 残差的计算
            template <typename T>
            bool operator() 
            (
                const T* params, // 模型参数，有3维
                T* residual      // 残差
            ) const             
            {
                residual[0] = T (_x) - params[0] * ceres::sin(params[1] * T(_t) + params[2]) - params[3]; // f(t) = a * sin(ω * t + θ) + b
                return true;
            }
            const double _x, _t;    // x,t数据
        };

        struct CURVE_FITTING_COST_PHASE
        {
            CURVE_FITTING_COST_PHASE (double x, double t, double a, double omega, double dc)
            : _x(x), _t(t), _a(a), _omega(omega), _dc(dc){}

            // 残差的计算
            template <typename T>
            bool operator()
            (
                const T* phase, // 模型参数，有1维
                T* residual     // 残差
            ) const 
            {
                residual[0] = T (_x) - T (_a) * ceres::sin(T(_omega) * T (_t) + phase[0]) - T(_dc); // f(x) = a * sin(ω * t + θ)
                return true;
            }
            const double _x, _t, _a, _omega, _dc;    // x,t数据
        };

        //目标信息
        struct TargetInfo
        {
            double speed;
            double dist;
            double timestamp;
        };

        struct PredictStatus
        {
            bool xyz_status[3];
        };

    public:
        PredictorParam predictor_param_;
    
    private:
        double params[4];
        std::deque<TargetInfo> history_info;                                    //目标队列

    public:
        TargetInfo last_target;                                                  //最后目标
        ParticleFilter pf;
        ParticleFilter pf_param_loader;
        int mode;                                                               //预测器模式，0为小符，1为大符
        int last_mode;
        bool is_params_confirmed;

        BuffPredictor();
        ~BuffPredictor();
        bool predict(double speed, double dist, double timestamp, double &result);
        double calcAimingAngleOffset(double params[4], double t0, double t1, int mode);
        double shiftWindowFilter(int start_idx);
        bool setBulletSpeed(double speed);
        double evalRMSE(double params[4]);
        double evalMAPE(double params[4]);
    };

} // namespace buff

#endif