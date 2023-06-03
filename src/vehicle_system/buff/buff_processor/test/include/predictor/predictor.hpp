/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 17:09:18
 * @LastEditTime: 2023-06-01 16:26:35
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/test/include/predictor/predictor.hpp
 */
#ifndef PREDICTOR_HPP_
#define PREDICTOR_HPP_

# pragma once
//c++
#include <iostream>
#include <ctime>
#include <future>
#include <random>
#include <vector>

//opencv
#include <opencv2/opencv.hpp>

//ceres/eigen
#include <ceres/ceres.h>
#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

//ros
#include <rclcpp/rclcpp.hpp>

#include "../../../filter/include/particle_filter.hpp"
#include "../../../../global_user/include/global_user/global_user.hpp"
#include "../../../include/predictor/param_struct.hpp"

using namespace std;
using namespace cv;
using namespace filter;

namespace buff_processor
{
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
        std::deque<TargetInfo> history_info;                                    //目标队列
    
    private:
        double params[4] = {0.01, 0.01, 0.01, 0.01};
        rclcpp::Logger logger_;

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

} // namespace buff_processor

#endif