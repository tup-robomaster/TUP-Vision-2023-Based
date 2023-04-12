/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 17:09:18
 * @LastEditTime: 2023-03-20 19:58:46
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
#include <mutex>
#include <atomic>
#include <thread>

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
#include "global_interface/msg/buff.hpp"
#include "./param_struct.hpp"

using namespace std;
using namespace cv;
using namespace filter;
using namespace global_user;
using namespace global_interface;
namespace buff_processor
{
    class BuffPredictor
    {
        typedef global_interface::msg::Buff BuffMsg;
    private:
        struct TargetInfo
        {
            // double dist;
            bool is_switched;
            double abs_angle;
            double relative_angle;
            double delta_angle;
            double angle_offset;
            double timestamp;
        };

    public:
        atomic<int> mode;                    //预测器模式，0为小符，1为大符
        atomic<int> last_mode;
        atomic<bool> is_params_confirmed;
        ParticleFilter pf;
        TargetInfo last_target;              //最后目标
        ParticleFilter pf_param_loader;
        PredictorParam predictor_param_;
        std::deque<TargetInfo> history_info; //目标队列
        bool is_direction_confirmed;
        vector<double> delta_angle_vec_;
        queue<PredInfo> pred_info_queue_;

        double base_angle_;
        atomic<int> sign_;
        double angle_offset_;
        bool is_switched_;
        double last_angle_offset_;
        std::deque<TargetInfo> predict_info_;
        atomic<double> last_phase_;
        atomic<double> phase_;
        double origin_timestamp_;
        int error_cnt_;
        double last_result_;
        double last_last_result_;
        double last_last_last_result_;

        double cur_pred_angle_;
        double last_pred_angle_;

        int rmse_error_cnt_;
        double ave_speed_;
        bool is_last_result_exist_;
        int lost_cnt_;
    private:
        double params[4] = {0.1, 0.1, 0.1, 0.1};
        rclcpp::Logger logger_;
        Mutex mutex_;

    public:
        BuffPredictor();
        ~BuffPredictor();
        
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        bool curveFitting(BuffMsg& buff_msg);
        bool predict(BuffMsg buff_msg, double dist, double &result);
        double calPreAngle(double* params, double timestamp);
        bool setBulletSpeed(double speed);
        double evalRMSE(double params[4]);

        // double evalMAPE(double params[4]);
        // double shiftWindowFilter(int start_idx);
        // bool predict(double speed, double dist, double timestamp, double &result);
        // double calcAimingAngleOffset(double params[4], double t0, double t1, int mode);
    };
} // namespace buff_processor

#endif