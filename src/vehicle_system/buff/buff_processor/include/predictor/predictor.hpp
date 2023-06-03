/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 17:09:18
 * @LastEditTime: 2023-06-03 23:59:58
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/include/predictor/predictor.hpp
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
    public:
        BuffPredictor();
        ~BuffPredictor();
        
        bool curveFitting(BuffMsg& buff_msg);
        bool predict(BuffMsg buff_msg, BuffInfo& buff_info, double &result);
        double calPreAngle(double* params, double timestamp);
        bool setBulletSpeed(double speed);
        double evalRMSE(double params[4]);

    public:
        int mode_;                    //预测器模式，0为小符，1为大符
        int last_mode_;
        bool is_params_confirmed_;
        
        ParticleFilter pf_;
        BuffAngleInfo last_target_;              //最后目标
        ParticleFilter pf_param_loader_;
        PredictorParam predictor_param_;
        std::deque<BuffAngleInfo> history_info_; //目标队列
        bool is_direction_confirmed_;
        deque<double> delta_angle_vec_;
        queue<PredInfo> pred_info_queue_;

        double base_angle_;
        int sign_;
        double angle_offset_;
        bool is_switched_;
        double last_angle_offset_;
        std::deque<BuffAngleInfo> predict_info_;
        atomic<double> last_phase_;
        atomic<double> phase_;
        
        int error_cnt_;
        double cur_pred_angle_;
        double last_pred_angle_;

        int rmse_error_cnt_;
        double ave_speed_;
        bool is_last_result_exist_;
        int lost_cnt_;
        
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    private:
        double params_[4] = {0.1, 0.1, 0.1, 0.1};

        rclcpp::Logger logger_;
    };
} // namespace buff_processor

#endif