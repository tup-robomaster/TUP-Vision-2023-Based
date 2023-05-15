/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 11:28:53
 * @LastEditTime: 2023-05-14 14:08:42
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/prediction/prediction.hpp
 */
#ifndef PREDICTION_HPP_
#define PREDICTION_HPP_

#pragma once

//ros
#include <rclcpp/rclcpp.hpp>

//opencv
#include <opencv2/opencv.hpp>

//IMM Model(CV、CA、CT)
#include "../filter/model_generator.hpp"

#include "./param_struct.hpp"
#include "./curve_fitting.hpp"

#include "global_interface/msg/autoaim.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"

using namespace global_user;
using namespace coordsolver;
using namespace Eigen;
namespace armor_processor
{
    class ArmorPredictor
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef Vector<double, 6> Vector6d;
        
    public:
        ArmorPredictor();
        ~ArmorPredictor();
        // ArmorPredictor(const PredictParam& predict_param, vector<double>* singer_param, const DebugParam& debug_param);
        // Eigen::Vector3d predict(TargetInfo target, uint64_t timestamp, double& delay_time, cv::Mat* src = nullptr);
        // PostProcessInfo&& postProcess(AutoaimMsg& target_msg);

        void initPredictor();
        void initPredictor(const vector<double>* uniform_ekf_param, const vector<double>* singer_ekf_param);
        bool resetPredictor();
        bool updatePredictor(Eigen::VectorXd meas);
        bool predict(TargetInfo target, double dt, double pred_dt, double& delay_time, Eigen::Vector3d& pred_point3d, vector<Eigen::Vector4d>& armor3d_vec, cv::Mat* src = nullptr);

    public:
        PredictParam predict_param_;  //滤波先验参数/模型先验参数/调试参数
        DebugParam debug_param_;

    private:
        int history_deque_lens_ = 50; //历史队列长度
        std::deque<TargetInfo> history_info_; //历史测量信息
        std::deque<TargetInfo> history_pred_; //历史预测信息
        std::deque<TargetInfo> history_losting_pred_; //历史目标losting后预测信息
        
    private:
        double evalRMSE(double* params);
        double calcError();
        void updateVel(Eigen::Vector3d vel_3d);
        void updateAcc(Eigen::Vector3d acc_3d);

    public:
        double history_vel_[3][4] = {{0}};
        double history_acc_[3][4] = {{0}};
        double predict_vel_[3][4] = {{0}};
        double predict_acc_[3][4] = {{0}};
    
    public:
        // uniform ekf
        bool is_ekf_init_;
        UniformModel uniform_ekf_;

        // singer ekf
        bool is_singer_init_;
        SingerModel singer_ekf_;
        
        PredictorState predictor_state_ = LOST;
        Vector6d last_state_;
        Vector4d last_meas_;
        deque<Vector6d> history_switched_state_vec_;
        // deque<Vector6d> history_state_vec_;

    public:
        double now_ = 0.0;
        bool is_init_;
        bool is_imm_init_;
        bool fitting_disabled_; // 是否禁用曲线拟合
        bool filter_disabled_;  // 是否禁用滤波
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Logger logger_;
        TargetInfo final_target_;  //最终击打目标信息
        int lost_cnt_ = 0;
        int spin_switch_cnt_ = 0;
        double cur_rangle_ = 0.0;
        double last_rangle = 0.0;
        
    private:
        // IMM Model.
        std::shared_ptr<IMM> imm_;
        ModelGenerator model_generator_;
        bool predictBasedImm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, int64_t timestamp);
        
        // CS Model.
        // PredictStatus predictBasedSinger(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d target_vel, Eigen::Vector2d target_acc, int64_t timestamp);
        bool predictBasedSinger(bool is_target_lost, Eigen::Vector3d meas, Eigen::Vector3d& result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, double dt, double pred_dt);

        // Uniform Model.
        bool predictBasedUniformModel(bool is_target_lost, SpinHeading spin_state, Eigen::VectorXd meas, double dt, double pred_dt, double spinning_period, Eigen::Vector3d& result, vector<Eigen::Vector4d>& armor3d_vec);
        
        // 前哨站旋转装甲板曲线拟合预测函数    
        bool spinningPredict(bool is_controlled, TargetInfo& target, Eigen::Vector3d& result, int64_t timestamp);
        
        // 滑窗滤波
        Eigen::Vector3d shiftWindowFilter(int start_idx);

        // 计算车辆中心
        Eigen::Vector2d calcCircleCenter(Eigen::VectorXd meas);

        // 计算车辆半径
        double calcCircleRadius(Eigen::Vector3d p1, Eigen::Vector3d p2);
    };
} //namespace armor_processor

#endif