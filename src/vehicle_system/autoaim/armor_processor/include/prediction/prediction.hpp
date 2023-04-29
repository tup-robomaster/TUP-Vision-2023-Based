/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 11:28:53
 * @LastEditTime: 2023-04-26 22:04:55
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/prediction/prediction.hpp
 */
#ifndef PREDICTION_HPP_
#define PREDICTION_HPP_

#pragma once

//ros
#include <rclcpp/rclcpp.hpp>

//opencv
#include <opencv2/opencv.hpp>

//Singer Model
#include "../filter/singer_model.hpp"
//IMM Model(CV、CA、CT)
#include "../filter/model_generator.hpp"

#include "./param_struct.hpp"
#include "../filter/particle_filter.hpp"
#include "./curve_fitting.hpp"

#include "global_interface/msg/autoaim.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"

using namespace global_user;
using namespace coordsolver;
namespace armor_processor
{
    class ArmorPredictor
    {
        typedef global_interface::msg::Armor ArmorMsg;
        typedef global_interface::msg::Autoaim AutoaimMsg;
        
    public:
        ArmorPredictor();
        ~ArmorPredictor();
        // ArmorPredictor(const PredictParam& predict_param, vector<double>* singer_param, const DebugParam& debug_param);

        void initPredictor(const vector<double>* uniform_param);
        bool resetPredictor();
        bool updatePredictor(Eigen::VectorXd meas);
        bool predict(TargetInfo target, double bullet_speed, double dt, double& delay_time, Eigen::Vector3d& pred_point3d, vector<Eigen::Vector4d>& armor3d_vec, cv::Mat* src = nullptr);
        
        // Eigen::Vector3d predict(TargetInfo target, uint64_t timestamp, double& delay_time, cv::Mat* src = nullptr);
        // bool asyncPrediction(bool is_filtering, bool is_target_lost, bool is_spinning, Eigen::Vector3d meas, int64_t timestamp, Eigen::Vector3d& result);
        // PostProcessInfo&& postProcess(AutoaimMsg& target_msg);

    public:
        PredictParam predict_param_;  //滤波先验参数/模型先验参数/调试参数
        DebugParam debug_param_;

    private:
        int history_deque_lens_; //历史队列长度
        std::deque<TargetInfo> history_info_; //历史测量信息
        std::deque<TargetInfo> history_pred_; //历史预测信息
        std::deque<TargetInfo> history_losting_pred_; //历史目标losting后预测信息
        
    private:
        double evalRMSE(double* params);
        double calcError();
        void updateVel(bool is_spinning, Eigen::Vector3d vel_3d);
        void updateAcc(bool is_spinning, Eigen::Vector3d acc_3d);

    private:
        double history_vel_[2][3][4] = {{{0}}};
        double history_acc_[2][3][4] = {{{0}}};
        double predict_vel_[2][3][4] = {{{0}}};
        double predict_acc_[2][3][4] = {{{0}}};
    
    public:
        // ekf
        bool is_ekf_init_;
        UniformModel uniform_ekf_;
        PredictorState predictor_state_ = LOST;

    public:
        // 卡尔曼滤波
        bool is_singer_init_[2][3];
        vector<double> singer_param_[2][3]; //cs模型参数 
        SingerModel singer_model_[2][3];
        KalmanFilter singer_kf_[2][3];
        
    private:
        void kfInit();                      // 滤波参数初始化（矩阵维度、初始值）
        void kfInit(int axis);              // 滤波参数初始化（矩阵维度、初始值）

    public:
        bool is_init_;
        bool is_imm_init_;
        bool fitting_disabled_; // 是否禁用曲线拟合
        bool filter_disabled_;  // 是否禁用滤波
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Logger logger_;
        TargetInfo final_target_;  //最终击打目标信息
        int lost_cnt_ = 0;
        double delay_time_ = 200;
        // double target_period_ = 0.0;
        // int error_cnt_ = 0;
        // double cur_pred_error_;
    
    private:
        // IMM Model.
        std::shared_ptr<IMM> imm_;
        ModelGenerator model_generator_;
        PredictStatus predictBasedImm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, int64_t timestamp);
        
        // CS Model.
        // PredictStatus predictBasedSinger(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d target_vel, Eigen::Vector2d target_acc, int64_t timestamp);
        bool predictBasedSinger(bool is_target_lost, bool is_spinning, int axis, double measurement, double& result, double target_vel, double target_acc, int64_t timestamp);

        // Uniform Model.
        bool predictBasedUniformModel(bool is_target_lost, Eigen::VectorXd meas, double dt, double pred_dt, double spinning_period, Eigen::Vector3d& result, vector<Eigen::Vector4d>& armor3d_vec);
        
        // 前哨站旋转装甲板曲线拟合预测函数    
        PredictStatus spinningPredict(bool is_controlled, TargetInfo& target, Eigen::Vector3d& result, int64_t timestamp);
        
        // 滑窗滤波
        Eigen::Vector3d shiftWindowFilter(int start_idx);

        Eigen::Vector2d calcCircleCenter(Eigen::VectorXd meas);

        // 粒子滤波
        // PredictStatus predictBasePF(TargetInfo target, Vector3d& result, int64_t timestamp);
        // PredictStatus uncoupleFittingPredict(Eigen::Vector3d& result, int64_t timestamp);
        // PredictStatus coupleFittingPredict(bool is_still_spinning, TargetInfo target, Eigen::Vector3d& result, int64_t timestamp);
    };
} //namespace armor_processor

#endif