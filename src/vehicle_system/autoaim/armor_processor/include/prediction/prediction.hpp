/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 11:28:53
 * @LastEditTime: 2023-04-15 19:54:56
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
        typedef global_interface::msg::Autoaim AutoaimMsg;
        
    public:
        ArmorPredictor();
        ~ArmorPredictor();
        ArmorPredictor(const PredictParam& predict_param, vector<double>* singer_param, 
            const PathParam& path_param, const DebugParam& debug_param);
    
    private:
        std::deque<TargetInfo> history_info_; //历史测量信息
        std::deque<TargetInfo> history_pred_; //历史预测信息
        int history_deque_lens_; //历史队列长度

    public:
        YAML::Node config_;
        PredictParam predict_param_;  //滤波先验参数/模型先验参数/调试参数
        DebugParam debug_param_;
        std::string filter_param_path_;
        
        bool setBulletSpeed(double speed);
        // void loadParam(std::string filter_param_path);
        Eigen::Vector3d predict(AutoaimMsg& target_msg, uint64 timestamp, double& sleep_time, cv::Mat* src = nullptr);
        PostProcessInfo&& postProcess(AutoaimMsg& target_msg);
        void curveDrawer(int axis, cv::Mat& src, double* params, cv::Point2i start_pos);

    private:
        std::deque<cv::Point2d> history_pred_info_;
        std::deque<cv::Point2d> history_origin_info_;
        std::deque<double> history_y_info_;
        int64_t last_start_timestamp_;
        double last_end_x_;
        bool is_predicted_;
        double target_period_;
        bool is_params_confirmed_;
        TargetInfo final_target_;  //最终击打目标信息
        double last_pred_dx = 0.0;

        //移动轨迹拟合预测（小陀螺+横移->旋轮线，若目标处于原地小陀螺状态，则剔除掉模型中的横移项）
        double fitting_params_[5] = {0.1, 0.1, 0.1, 0.1, 0.1};
    
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
        int error_cnt_ = 0;
        double cur_pred_error_;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Logger logger_;
    
    private:
        // IMM Model.
        std::shared_ptr<IMM> imm_;
        ModelGenerator model_generator_;
        PredictStatus predictBasedImm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector3d& target_vel, Eigen::Vector3d& target_acc, int64_t timestamp);
        
        // CS Model.
        // PredictStatus predictBasedSinger(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d target_vel, Eigen::Vector2d target_acc, int64_t timestamp);
        bool predictBasedSinger(bool is_spinning, int axis, double measurement, double& result, double target_vel, double target_acc, int64_t timestamp);

        // 前哨站旋转装甲板曲线拟合预测函数    
        PredictStatus spinningPredict(bool is_controlled, TargetInfo& target, Eigen::Vector3d& result, int64_t timestamp);
        
        // 粒子滤波
        // PredictStatus predictBasePF(TargetInfo target, Vector3d& result, int64_t timestamp);
        // PredictStatus uncoupleFittingPredict(Eigen::Vector3d& result, int64_t timestamp);
        PredictStatus coupleFittingPredict(bool is_still_spinning, TargetInfo target, Eigen::Vector3d& result, int64_t timestamp);
        
        // 滑窗滤波
        Eigen::Vector3d shiftWindowFilter(int start_idx);
    };
} //namespace armor_processor

#endif