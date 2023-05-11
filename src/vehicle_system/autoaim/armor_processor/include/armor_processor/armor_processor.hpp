/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2023-04-26 22:03:50
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor/armor_processor.hpp
 */
#ifndef ARMOR_PRECESSOR_HPP_
#define ARMOR_PROCESSOR_HPP_

#pragma once 

#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/obj_hp.hpp"
#include "../prediction/prediction.hpp"

using namespace global_user;
using namespace coordsolver;
namespace armor_processor
{
    class Processor
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef global_interface::msg::ObjHP ObjHPMsg;

    public:
        Processor();
        Processor(const PredictParam& predict_param, vector<double>* uniform_ekf_param, vector<double>* singer_ekf_param, const DebugParam& debug_param);
        ~Processor();

        // std::unique_ptr<Eigen::Vector3d> predictor(cv::Mat& src, AutoaimMsg& Autoaim, double& sleep_time);
        // void loadParam(std::string filter_param_path);

        //预测(接收armor_detector节点发布的目标信息进行预测)
        CoordSolver coordsolver_;
        void init(std::string coord_path, std::string coord_name);
        bool predictor(AutoaimMsg& Autoaim, Eigen::Vector3d& pred_result, vector<Eigen::Vector4d>& armor3d_vec, double& sleep_time);

        void curveDrawer(int axis, cv::Mat& src, double* params, cv::Point2i start_pos);
        // bool autoShootingLogic(AutoaimMsg& armor, PostProcessInfo& post_process_info);
        // bool setBulletSpeed(double speed);
    
        int lost_cnt_ = 0;
        bool is_filter_ = true;
        bool is_fitting_ = false;
        bool is_last_exists_ = false;
        bool is_param_initialized_ = false;
        rclcpp::Time last_timestamp_;
        // double target_period_ = 0.0;
        TargetInfo last_target_;
        
        //预测器(每辆车默认分配4个预测器，平衡仅使用2个预测器)
        //目前只分配一个预测器
        ArmorPredictor armor_predictor_;

        PredictParam predict_param_;
        DebugParam debug_param_;
        
    private:
        std::map<std::string, int> car_id_map_;
        rclcpp::Logger logger_;
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    };
} //namespace armor_processor

#endif