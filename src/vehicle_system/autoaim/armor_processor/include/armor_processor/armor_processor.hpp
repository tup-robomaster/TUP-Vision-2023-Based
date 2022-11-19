/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2022-11-19 12:51:45
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor/armor_processor.hpp
 */
#ifndef ARMOR_PRECESSOR_HPP
#define ARMOR_PROCESSOR_HPP

#pragma once 

// #include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/target.hpp"
// #include "global_interface/msg/gimbal.hpp"
// #include "global_interface/msg/armor.hpp"
// #include "global_interface/msg/armors.hpp"

#include "../prediction/prediction.h"

// #include <Eigen/Core>
// #include <ceres/ceres.h>
// #include <glog/logging.h>
// #include <fmt/format.h>
// #include <yaml-cpp/yaml.h>

typedef geometry_msgs::msg::Point GeometryPoint;

namespace armor_processor
{
    class Processor
    {
    public:
        Processor(const PredictParam& predict_param, const SingerModelParam& singer_model_param, const DebugParam& debug_param, 
            const std::string& filter_param_path, const std::string& coord_param_path, 
            const std::string& coord_param_name);
        ~Processor();

        //预测(接收armor_detector节点发布的目标信息进行预测)
        void predictor(global_interface::msg::Target& target_info);

    private:
        Eigen::Vector3d aiming_point_;
        // GeometryPoint aiming_point_;

    public:
        std::string coord_param_path_;
        std::string coord_param_name_;
        bool is_initialized;
        ArmorPredictor armor_predictor_;
        coordsolver::coordsolver coordsolver_;

        Eigen::Matrix3d rmat_imu;
    
    public:
        // Debug dynamically.
        void setMaxTimeDelta(int& max_time_delta);
        void setMinFittingLens(int& min_fitting_lens);
        void setMaxVelocity(int& max_v);
        void setShootDelay(int& shoot_delay);
        void setMaxCost(int& max_cost);
        void disabledFitting(bool& diabled_fitting);
        void drawPredict(bool& draw_predict);
        void showPredict(bool& show_predict);
        void usingImu(bool& using_imu);
        void showTransformedInfo(bool& show_transformed_info);
        void set_alpha(double& alpha);
        void set_a_max(double a_max);
        void set_p_max(double& p_max);
        void set_p0(double& p0);
        void set_sigma();
    };
} //namespace armor_processor

#endif