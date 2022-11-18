/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2022-11-18 21:54:51
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
        Processor(const PredictParam& predict_param, DebugParam& debug_param, std::string filter_param_path, std::string coord_param_path, std::string coord_param_name);
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
        void setMaxTimeDelta(int& max_time_delta);
        void setMinFittingLens(int& min_fitting_lens);
        void setMaxVelocity(int& max_v);
        void setShootDelay(int& shoot_delay);
    };
} //namespace armor_processor

#endif