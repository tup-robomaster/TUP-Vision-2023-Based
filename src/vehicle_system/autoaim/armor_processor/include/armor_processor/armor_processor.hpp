/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2022-12-26 00:57:42
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor/armor_processor.hpp
 */
#ifndef ARMOR_PRECESSOR_HPP_
#define ARMOR_PROCESSOR_HPP_

#pragma once 

// #include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/target.hpp"
// #include "global_interface/msg/gimbal.hpp"
// #include "global_interface/msg/armor.hpp"
// #include "global_interface/msg/armors.hpp"

#include "../prediction/prediction.hpp"

// #include <Eigen/Core>
// #include <ceres/ceres.h>
// #include <glog/logging.h>
// #include <fmt/format.h>
// #include <yaml-cpp/yaml.h>

using namespace global_user;
// using namespace coordsolver;
namespace armor_processor
{
    class Processor : public ArmorPredictor
    {
        typedef geometry_msgs::msg::Point GeometryPoint;
        // typedef global_interface::msg::Target TargetMsg;
    public:
        // Processor(const PredictParam& predict_param, const SingerModelParam& singer_model_param, const DebugParam& debug_param, 
        //     const std::string& filter_param_path, const std::string& coord_param_path, 
        //     const std::string& coord_param_name);
        Processor();
        // Processor(const PredictParam& predict_param, const SingerModel& singer_model_param, const DebugParam& debug_param, 
        //     const std::string& filter_param_path, const std::string& coord_param_path, 
        //     const std::string& coord_param_name);
        ~Processor();

        //预测(接收armor_detector节点发布的目标信息进行预测)
        void predictor(TargetInfo& target);
        void predictor(TaskData& src, TargetInfo& target);

    private:
        // Eigen::Vector3d aiming_point_;
        // std::unique_ptr<ArmorPredictor> armor_predictor_;
        // GeometryPoint aiming_point_;

    public:
        // bool is_initialized_;
        // bool is_ekf_initialized_;
        // std::string coord_param_path_;
        // std::string coord_param_name_;
        // CoordSolver coordsolver_;
        // Eigen::Matrix3d rmat_imu;
    
    public:
        // Debug dynamically.
        void setPredictParam(int& param, int idx);
        void setDebugParam(bool& param, int idx);
        void setSingerParam(double& param, int idx);
    };
} //namespace armor_processor

#endif