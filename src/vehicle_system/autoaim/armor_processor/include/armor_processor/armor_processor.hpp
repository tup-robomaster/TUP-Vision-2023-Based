/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2023-03-12 11:02:32
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
    class Processor : public ArmorPredictor
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef global_interface::msg::ObjHP ObjHPMsg;

    public:
        Processor();
        Processor(const PredictParam& predict_param, vector<double>* singer_model_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Processor();

        //预测(接收armor_detector节点发布的目标信息进行预测)
        CoordSolver coordsolver_;
        std::unique_ptr<Eigen::Vector3d> predictor(AutoaimMsg& Autoaim, double& sleep_time);
        std::unique_ptr<Eigen::Vector3d> predictor(cv::Mat& src, AutoaimMsg& Autoaim, double& sleep_time);
        
        void init(std::string coord_path, std::string coord_name);
        bool autoShootingLogic(AutoaimMsg& armor, PostProcessInfo& post_process_info);
    
    private:
        PathParam path_param_;
        std::map<std::string, int> car_id_map_;
    };
} //namespace armor_processor

#endif