/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2022-10-27 19:53:52
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor/armor_processor.cpp
 */
#include "../../include/armor_processor/armor_processor.hpp"

namespace armor_processor
{
    Processor::Processor(const PredictParam& predict_param, DebugParam& debug_param, std::string coord_file)
    : armor_predictor_(predict_param, debug_param, coord_file)
    {
        if(!debug_param.using_imu)
        {
            //TODO:暂时未使用陀螺仪数据
            rmat_imu = Eigen::Matrix3d::Identity();
        }
    }

    Processor::~Processor()
    {

    }

    void Processor::predictor(global_interface::msg::Target& target_info)
    {
        Eigen::Vector3d target_;
        target_[0] = target_info.aiming_point.x;
        target_[1] = target_info.aiming_point.y;
        target_[2] = target_info.aiming_point.z;

        if(armor_predictor_.debug_param_.using_imu)
        {
            rmat_imu[0] = target_info.rmat_imu.x;
            rmat_imu[1] = target_info.rmat_imu.y;
            rmat_imu[2] = target_info.rmat_imu.z;
        }

        if(target_info.target_switched)
        {
            aiming_point_ = target_;
        }
        else
        {
            auto aiming_point_world = armor_predictor_.predict(target_, target_info.timestamp);
            aiming_point_ = coordsolver_.worldToCam(aiming_point_world, rmat_imu);
        }
        
        if(armor_predictor_.debug_param_.show_predict)
        {
            auto aiming_2d = coordsolver_.reproject(aiming_point_);
            // circle(src.img, aiming_2d, 2, {0, 255, 255}, 2);
        }
    }

} // armor_processor