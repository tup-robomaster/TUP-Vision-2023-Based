/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2022-12-26 00:56:26
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor/armor_processor.cpp
 */
#include "../../include/armor_processor/armor_processor.hpp"

namespace armor_processor
{
    // Processor::Processor(const PredictParam& predict_param, const SingerModelParam& singer_model_param,
    //     const DebugParam& debug_param, const std::string& filter_param_path, 
    //     const std::string& coord_param_path, const std::string& coord_param_name)
    // : armor_predictor_(predict_param, singer_model_param, debug_param, filter_param_path)
    // {
    //     // if(!debug_param.using_imu)
    //     // {
    //     //     //TODO:暂时未使用陀螺仪数据
    //     //     rmat_imu = Eigen::Matrix3d::Identity();
    //     // }
    //     coord_param_path_ = coord_param_path;
    //     coord_param_name_ = coord_param_name;
    //     is_initialized = false;

    //     rmat_imu = Eigen::Matrix3d::Identity();
    // }
    Processor::Processor()
    : is_init(false), is_ekf_init(false), is_imm_init(false)
    {
        
    }

    Processor::Processor(const PredictParam& predict_param, const SingerModel& singer_model_param,
        const DebugParam& debug_param, const std::string& filter_param_path, 
        const std::string& coord_param_path, const std::string& coord_param_name)
    {
        armor_predictor_->predict_param_ = predict_param;
        armor_predictor_->singer_param_ = singer_model_param;
        armor_predictor_->filter_param_path_ = filter_param_path;
        // if(!debug_param.using_imu)
        // {
        //     //TODO:暂时未使用陀螺仪数据
        //     rmat_imu = Eigen::Matrix3d::Identity();
        // }
        coord_param_path_ = coord_param_path;
        coord_param_name_ = coord_param_name;
        is_initialized_ = false;

        // rmat_imu = Eigen::Matrix3d::Identity();
    }

    Processor::~Processor()
    {

    }
    
    void Processor::predictor(TargetInfo& target)
    {
        
    }
    
    void Processor::predictor(TaskData& src, TargetInfo& target_info)
    {
        // if(!is_initialized)
        // {
        //     coordsolver_.loadParam(coord_param_path_, coord_param_name_);
        //     is_initialized = true;
        // }
        if(!is_ekf_initialized_)
        {
            armor_predictor_->is_ekf_init = false;
        }

        // Eigen::Vector3d target_ = {target_info.xyz[0], target_info.aiming_point.y, target_[2] = target_info.aiming_point.z};
        // if(armor_predictor_.debug_param_.using_imu)
        // {
        //     rmat_imu[0] = target_info.rmat_imu.x;
        //     rmat_imu[1] = target_info.rmat_imu.y;
        //     rmat_imu[2] = target_info.rmat_imu.z;
        // }
        double sleep_time = 0;
        if(target_info.is_target_switched)
        {
            aiming_point_ = target_info.xyz;
        }
        else
        {
            auto aiming_point_world = armor_predictor_->predict(target_info, src.timestamp, sleep_time, &src.img);
            // aiming_point_ = coordsolver_.worldToCam(aiming_point_world, rmat_imu);
        }

        // if(armor_predictor_.debug_param_.show_predict)
        // {
        //     auto aiming_2d = coordsolver_.reproject(aiming_point_);
        //     // circle(src.img, aiming_2d, 2, {0, 255, 255}, 2);
        // }
    }

    void Processor::setPredictParam(int& param, int idx)
    {
        switch (idx)
        {
        case 1:
            armor_predictor_->predict_param_.max_delta_time = param;
            break;
        case 2:
            armor_predictor_->predict_param_.min_fitting_lens = param;
            break;
        case 3:
            armor_predictor_->predict_param_.max_v = param;
            break;
        case 4:
            armor_predictor_->predict_param_.shoot_delay = param;
            break;
        case 5:
            armor_predictor_->predict_param_.max_cost = param;
            break;
        default:
            break;
        }
    }

    void Processor::setDebugParam(bool& param, int idx)
    {
        switch (idx)
        {
        case 1:
            armor_predictor_->debug_param_.disable_fitting = param;
            break;
        case 2:
            armor_predictor_->debug_param_.disable_filter = param;
            break;
        case 3:
            armor_predictor_->debug_param_.draw_predict = param;
            break;
        case 4:
            armor_predictor_->debug_param_.show_predict = param;
            break;
        case 5:
            armor_predictor_->debug_param_.show_transformed_info = param;
            break;
        case 6:
            armor_predictor_->debug_param_.using_imu = param;
            break;
        default:
            break;
        }
    }

    void Processor::setSingerParam(double& param, int idx)
    {
        switch (idx)
        {
        case 1:
            armor_predictor_->setSingerParam(param, 1);
            break;
        case 2:
            armor_predictor_->setSingerParam(param, 2);
            break;
        case 3:
            armor_predictor_->setSingerParam(param, 3);
            break;
        case 4:
            armor_predictor_->setSingerParam(param, 4);
            break;
        case 5:
            armor_predictor_->setSingerParam(param, 5);
            break;
        case 6:
            armor_predictor_->setSingerParam(param, 6);
            break;
        case 7:
            armor_predictor_->setSingerParam(param, 7);
            break;
        case 8:
            armor_predictor_->setSingerParam(param, 8);
            break;
        default:
            break;
        }
    }
} // armor_processor