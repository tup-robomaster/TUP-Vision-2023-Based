/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2023-01-06 21:51:07
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

    Processor::Processor(const PredictParam& predict_param, const SingerModel& singer_model_param, const PathParam& path_param, const DebugParam& debug_param)
    : ArmorPredictor(predict_param, singer_model_param, path_param, debug_param), path_param_(path_param)
    {
        is_init = false;
        is_imm_init = false;
        is_ekf_init = false;
        // init(path_param.coord_path, path_param.coord_name);
    }
    
    Processor::Processor()
    : ArmorPredictor()
    {
        // PathParam path;
        // init(path.coord_path, path.coord_name);
    }

    // Processor::Processor(const PredictParam& predict_param, const SingerModel& singer_model_param,
    //     const DebugParam& debug_param, const std::string& filter_param_path, 
    //     const std::string& coord_param_path, const std::string& coord_param_name)
    // {
    //     predict_param_ = predict_param;
    //     singer_param_ = singer_model_param;
    //     filter_param_path_ = filter_param_path;
    //     // if(!debug_param.using_imu)
    //     // {
    //     //     //TODO:暂时未使用陀螺仪数据
    //     //     rmat_imu = Eigen::Matrix3d::Identity();
    //     // }
    //     coord_param_path_ = coord_param_path;
    //     coord_param_name_ = coord_param_name;
    //     is_initialized_ = false;

    //     // rmat_imu = Eigen::Matrix3d::Identity();
    // }

    Processor::~Processor()
    {
        
    }

    void Processor::init(std::string coord_path, std::string coord_name)
    {
        try
        {
            auto success = coordsolver_.loadParam(coord_path, coord_name);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    std::unique_ptr<Eigen::Vector3d> Processor::predictor(AutoaimMsg& target, double& sleep_time)
    {
        if(target.target_switched)
        {
            is_ekf_init = false;
            is_imm_init = false;
            reInitialize();
        }

        auto hit_point = predict(target, target.timestamp, sleep_time);
        return std::make_unique<Eigen::Vector3d>(hit_point);
    }
    
    std::unique_ptr<Eigen::Vector3d> Processor::predictor(cv::Mat& src, AutoaimMsg& target, double& sleep_time)
    {
        if(target.target_switched)
        {
            is_ekf_init = false;
            is_imm_init = false;
            reInitialize();
        }

        auto hit_point = predict(target, target.timestamp, sleep_time, &src);
        return std::make_unique<Eigen::Vector3d>(hit_point);
    }

    void Processor::setPredictParam(int& param, int idx)
    {
        switch (idx)
        {
        case 1:
            predict_param_.max_delta_time = param;
            break;
        case 2:
            predict_param_.min_fitting_lens = param;
            break;
        case 3:
            predict_param_.max_v = param;
            break;
        case 4:
            predict_param_.shoot_delay = param;
            break;
        case 5:
            predict_param_.max_cost = param;
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
            debug_param_.disable_fitting = param;
            break;
        case 2:
            debug_param_.disable_filter = param;
            break;
        case 3:
            debug_param_.draw_predict = param;
            break;
        case 4:
            debug_param_.show_predict = param;
            break;
        case 5:
            debug_param_.show_transformed_info = param;
            break;
        case 6:
            debug_param_.using_imu = param;
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
            setSingerParam(param, 1);
            break;
        case 2:
            setSingerParam(param, 2);
            break;
        case 3:
            setSingerParam(param, 3);
            break;
        case 4:
            setSingerParam(param, 4);
            break;
        case 5:
            setSingerParam(param, 5);
            break;
        case 6:
            setSingerParam(param, 6);
            break;
        case 7:
            setSingerParam(param, 7);
            break;
        case 8:
            setSingerParam(param, 8);
            break;
        default:
            break;
        }
    }
} // armor_processor