/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2022-11-21 10:07:44
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

    Processor::Processor(const PredictParam& predict_param, const SingerModel& singer_model_param,
        const DebugParam& debug_param, const std::string& filter_param_path, 
        const std::string& coord_param_path, const std::string& coord_param_name)
    : armor_predictor_(predict_param, singer_model_param, debug_param, filter_param_path)
    {
        // if(!debug_param.using_imu)
        // {
        //     //TODO:暂时未使用陀螺仪数据
        //     rmat_imu = Eigen::Matrix3d::Identity();
        // }
        coord_param_path_ = coord_param_path;
        coord_param_name_ = coord_param_name;
        is_initialized = false;

        rmat_imu = Eigen::Matrix3d::Identity();
    }

    Processor::~Processor()
    {

    }

    void Processor::predictor(global_interface::msg::Target& target_info)
    {
        if(!is_initialized)
        {
            coordsolver_.loadParam(coord_param_path_, coord_param_name_);
            is_initialized = true;
        }

        Eigen::Vector3d target_;
        target_[0] = target_info.aiming_point.x;
        target_[1] = target_info.aiming_point.y;
        target_[2] = target_info.aiming_point.z;

        if(armor_predictor_.debug_param_.using_imu)
        {
            // rmat_imu[0] = target_info.rmat_imu.x;
            // rmat_imu[1] = target_info.rmat_imu.y;
            // rmat_imu[2] = target_info.rmat_imu.z;
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

        // if(armor_predictor_.debug_param_.show_predict)
        // {
        //     auto aiming_2d = coordsolver_.reproject(aiming_point_);
        //     // circle(src.img, aiming_2d, 2, {0, 255, 255}, 2);
        // }
    }

    void Processor::setMaxTimeDelta(int& max_time_delta)
    {
        armor_predictor_.predict_param_.max_time_delta = max_time_delta;
    }

    void Processor::setMinFittingLens(int& min_fitting_lens)
    {
        armor_predictor_.predict_param_.min_fitting_lens = min_fitting_lens;
    }

    void Processor::setMaxVelocity(int& max_v)
    {
        armor_predictor_.predict_param_.max_v = max_v;
    }

    void Processor::setShootDelay(int& shoot_delay)
    {
        armor_predictor_.predict_param_.shoot_delay = shoot_delay;
    }

    void Processor::setMaxCost(int& max_cost)
    {
        armor_predictor_.predict_param_.max_cost = max_cost;
    }

    void Processor::disabledFitting(bool& diabled_fitting)
    {
        armor_predictor_.debug_param_.disable_fitting = diabled_fitting;
    }

    void Processor::drawPredict(bool& draw_predict)
    {
        armor_predictor_.debug_param_.draw_predict = draw_predict;
    }

    void Processor::showPredict(bool& show_predict)
    {
        armor_predictor_.debug_param_.show_predict = show_predict;
    }

    void Processor::usingImu(bool& using_imu)
    {
        armor_predictor_.debug_param_.using_imu = using_imu;
    }

    void Processor::showTransformedInfo(bool& show_transformed_info)
    {
        armor_predictor_.debug_param_.show_transformed_info = show_transformed_info;
    }

    void Processor::set_alpha(double& alpha)
    {
        armor_predictor_.set_singer_alpha(alpha);
    }
    void Processor::set_a_max(double a_max)
    {
        armor_predictor_.set_singer_a_max(a_max);
    }

    void Processor::set_p_max(double& p_max)
    {
        armor_predictor_.set_singer_p_max(p_max);
    }

    void Processor::set_p0(double& p0)
    {
        armor_predictor_.set_singer_p0(p0);
    }

    void Processor::set_sigma(double& sigma)
    {
        armor_predictor_.set_singer_sigma(sigma);
    }

    void Processor::set_dt(double& dt)
    {
        armor_predictor_.set_singer_sigma(dt);
    }

    void Processor::set_p(double& p)
    {
        armor_predictor_.set_singer_sigma(p);
    }

    void Processor::set_r(double& r)
    {
        armor_predictor_.set_singer_sigma(r);
    }
} // armor_processor