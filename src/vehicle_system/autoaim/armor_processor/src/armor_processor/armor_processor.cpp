/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2023-02-05 00:50:19
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

    Processor::Processor(const PredictParam& predict_param, const vector<double>& singer_model_param, const PathParam& path_param, const DebugParam& debug_param)
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

    /**
     * @brief 从外部加载坐标解算类参数
     * 
     * @param coord_path 相机标定参数文件路径
     * @param coord_name 相机型号名
     */
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

    /**
     * @brief 对目标装甲板的位置进行预测
     * 
     * @param target 目标装甲板message信息
     * @param sleep_time 休眠时间，对应预测延迟时间，用于改变预测点message的时间戳从而方便观察测量值与预测量间的误差
     * @return std::unique_ptr<Eigen::Vector3d> 
     */
    std::unique_ptr<Eigen::Vector3d> Processor::predictor(AutoaimMsg& target, double& sleep_time)
    {
        if(target.target_switched)
        {
            is_ekf_init = false;
            is_imm_init = false;
            // reInitialize();
        }

        auto hit_point = predict(target, target.timestamp, sleep_time);
        return std::make_unique<Eigen::Vector3d>(hit_point);
    }
    
    /**
     * @brief 同上
     * 
     * @param src 传入图像信息，方便可视化
     * @param target 
     * @param sleep_time 
     * @return std::unique_ptr<Eigen::Vector3d> 
     */
    std::unique_ptr<Eigen::Vector3d> Processor::predictor(cv::Mat& src, AutoaimMsg& target, double& sleep_time)
    {
        if(target.target_switched)
        {
            is_ekf_init = false;
            is_imm_init = false;
            // reInitialize();
        }

        auto hit_point = predict(target, target.timestamp, sleep_time, &src);
        return std::make_unique<Eigen::Vector3d>(hit_point);
    }
    
    void Processor::setPredictParam(int& param, int idx)
    {
        set_predict_param(std::move(param), idx);
    }

    void Processor::setDebugParam(bool& param, int idx)
    {
        set_debug_param(std::move(param), idx);
    }

    void Processor::setSingerParam(vector<double>& singer_param)
    {
        set_singer_param(std::move(singer_param));
    }
    // void Processor::setSingerParam(double& param, int idx)
    // {
    //     set_singer_param(std::move(param), idx);
    // }

    void Processor::setImmParam(IMMParam& imm_param)
    {
        set_imm_param(std::move(imm_param));
    }
    // void Processor::setImmParam(double& param, int idx)
    // {
    //     set_imm_param(std::move(param), idx);
    // }
} // armor_processor