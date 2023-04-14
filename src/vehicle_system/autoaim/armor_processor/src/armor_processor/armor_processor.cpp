/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2023-04-14 14:09:04
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor/armor_processor.cpp
 */
#include "../../include/armor_processor/armor_processor.hpp"

namespace armor_processor
{
    Processor::Processor(const PredictParam& predict_param, vector<double>* singer_model_param, const PathParam& path_param, const DebugParam& debug_param)
    : ArmorPredictor(predict_param, singer_model_param, path_param, debug_param), path_param_(path_param)
    {
        is_init_ = false;
        is_imm_init_ = false;
        // is_ekf_init = false;
        is_singer_init_[0][0] = false;
        is_singer_init_[0][1] = false;
        is_singer_init_[0][2] = false;
        is_singer_init_[1][0] = false;
        is_singer_init_[1][1] = false;
        is_singer_init_[1][2] = false;

        car_id_map_ = {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };
        // init(path_param.coord_path, path_param.coord_name);
    }
    
    Processor::Processor()
    : ArmorPredictor()
    {
        is_init_ = false;
        is_imm_init_ = false;
        // is_ekf_init = false;
        is_singer_init_[0][0] = false;
        is_singer_init_[0][1] = false;
        is_singer_init_[0][2] = false;
        is_singer_init_[1][0] = false;
        is_singer_init_[1][1] = false;
        is_singer_init_[1][2] = false;

        car_id_map_ = {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };
        // PathParam path;
        // init(path.coord_path, path.coord_name);
    }

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
            success = coordsolver_.setStaticAngleOffset(predict_param_.angle_offset);
            is_init_ = true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Error while initializing: %s", e.what());
        }
    }

    /**
     * @brief 自动发弹逻辑函数
     * 
     * @param armor 目标装甲信息
     * @param hp 车辆血量信息
     * @return true 
     * @return false 
     */
    bool Processor::autoShootingLogic(AutoaimMsg& armor, PostProcessInfo& post_process_info)
    {
        // post_process_info = postProcess(armor);
    
        // // 如果当前目标血量偏低直接发弹
        // if (armor.hp <= 75)
        // {
        //     post_process_info.find_target = true;
        //     post_process_info.is_shooting = true;
        //     post_process_info.switch_target = false;    
        // } 
        // else if (post_process_info.track_3d_pos.norm() <= 4.5 && post_process_info.hp <= 200)
        // {
        //     post_process_info.find_target = true;
        //     post_process_info.is_shooting = true;
        //     post_process_info.switch_target = false;
        // }
        // else if (post_process_info.track_3d_pos.norm() <= 2.5 && post_process_info.hp <= 500)
        // {
        //     post_process_info.find_target = true;
        //     post_process_info.is_shooting = true;
        //     post_process_info.switch_target = false;
        // }
        // else
        // {
        //     return false;
        // }
        PostProcessInfo post_info = PostProcessInfo();
        double sleep_time = 0.0;
        post_info.track_3d_pos = {armor.aiming_point_world.x, armor.aiming_point_world.y, armor.aiming_point_world.z};
        post_info.pred_3d_pos = *(predictor(armor, sleep_time));
        if (post_process_info.track_3d_pos.norm() <= 4.5)
        {
            post_info.find_target = true;
            post_info.is_shooting = true;
            post_info.switch_target = false;
        }
        else
        {
            post_info.find_target = true;
            post_info.is_shooting = false;
            post_info.switch_target = true;
        }

        return true;
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
            // is_ekf_init = false;
            is_singer_init_[0][0] = false;
            is_singer_init_[0][1] = false;
            is_singer_init_[0][2] = false;
            is_singer_init_[1][0] = false;
            is_singer_init_[1][1] = false;
            is_singer_init_[1][2] = false;
            is_imm_init_ = false;
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
            // is_ekf_init = false;
            is_imm_init_ = false;
            is_singer_init_[0][0] = false;
            is_singer_init_[0][1] = false;
            is_singer_init_[0][2] = false;
            is_singer_init_[1][0] = false;
            is_singer_init_[1][1] = false;
            is_singer_init_[1][2] = false;
        }

        auto hit_point = predict(target, target.timestamp, sleep_time, &src);
        return std::make_unique<Eigen::Vector3d>(hit_point);
    }
} // armor_processor