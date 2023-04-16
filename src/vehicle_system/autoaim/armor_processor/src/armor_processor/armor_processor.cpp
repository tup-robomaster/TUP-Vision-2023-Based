/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2023-04-16 19:02:55
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor/armor_processor.cpp
 */
#include "../../include/armor_processor/armor_processor.hpp"

namespace armor_processor
{
    Processor::Processor(const PredictParam& predict_param, vector<double>* singer_model_param, const DebugParam& debug_param)
    : armor_predictor_(predict_param, singer_model_param, debug_param)
    {
        car_id_map_ = {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };

        //初始化预测器
        armor_predictor_.resetPredictor();
    }
    
    Processor::Processor()
    {
        car_id_map_ = {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };

        //初始化预测器
        armor_predictor_.resetPredictor();
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
            success = coordsolver_.setStaticAngleOffset(armor_predictor_.predict_param_.angle_offset);
            is_param_initialized_ = true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(armor_predictor_.logger_, "Error while initializing: %s", e.what());
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
        // PostProcessInfo post_info = PostProcessInfo();
        // double sleep_time = 0.0;
        // post_info.track_3d_pos = {armor.aiming_point_world.x, armor.aiming_point_world.y, armor.aiming_point_world.z};
        // post_info.pred_3d_pos = *(predictor(armor, sleep_time));
        // if (post_process_info.track_3d_pos.norm() <= 4.5)
        // {
        //     post_info.find_target = true;
        //     post_info.is_shooting = true;
        //     post_info.switch_target = false;
        // }
        // else
        // {
        //     post_info.find_target = true;
        //     post_info.is_shooting = false;
        //     post_info.switch_target = true;
        // }

        return true;
    }

    /**
     * @brief 对目标装甲板的位置进行预测
     * 
     * @param target 目标装甲板message信息
     * @param sleep_time 休眠时间，对应预测延迟时间，用于改变预测点message的时间戳从而方便观察测量值与预测量间的误差
     * @return std::unique_ptr<Eigen::Vector3d> 
     */
    bool Processor::predictor(AutoaimMsg& target_msg, Eigen::Vector3d& pred_result, double& sleep_time)
    {
        bool is_success = false;
        rclcpp::Time stamp = target_msg.header.stamp;
        int64_t dt = stamp.nanoseconds();

        Eigen::Vector3d xyz = {target_msg.aiming_point_world.x, target_msg.aiming_point_world.y, target_msg.aiming_point_world.z};
        TargetInfo target = 
        {
            std::move(xyz),
            xyz.norm(),
            dt,
            target_msg.period,
            target_msg.is_target_lost,
            target_msg.target_switched,
            target_msg.is_spinning,
            target_msg.spinning_switched,
            target_msg.clockwise,
            false,
            (SpinningStatus)(target_msg.is_still_spinning),
            (OutpostStatus)(target_msg.is_controlled),
            armor_predictor_.predict_param_.system_model
        };

        if (target.is_spinning && target.is_spinning_switched)
        {
            armor_predictor_.target_period_ = target.period;
        }
        if(target.is_outpost_mode || target.is_spinning)
        {
            armor_predictor_.fitting_disabled_ = false;
            armor_predictor_.filter_disabled_ = true;
        }
        else
        {
            armor_predictor_.fitting_disabled_ = true;
            armor_predictor_.filter_disabled_ = false;
        }

        if (target.is_target_lost && armor_predictor_.predictor_state_ == PREDICTING)
        {
            armor_predictor_.predictor_state_ = LOSTING;
        }

        if (target.is_target_lost && armor_predictor_.predictor_state_ == LOSTING)
        {
            if (lost_cnt_ <= 5)
            {
                //将预测器状态置为丢失预测状态（进入预测追踪阶段）
                armor_predictor_.predictor_state_ = LOSTING;
                ++lost_cnt_;
            }
            else if (lost_cnt_ > 5)
            {
                armor_predictor_.predictor_state_ = LOST;
                lost_cnt_ = 0;
            }
        }
        else
        {
            if(!target.is_target_switched && armor_predictor_.predictor_state_ == LOSTING)
            {   //当目标丢失后又出现时直接预测
                for (int ii = 0; ii < 3; ii++)
                {
                    Eigen::Vector3d pred_state = armor_predictor_.singer_kf_[target.is_spinning][ii].x();
                    if (abs(target.xyz[ii] - pred_state[0]) > 0.2)
                    {
                        armor_predictor_.singer_kf_[target.is_spinning][ii].x_ << target.xyz[ii], pred_state[1], pred_state[2];   
                    }
                }
                armor_predictor_.predictor_state_ = PREDICTING;
            }
            else if (target.is_target_switched || armor_predictor_.predictor_state_ == LOST)
            {
                //重置预测器
                armor_predictor_.resetPredictor();
                armor_predictor_.predictor_state_ = PREDICTING;
            }
            else if (target.is_spinning && target.is_spinning_switched)
            {
                //更新预测器（位置&速度继承）
                armor_predictor_.updatePredictor(target.is_spinning, target);
                armor_predictor_.predictor_state_ = PREDICTING;
            }
        }

        if (armor_predictor_.predictor_state_ != LOST)
        {
            pred_result = armor_predictor_.predict(target, target.timestamp, sleep_time);
            is_success = true;
        }

        return is_success;
    }

    /**
     * @brief 设置弹速
     * 
     * @param speed 
     * @return true 
     * @return false 
     */
    bool Processor::setBulletSpeed(double speed)
    {
        armor_predictor_.predict_param_.bullet_speed = speed;
        return true;
    }

    /**
     * @brief Draw curve.
    */
    void Processor::curveDrawer(int axis, cv::Mat& src, double* params, cv::Point2i start_pos)
    {
        try
        {
            float mean_v = (params[0] + params[1] + params[2] + params[3]) / 4.0;
            char ch[15];
            sprintf(ch, "%.3f", mean_v);
            std::string str = ch;
            float k1 = 50;
            if (!src.empty())
            {
                string axis_str = (axis == 0 && start_pos.x == 260) ? "X_AXIS" : (((axis == 1 && start_pos.x == 260)) ? "Y_AXIS" : (((axis == 3 && start_pos.x == 260)) ? "Z_AXIS" : ""));
                cv::putText(src, axis_str, cv::Point(1, start_pos.y * (axis + 1) - 5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 125));
                cv::line(src, cv::Point(start_pos.x - 240, start_pos.y * (axis + 1)), cv::Point(start_pos.x, start_pos.y * (axis + 1)), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
                cv::line(src, cv::Point(start_pos.x - 240, start_pos.y * (axis + 1) + mean_v * k1), cv::Point(start_pos.x, start_pos.y * (axis + 1) + mean_v * k1), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
                cv::putText(src, str, cv::Point(start_pos.x + 10, start_pos.y * (axis + 1) + mean_v * k1), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 0));
                cv::line(src, cv::Point(start_pos.x - 180, start_pos.y * (axis + 1) + params[3] * k1), cv::Point(start_pos.x - 120, start_pos.y * (axis + 1) + params[2] * k1), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
                cv::line(src, cv::Point(start_pos.x - 120, start_pos.y * (axis + 1) + params[2] * k1), cv::Point(start_pos.x - 60, start_pos.y * (axis + 1) + params[1] * k1), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
                cv::line(src, cv::Point(start_pos.x - 60, start_pos.y * (axis + 1) + params[1] * k1), cv::Point(start_pos.x, start_pos.y * (axis + 1) + params[0] * k1), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(armor_predictor_.logger_, "Error while drawing curve: %s", e.what());
        }
    }

    // /**
    //  * @brief 加载滤波参数
    //  * 
    //  * @param filter_param_path 滤波参数文件路径
    //  */
    // void Processor::loadParam(std::string filter_param_path)
    // {
    //     if(!is_init_)
    //     {
    //         config_ = YAML::LoadFile(filter_param_path_);
    //         pf_pos.initParam(config_, "pos");
    //         pf_v.initParam(config_, "v");
    //         is_init_ = true;
    //     }
    // }
    
    // /**
    //  * @brief 同上
    //  * 
    //  * @param src 传入图像信息，方便可视化
    //  * @param target 
    //  * @param sleep_time 
    //  * @return std::unique_ptr<Eigen::Vector3d> 
    //  */
    // std::unique_ptr<Eigen::Vector3d> Processor::predictor(cv::Mat& src, AutoaimMsg& target, double& sleep_time)
    // {
    //     if(target.target_switched)
    //     {

    //     }

    //     auto hit_point = armor_predictor_.predict(target, target.timestamp, sleep_time, &src);
    //     return std::make_unique<Eigen::Vector3d>(hit_point);
    // }
} // armor_processor