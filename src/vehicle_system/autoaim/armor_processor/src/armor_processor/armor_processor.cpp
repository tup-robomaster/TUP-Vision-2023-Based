/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 10:49:05
 * @LastEditTime: 2023-05-31 17:09:45
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor/armor_processor.cpp
 */
#include "../../include/armor_processor/armor_processor.hpp"

namespace armor_processor
{
    Processor::Processor(const PredictParam& predict_param, vector<double>* uniform_ekf_param,
        vector<double>* singer_ekf_param, const DebugParam& debug_param)
    : logger_(rclcpp::get_logger("armor_processor")), predict_param_(predict_param), debug_param_(debug_param),
    armor_predictor_(predict_param, debug_param)
    {
        //初始化预测器
        armor_predictor_.initPredictor(uniform_ekf_param, singer_ekf_param);
        armor_predictor_.resetPredictor();

        car_id_map_ = {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };
    }
    
    Processor::Processor()
    : logger_(rclcpp::get_logger("armor_processor"))
    {
        car_id_map_ = {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };

        //初始化预测器
        armor_predictor_.initPredictor();
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
            is_param_initialized_ = true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Error while initializing: %s", e.what());
        }
    }

    /**
     * @brief 对目标装甲板的位置进行预测
     * 
     * @param target 目标装甲板message信息
     * @param sleep_time 休眠时间，对应预测延迟时间，用于改变预测点message的时间戳从而方便观察测量值与预测量间的误差
     * @return std::unique_ptr<Eigen::Vector3d> 
     */
    bool Processor::predictor(AutoaimMsg& target_msg, Eigen::Vector3d& pred_result, vector<Eigen::Vector4d>& armor3d_vec, double& sleep_time)
    {
        bool is_success = false;
        rclcpp::Time stamp = target_msg.header.stamp;
        armor_predictor_.now_ = stamp.nanoseconds() / 1e9;
        // cout << "now:" << armor_predictor_.now_ << endl;

        double dt = (stamp.nanoseconds() - last_timestamp_.nanoseconds()) / 1e9;
        double bullet_speed = coordsolver_.getBulletSpeed();
        if (dt > 1.5)
            dt = 0.015;

        RCLCPP_WARN_THROTTLE(
            logger_, 
            steady_clock_, 
            100, 
            "bullet_speed:%.3f dt:%.3f", 
            bullet_speed, dt
        );
        
        if (target_msg.is_target_lost && armor_predictor_.predictor_state_ == LOSTING)
        {
            double pred_dt = last_target_.xyz.norm() / bullet_speed + predict_param_.shoot_delay / 1e3;
            last_target_.is_target_lost = true;
            int max_losting_cnt = (target_msg.mode == AUTOAIM_SLING ? 35 : 5);
            
            if (lost_cnt_ <= max_losting_cnt)
            {
                //进入预测追踪阶段
                is_success = armor_predictor_.predict(last_target_, dt, pred_dt, sleep_time, pred_result, armor3d_vec);
                ++lost_cnt_;
            }
            else
            {
                armor_predictor_.predictor_state_ = LOST;
                lost_cnt_ = 0;
            }

            RCLCPP_WARN_THROTTLE(
                logger_,
                steady_clock_,
                50,
                "Losting: %d",
                lost_cnt_
            );
        }

        for (auto armor : target_msg.armors)
        {
            double rangle = armor.rangle;
            Eigen::Vector3d xyz = {armor.point3d_world.x, armor.point3d_world.y, armor.point3d_world.z};
            // cout << "armor_point3d_world:" << xyz(0) << " " << xyz(1) << " " << xyz(2) << endl;
            
            double pred_dt = xyz.norm() / bullet_speed + predict_param_.shoot_delay / 1e3;
            Eigen::VectorXd state = armor_predictor_.uniform_ekf_.x();
            Eigen::Vector3d center_xyz = {state(0), state(1), state(2)};

            TargetInfo target = 
            { 
                std::move(xyz),
                rangle,
                xyz.norm(),
                dt,
                target_msg.spinning_period,
                target_msg.is_target_lost,
                target_msg.target_switched,
                target_msg.is_spinning,
                target_msg.spinning_switched,
                target_msg.is_clockwise,
                (target_msg.mode == AUTOAIM_SLING ? true : false),
                (SpinningStatus)(target_msg.is_still_spinning),
                predict_param_.system_model
            };
            
            if (!target.is_target_lost && armor_predictor_.predictor_state_ == LOSTING)
            {   //目标丢失后又重新出现
                armor_predictor_.predictor_state_ = PREDICTING;
                if (target.is_spinning_switched)
                {
                    Eigen::Vector4d meas = {xyz(0), xyz(1), xyz(2), rangle};
                    armor_predictor_.updatePredictor(target.is_spinning, meas);
                }
                is_success = armor_predictor_.predict(target, dt, pred_dt, sleep_time, pred_result, armor3d_vec);
                lost_cnt_ = 0;

                RCLCPP_INFO_THROTTLE(
                    logger_,
                    steady_clock_,
                    50,
                    "Losting appearing..."  
                );
            }
            else if (target.is_target_switched && !target.is_target_lost)
            {
                armor_predictor_.predictor_state_ = PREDICTING;
                if (armor_predictor_.resetPredictor())
                {
                    RCLCPP_WARN(logger_, "Reset predictor...");
                    is_success = armor_predictor_.predict(target, dt, pred_dt, sleep_time, pred_result, armor3d_vec);
                }                
            }
            else if (target.is_spinning_switched && !target.is_target_lost)
            {
                RCLCPP_WARN(logger_, "Update predictor...");
                // target_period_ = target.period;
                armor_predictor_.predictor_state_ = PREDICTING;

                Eigen::Vector4d meas = {xyz(0), xyz(1), xyz(2), rangle};
                armor_predictor_.updatePredictor(target.is_spinning, meas);
                is_success = armor_predictor_.predict(target, dt, pred_dt, sleep_time, pred_result, armor3d_vec);
            }
            else if (!target.is_target_lost)
            {
                armor_predictor_.predictor_state_ = PREDICTING;
                is_success = armor_predictor_.predict(target, dt, pred_dt, sleep_time, pred_result, armor3d_vec);
            }

            last_target_ = target;
        }

        RCLCPP_WARN_THROTTLE(
            logger_, 
            steady_clock_, 
            100, 
            "State:%s", 
            armor_predictor_.predictor_state_ == LOST ? "LOST" : (armor_predictor_.predictor_state_ == LOSTING ? "LOSTING" : "PREDICTING")
        );
        last_timestamp_ = stamp;
        return is_success;
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
                string axis_str = (axis == 0 && start_pos.x == 260) ? "X_AXIS" : (((axis == 1 && start_pos.x == 260)) ? "Y_AXIS" : (((axis == 2 && start_pos.x == 260)) ? "Z_AXIS" : ""));
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
            RCLCPP_ERROR(logger_, "Error while drawing curve: %s", e.what());
        }
    }
} // armor_processor