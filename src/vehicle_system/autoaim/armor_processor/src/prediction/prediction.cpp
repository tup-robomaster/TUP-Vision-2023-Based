/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2023-03-01 09:51:45
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.hpp"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor()
    : logger_(rclcpp::get_logger("armor_prediction"))
    {
        // KF initialized.
        singer_kf_[0].Init(3, 1, 1);
        singer_kf_[1].Init(3, 1, 1);
        singer_kf_[2].Init(3, 1, 1);
        singer_model_[0] = SingerModel(3, 1, 1);
        singer_model_[1] = SingerModel(3, 1, 1);
        singer_model_[2] = SingerModel(3, 1, 1);

        // SingerModel singer;
        kfInit();

        // Load pf param.
        // loadParam(filter_param_path_);

        filter_disabled_ = false;
        fitting_disabled_ = false;
        is_init_ = false;
        is_predicted_ = false;
        is_singer_init_[0] = false;
        is_singer_init_[1] = false;
        is_singer_init_[2] = false;
    }

    ArmorPredictor::~ArmorPredictor(){}

    ArmorPredictor::ArmorPredictor(const PredictParam& predict_param, vector<double>* singer_param, 
        const PathParam& path_param, const DebugParam& debug_param)
    : predict_param_(predict_param), filter_param_path_(path_param.filter_path), debug_param_(debug_param),
    logger_(rclcpp::get_logger("armor_prediction"))
    {
        // config_ = YAML::LoadFile(coord_file);
        // pf_pos.initParam(config_, "pos");
        // pf_v.initParam(config_, "v");

        // KF initialized.
        singer_kf_[0].Init(3, 1, 1);
        singer_kf_[1].Init(3, 1, 1);
        singer_kf_[2].Init(3, 1, 1);
        singer_param_[0] = singer_param[0];
        singer_param_[1] = singer_param[1];
        singer_param_[2] = singer_param[2];
        singer_model_[0] = SingerModel(singer_param_[0], 3, 1, 1);
        singer_model_[1] = SingerModel(singer_param_[1], 3, 1, 1);
        singer_model_[2] = SingerModel(singer_param_[2], 3, 1, 1);

        // cout << "singer_param:" <<  singer_param_[0][0] << " " << singer_param_[0][1] << " " << singer_param_[0][2] << " " <<  singer_param_[0][3]
            // << " " <<  singer_param_[0][4] << " " <<  singer_param_[0][5] << " " <<  singer_param_[0][6] << " " <<  singer_param_[0][7] << endl;
        
        // SingerModel singer;
        kfInit();

        // Load pf param.
        // loadParam(filter_param_path_);

        filter_disabled_ = debug_param_.disable_filter;
        fitting_disabled_ = false;
        is_init_ = false;
        is_singer_init_[0] = false;
        is_singer_init_[1] = false;
        is_singer_init_[2] = false;
    }

    /**
     * @brief 加载滤波参数
     * 
     * @param filter_param_path 滤波参数文件路径
     */
    void ArmorPredictor::loadParam(std::string filter_param_path)
    {
        if(!is_init_)
        {
            config_ = YAML::LoadFile(filter_param_path_);
            pf_pos.initParam(config_, "pos");
            pf_v.initParam(config_, "v");
            is_init_ = true;
        }
    }

    PostProcessInfo&& ArmorPredictor::postProcess(AutoaimMsg& target_msg)
    {
        PostProcessInfo post_process_info;
        double delay_time = 0.0;
        post_process_info.track_3d_pos = {target_msg.aiming_point_world.x, target_msg.aiming_point_world.y, target_msg.aiming_point_world.z};
        post_process_info.pred_3d_pos = predict(target_msg, target_msg.timestamp, delay_time);
        
        // 1.根据目标速度判定是否击打或切换目标
        if ((history_vel_[1][0] > 2.0 || history_vel_[1][0] > 2.0) && (predict_vel_[1][0] > 2.0 || predict_vel_[1][0] > 2.0)
            && (history_vel_[0][0] > 2.0 || history_vel_[0][0] > 2.0) && (predict_vel_[0][0] > 2.0 || predict_vel_[0][0] > 2.0))
        {
            post_process_info.switch_target = true;
            post_process_info.is_shooting = false;
        }
        else if ((history_vel_[1][0] > 0.8 || history_vel_[1][0] > 0.8) && (predict_vel_[1][0] > 0.8 || history_vel_[1][0] > 0.8)
            && (history_vel_[0][0] > 0.8 || history_vel_[0][0] > 0.8) && (predict_vel_[0][0] > 0.8 || history_vel_[0][0] > 0.8))
        {
            post_process_info.switch_target = false;
            post_process_info.is_shooting = false;
        }
        else
        {
            // 2.根据目标的预测误差进行判断
            if (!filter_disabled_ && cur_pred_error_ != 0.0)
            {
                if(cur_pred_error_ > 0.2)
                {
                    post_process_info.switch_target = true;
                    post_process_info.is_shooting = false;
                }
                if(cur_pred_error_ > 0.1 && ((is_singer_init_[0] && is_singer_init_[1] && is_singer_init_[2]) || is_imm_init_))
                {
                    post_process_info.is_shooting = false;
                    post_process_info.switch_target = false;
                }
            }

            // 3.根据目标的预测位置信息
            double meas_y = post_process_info.track_3d_pos[0] + history_vel_[0][0] * delay_time / 1e9 + 0.5 * history_acc_[0][0] * pow(delay_time / 1e9, 2);
            double pred_y = post_process_info.track_3d_pos[0] + predict_vel_[0][0] * delay_time / 1e9 + 0.5 * predict_acc_[0][0] * pow(delay_time / 1e9, 2);
            double meas_x = post_process_info.track_3d_pos[1] + history_vel_[1][0] * delay_time / 1e9 + 0.5 * history_acc_[1][0] * pow(delay_time / 1e9, 2);
            double pred_x = post_process_info.track_3d_pos[1] + predict_vel_[1][0] * delay_time / 1e9 + 0.5 * predict_acc_[1][0] * pow(delay_time / 1e9, 2);
            
            double error = sqrt(pow(post_process_info.pred_3d_pos[1] - meas_x, 2) + pow(post_process_info.pred_3d_pos[1] - pred_x, 2)) / 2.0;
            if(post_process_info.pred_3d_pos[0] > 4.5 && error > 0.15)
            {
                post_process_info.is_shooting = false;
                post_process_info.switch_target = true;
            }
        }

        return std::move(post_process_info);
    }

    /**
     * @brief 对目标位置进行预测
     * 
     * @param target_msg 目标message
     * @param timestamp 本帧对应的时间戳
     * @param delay_time 休眠时间，对应于预测延迟量 
     * @param src 图像数据
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d ArmorPredictor::predict(AutoaimMsg& target_msg, uint64 timestamp, double& delay_time, cv::Mat* src)
    {
        auto t1 = steady_clock_.now();

        rclcpp::Time stamp = target_msg.header.stamp;
        int64_t dt = stamp.nanoseconds();
        Eigen::Vector3d xyz = {target_msg.aiming_point_world.x, target_msg.aiming_point_world.y, target_msg.aiming_point_world.z};
        TargetInfo target = 
        {
            std::move(xyz),
            xyz.norm(),
            dt,
            target_msg.period,
            target_msg.target_switched,
            target_msg.is_spinning,
            target_msg.spinning_switched,
            target_msg.clockwise,
            false,
            (SpinningStatus)(target_msg.is_still_spinning),
            (OutpostStatus)(target_msg.is_controlled),
            predict_param_.system_model
        };
        // RCLCPP_INFO(logger_, "src_timestamp:%.8f", dt / 1e9);

        if(target.is_outpost_mode || target.is_spinning)
        {
            fitting_disabled_ = false;
            filter_disabled_ = true;
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Spinning...");
        }
        else
        {
            fitting_disabled_ = true;
            filter_disabled_ = false;
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Maneuvering...");
        }
        filter_disabled_ = false;
        fitting_disabled_ = true;

        // -------------对位置进行粒子滤波,以降低测距噪声影响-----------------
        // Eigen::VectorXd measure (2);
        // measure << target.xyz[1], target.xyz[0];
        // bool is_pos_filter_ready = pf_pos.update(measure);
        // Eigen::VectorXd predict_pos_xy = pf_pos.predict();
        // Eigen::Vector3d predict_pos = {predict_pos_xy[0], predict_pos_xy[1], target.xyz[2]};
        // if(is_pos_filter_ready || abs(target.xyz[2] - predict_pos[1]) < 0.20)
        // {   //对位置进行粒子滤波,以降低测距噪声影响
        //     target.xyz[2] = predict_pos[1];
        // }

        // Eigen::Vector3d result_pf = {0, 0, 0};
        // PredictStatus is_pf_available;

        Eigen::Vector3d result = {0, 0, 0};
        Eigen::Vector3d result_fitting = {0, 0, 0};
        double result_singer[3] = {0.0, 0.0, 0.0};
        Eigen::Vector3d result_imm = {0, 0, 0};
        PredictStatus is_fitting_available;
        PredictStatus is_singer_available;
        PredictStatus is_imm_available;
        
        if(!fitting_disabled_)
        {
            if(!target.is_target_switched)
            {
                history_origin_info_.push_back(cv::Point2d(history_info_.front().xyz[1], history_info_.front().xyz[0]));
                history_info_.push_back(std::move(target));
                if(history_origin_info_.size() > 2)
                {   // Reserve history fitting front element.
                    history_origin_info_.pop_front();    
                }
            }
            else
            {
                is_predicted_ = false;
                history_origin_info_.clear();
                history_origin_info_.push_back(cv::Point2d(history_info_.front().xyz[1], history_info_.front().xyz[0]));
                history_info_.clear();
                history_info_.push_back(target);
            }

            if(target.spinning_switched)
            {
                history_info_.clear();
                history_info_.push_back(target);
                RCLCPP_INFO(logger_, "Target spinning switched...");
            }
        }
        else if (!filter_disabled_)
        {
            if (target.is_target_switched)
            {
                history_vel_[0][3] = history_vel_[0][2] = history_vel_[0][1] = history_vel_[0][0] = 0.0;
                history_acc_[0][3] = history_acc_[0][2] = history_acc_[0][1] = history_acc_[0][0] = 0.0;
                predict_vel_[0][3] = predict_vel_[0][2] = predict_vel_[0][1] = predict_vel_[0][0] = 0.0;
                predict_acc_[0][3] = predict_acc_[0][2] = predict_acc_[0][1] = predict_acc_[0][0] = 0.0;
                history_vel_[1][3] = history_vel_[1][2] = history_vel_[1][1] = history_vel_[1][0] = 0.0;
                history_acc_[1][3] = history_acc_[1][2] = history_acc_[1][1] = history_acc_[1][0] = 0.0;
                predict_vel_[1][3] = predict_vel_[1][2] = predict_vel_[1][1] = predict_vel_[1][0] = 0.0;
                predict_acc_[1][3] = predict_acc_[1][2] = predict_acc_[1][1] = predict_acc_[1][0] = 0.0;
                error_cnt_ = 0;
                history_info_.clear();
                history_pred_.clear();
            }
            if((int)history_info_.size() > 200)
                history_info_.pop_front();
            if((int)history_pred_.size() > 50)
                history_pred_.pop_front();
            history_info_.push_back(target);
        }
        
        // //若位置粒子滤波器未完成初始化或滤波结果与目前位置相距过远,则本次不对目标位置做滤波,直接向队列压入原值
        // if (!is_pos_filter_ready || (predict_pos - xyz).norm() > 0.1)
        // {
        //     history_info_.push_back(target);
        // }
        // else
        // {  //若位置粒子滤波器已完成初始化且预测值大小恰当,则对目标位置做滤波
        //     // cout<<"FIL:"<<predict_pos[0] - xyz[0]<<endl;
        //     target.xyz[0] = predict_pos[0];
        //     target.xyz[1] = predict_pos[1];
        //     history_info_.push_back(target);
        // }

        // auto d_xyz = target.xyz - final_target_.xyz;
        // auto delta_t = timestamp - final_target_.timestamp;
        auto last_dist = history_info_.back().dist;
        int64_t delta_time_estimate = (int64_t)((last_dist / predict_param_.bullet_speed) * 1e9) + (int64_t)(predict_param_.shoot_delay * 1e6);
        int64_t time_estimate = delta_time_estimate + history_info_.back().timestamp;
        delay_time = delta_time_estimate;
        // cout << "delay_time:" << delta_time_estimate / 1e9 << " dt:" << time_estimate / 1e9 << endl;

        if ((int)history_info_.size() < predict_param_.min_fitting_lens)
        {
            if(!fitting_disabled_ && is_predicted_ && (int)history_info_.size() > 0)
            {
                // cout << "is_pred..." << endl;
                int64_t last_to_now_timestamp = history_info_.back().timestamp - last_start_timestamp_;
                int64_t tt = time_estimate - last_start_timestamp_;
                double last_pred_x = fitting_params_[0] * (last_to_now_timestamp / 1e9) + fitting_params_[1];
                double delta_y = last_pred_x - target.xyz[0]; 
                // double tt = time_estimate / 1e3;
                // result[0] = fitting_params_[0] * (delta_time_estimate * history_info_.size() / 1e3) + history_info_.front().xyz[0]; // x(t)=kt+x0
                result[0] = target.xyz[0]; // x(t)=kt+d
                result[1] = fitting_params_[0] * (tt / 1e9) + fitting_params_[1] - delta_y;
                result[2] = target.xyz[2];
                final_target_.xyz = result;
                return result;
            }
            final_target_ = target;
            return Vector3d{target.xyz[0], target.xyz[1], target.xyz[2]};
            return Vector3d{target.xyz[0], target.xyz[1], target.xyz[2]};
        }

        //-----------------进行滑窗滤波,备选方案,暂未使用-------------------------------------
        //如速度过大,可认为为噪声干扰,进行滑窗滤波滤除
        // if (((d_xyz.norm() / delta_t) * 1e3) >= max_v)
        // {
        //     history_info.push_back(target);
        //     auto filtered_xyz = shiftWindowFilter(history_info.size() - window_size - 1);
        //     target = {filtered_xyz, (int)filtered_xyz.norm(), timestamp};
        //     history_info.pop_back();
        // }
        // auto filtered_xyz = shiftWindowFilter(history_info.size() - window_size - 1);
        // filtered_xyz << xyz[0], xyz[1], filtered_xyz[2];
        // target = {filtered_xyz, (int)filtered_xyz.norm(), timestamp};
        // history_info.pop_back();
        
        //根据帧差法计算目标速度和加速度
        Eigen::Vector3d target_vel, target_acc;
        Eigen::Vector3d delta_pos_last = {0.0, 0.0, 0.0};
        Eigen::Vector3d delta_pos_now = {0.0, 0.0, 0.0};
        Eigen::Vector3d vel_last = {0.0, 0.0, 0.0};
        Eigen::Vector3d vel_now = {0.0, 0.0, 0.0};
        Eigen::Vector3d acc_now = {0.0, 0.0, 0.0};
        double dt_last = 0;
        double dt_now = 0;
        if (!filter_disabled_ && !target.is_spinning && (int)history_info_.size() >= predict_param_.min_fitting_lens)
        {
            // 计算目标速度、加速度 
            // 取目标t-2、t-1、t时刻的坐标信息
            delta_pos_last = history_info_.at(history_info_.size() - 2).xyz - history_info_.at(history_info_.size() - 3).xyz;
            dt_last = (history_info_.at(history_info_.size() - 2).timestamp - history_info_.at(history_info_.size() - 3).timestamp) / 1e9;
            if (dt_last != 0)
            {
                vel_last = delta_pos_last / dt_last;
            }

            delta_pos_now = target.xyz - history_info_.at(history_info_.size() - 2).xyz;
            dt_now = (target.timestamp - history_info_.at(history_info_.size() - 2).timestamp) / 1e9;
            if (dt_now != 0)
            {
                vel_now = delta_pos_now / dt_now;
            }

            if (dt_now + dt_last != 0)
            {
                acc_now = (vel_now - vel_last) / ((dt_now + dt_last) * 0.5);
            }

            // bool is_v_filter_ready = pf_v.update(measure_v);
            // Eigen::Vector2d predict_v_xy = pf_v.predict();
            target_vel = vel_now;
            target_acc = acc_now;
            if (debug_param_.draw_predict)
            {
                updateVel(target_vel);
                updateAcc(target_acc);
            }
        }
        // if(is_v_filter_ready)
        // {   //若速度粒子滤波器已完成初始化且预测值大小恰当，则对目标速度做滤波
        //     target_v[0] = predict_v_xy[0];
        //     target_v[1] = predict_v_xy[1];
        // }

        /********根据目标的运动状态选择预测模型**********/
        if (!filter_disabled_)
        {   // 目标机动预测
            if (target.system_model == CSMODEL)
            {   // 基于CS模型的卡尔曼滤波
                RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "CS model is predicting...");
                std::future<void> xyz_future[3];
                // is_ekf_available = predictBasedSinger(target, result_ekf, target_vel, target_acc, delta_time_estimate);
                if (debug_param_.x_axis_filter)
                {
                    // is_singer_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);
                    xyz_future[0] = std::async(std::launch::async, [&](){
                        is_singer_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);});
                }
                if (debug_param_.y_axis_filter)
                {
                    // is_singer_available.xyz_status[1] = predictBasedSinger(1, target.xyz[1], result_singer[1], target_vel[1], target_acc[1], delta_time_estimate);
                    xyz_future[1] = std::async(std::launch::async, [&](){
                        is_singer_available.xyz_status[1] = predictBasedSinger(1, target.xyz[1], result_singer[1], target_vel[1], target_acc[1], delta_time_estimate);});
                }
                if (debug_param_.z_axis_filter)
                {
                    // is_singer_available.xyz_status[2] = predictBasedSinger(2, target.xyz[2], result_singer[2], target_vel[2], target_acc[2], delta_time_estimate);
                    xyz_future[2] = std::async(std::launch::async, [&](){
                        is_singer_available.xyz_status[2] = predictBasedSinger(2, target.xyz[2], result_singer[2], target_vel[2], target_acc[2], delta_time_estimate);});
                }
                if (debug_param_.x_axis_filter && xyz_future[0].wait_for(1ms) == std::future_status::timeout)
                {
                    RCLCPP_WARN(logger_, "X_AXIS prediction timeout...");
                }
                if (debug_param_.y_axis_filter && xyz_future[1].wait_for(1ms) == std::future_status::timeout)
                {
                    RCLCPP_WARN(logger_, "Y_AXIS prediction timeout...");
                }
                if (debug_param_.z_axis_filter && xyz_future[2].wait_for(1ms) == std::future_status::timeout)
                {
                    RCLCPP_WARN(logger_, "Z_AXIS prediction timeout...");
                }
                result[0] = is_singer_available.xyz_status[0] ? result_singer[0] : target.xyz[0];
                result[1] = is_singer_available.xyz_status[1] ? result_singer[1] : target.xyz[1];
                result[2] = is_singer_available.xyz_status[2] ? result_singer[2] : target.xyz[2];
            }
            else if (target.system_model == IMMMODEL)
            {   // 基于IMM模型的卡尔曼滤波
                RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "IMM model is predicting...");
                // RCLCPP_INFO(logger_, "IMM model is predicting...");
                is_imm_available = predictBasedImm(target, result_imm, target_vel, target_acc, delta_time_estimate);

                result[0] = is_imm_available.xyz_status[0] ? result_imm[0] : target.xyz[0];
                result[1] = is_imm_available.xyz_status[1] ? result_imm[1] : target.xyz[1];
                result[2] = is_imm_available.xyz_status[2] ? result_imm[2] : target.xyz[2];
            }
        }
        else if (!fitting_disabled_)
        {   // 曲线拟合预测
            // 击打前哨站模式
            if (target.is_outpost_mode && target.outpost_status == NORMAL)
            {
                is_fitting_available = spinningPredict(false, target, result_fitting, time_estimate);  
            }
            else if (target.is_outpost_mode && target.outpost_status == CONTROLLED)
            {
                is_fitting_available = spinningPredict(true, target, result_fitting, time_estimate);  
            }

            // 反陀螺模式
            if (target.is_spinning && (!target.spinning_status) == STILL_SPINNING)
            {   
                RCLCPP_INFO(logger_, "STILL_SPINNING");
                is_fitting_available = coupleFittingPredict(true, target, result_fitting, time_estimate);  
                is_fitting_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);
            }
            else if (target.is_spinning && (!target.spinning_status) == MOVEMENT_SPINNING)
            {
                RCLCPP_INFO(logger_, "MOVEMENT_SPINNING");
                is_fitting_available = coupleFittingPredict(false, target, result_fitting, time_estimate);  
                is_fitting_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);
            }

            result[0] = is_fitting_available.xyz_status[0] ? result_fitting[0] : target.xyz[0];
            result[1] = is_fitting_available.xyz_status[1] ? result_fitting[1] : target.xyz[1];
            result[2] = is_fitting_available.xyz_status[2] ? result_fitting[2] : target.xyz[2];
        }

        if (fitting_disabled_ && filter_disabled_)
        {   //滤波和拟合均失效，使用当前目标位置信息
            result = Vector3d{target.xyz[0], target.xyz[1], target.xyz[2]};
        }

        // if (!filter_disabled_)
        // {
        //     double error = calcError();
        //     cur_pred_error_ = error;
        //     if (error > 0.10)
        //     {
        //         ++error_cnt_;
        //     }
        //     if (error_cnt_ > 5 || error > 0.20)
        //     {
        //         RCLCPP_WARN(logger_, "Prediction failed!");
        //         error_cnt_ = 0;
        //         cur_pred_error_ = 0.0;
        //         // is_ekf_init = false;
        //         is_singer_init_[0] = false;
        //         is_singer_init_[1] = false;
        //         is_singer_init_[2] = false;
        //         is_imm_init_ = false;
        //         result = target.xyz;
        //     }
        // }
        
        auto t2 = steady_clock_.now();
        double dr_ns = (t2 - t1).nanoseconds();
        if (debug_param_.print_delay)
        {
            RCLCPP_INFO(logger_, "target: x:%lf y:%lf z:%lf", target.xyz[0], target.xyz[1], target.xyz[2]);
            RCLCPP_INFO(logger_, "Predict time:%lfms", (dr_ns / 1e6));
            RCLCPP_INFO(logger_, "predict: x:%lf y:%lf z:%lf", result[0], result[1], result[2]);
        }

        if (debug_param_.draw_predict && !((*src).empty()))
        {   // Draw curve.
            if (is_singer_available.xyz_status[0])
            {
                curveDrawer(0, *src, predict_vel_[0], cv::Point2i(260, 200));
                // curveDrawer(0, *src, history_acc_[0], cv::Point2i(620, 200));
            } 
            if (is_singer_available.xyz_status[1])
            {
                curveDrawer(1, *src, predict_vel_[1], cv::Point2i(260, 200));
                // curveDrawer(1, *src, history_acc_[1], cv::Point2i(620, 200));
            }
            if (is_singer_available.xyz_status[2])
            {
                curveDrawer(3, *src, predict_vel_[2], cv::Point2i(260, 200));
                // curveDrawer(3, *src, history_acc_[2], cv::Point2i(620, 200));
            }
            if (debug_param_.show_fps)
            {
                char ch[15];
                sprintf(ch, "FPS:%.1f", (1e9 / dr_ns));
                std::string fps_str = ch;
                putText(*src, fps_str, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 0});
            }
        }
        final_target_.xyz = result;
        // return Vector3d{result[0], result[1], result[2]};
        return result;
    }

    /**
     * @brief 计算滤波预测值与测量值的误差，判断滤波是否发散
     * 
     * @return 返回预测值与测量值之间的误差
     */
    double ArmorPredictor::calcError()
    {
        bool flag = false;
        Eigen::Vector3d pred_error = {0.0, 0.0, 0.0};
        double error = 0.0;
        if ((int)history_pred_.size() > 0)
        {
            TargetInfo pre_info = history_pred_.front();
            for(int ii = 0; ii < (int)history_info_.size(); ii++)
            {
                if(ii != (int)(history_info_.size() - 1))
                {
                    if(pre_info.timestamp >= history_info_[ii].timestamp && pre_info.timestamp < history_info_[ii + 1].timestamp)
                    {
                        // RCLCPP_INFO(logger_, "pred_timestamp:%lfs meas_timestamp:%lfs", pre_info.timestamp / 1e9, history_info_[ii].timestamp / 1e9);
                        double dt = (history_info_[ii + 1].timestamp - history_info_[ii].timestamp) / 1e9;
                        double ddt = (pre_info.timestamp - history_info_[ii].timestamp) / 1e9;
                        auto weight = ddt / dt;
                        auto meas_pred = history_info_[ii].xyz * (1 - weight) + history_info_[ii].xyz * weight;
                        pred_error[0] = abs(meas_pred[0] - pre_info.xyz[0]);
                        pred_error[1] = abs(meas_pred[1] - pre_info.xyz[1]);
                        pred_error[2] = abs(meas_pred[2] - pre_info.xyz[2]);
                        history_pred_.pop_front();
                        flag = true;
                        break;
                    }
                }
                else if(pre_info.timestamp < history_info_[ii].timestamp)
                {
                    // RCLCPP_INFO(logger_, "pred_timestamp:%lfs meas_timestamp:%lfs", pre_info.timestamp / 1e9, history_info_[ii].timestamp / 1e9);
                    pred_error[0] = abs(history_info_[ii].xyz[0] - pre_info.xyz[0]);
                    pred_error[1] = abs(history_info_[ii].xyz[1] - pre_info.xyz[1]);
                    pred_error[2] = abs(history_info_[ii].xyz[2] - pre_info.xyz[2]);
                    history_pred_.pop_front();
                    flag = true;
                    break;
                }
            }

            if (flag)
            {
                error = sqrt(pow(pred_error[0], 2) + pow(pred_error[1], 2) + pow(pred_error[2], 2)) / 3.0;
                RCLCPP_INFO(logger_, "Prediction error:%lf", error);
            }
        }
        return std::move(error);
    }

    /**
     * @brief 设置弹速
     * 
     * @param speed 
     * @return true 
     * @return false 
     */
    bool ArmorPredictor::setBulletSpeed(double speed)
    {
        predict_param_.bullet_speed = speed;
        return true;
    }

    /**
     * @brief 曲线拟合预测函数（针对目标小陀螺情况）
     * 
     * @param is_still_spinning 
     * @param target 
     * @param result 
     * @param time_estimated 
     * @return PredictStatus 
     */
    PredictStatus ArmorPredictor::coupleFittingPredict(bool is_still_spinning, TargetInfo target, Eigen::Vector3d& result, int64_t time_estimated)
    {   
        /**
         * @brief 车辆小陀螺运动轨迹拟合(已知量：角速度&陀螺半径）
         * 若目标仅处于原地小陀螺状态，则剔除掉模型中的横移项，直接给平动项乘以系数0。
         */
        // cout << "fitting..." << endl;

        auto time_start = steady_clock_.now();

        double params[5] = {fitting_params_[0], fitting_params_[1], fitting_params_[2], fitting_params_[3], fitting_params_[4]};

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        
        options.max_num_iterations = 20;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        
        double x0 = history_info_.front().xyz[1];
        int64_t st = history_info_.begin()->timestamp;
        last_start_timestamp_ = st;
        // double x_sum = 0;
        for(auto& target_info : history_info_)
        {   
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1>
                (
                    new CurveFittingCost(0, target_info.xyz[1], target_info.xyz[0], (target_info.timestamp - st) / 1e9, x0)
                ),
                new ceres::CauchyLoss(0.5),
                &params[0],
                &params[1],
                &params[2],
                &params[3],
                &params[4]
            );
        }
        
        // problem.SetParameterUpperBound(&params[0], 0, 2.0);
        // problem.SetParameterLowerBound(&params[0], 0, 0.1);

        // problem.SetParameterUpperBound(&params[1], 0, 100);
        // problem.SetParameterLowerBound(&params[1], 0, 0.1);
        
        // problem.SetParameterUpperBound(&params[2], 0, 100);
        // problem.SetParameterLowerBound(&params[2], 0, 0.1);
        
        // problem.SetParameterUpperBound(&params[3], 0, 100);
        // problem.SetParameterLowerBound(&params[3], 0, 0.1);
        
        // problem.SetParameterUpperBound(&params[4], 0, 100);
        // problem.SetParameterLowerBound(&params[4], 0, 0.1);

        ceres::Solve(options, &problem, &summary);
        // auto x_future = std::async(std::launch::async, [&](){ceres::Solve(options, &problem, &summary);});
        // auto y_future = std::async(std::launch::async, [&](){ceres::Solve(options_y, &problem_y, &summary_y);});
        // x_future.wait();
        // y_future.wait();

        auto x_cost = summary.final_cost;
        
        PredictStatus is_available;
        auto x_rmse = evalRMSE(params);
        is_available.xyz_status[1] = (x_cost <= predict_param_.max_cost && x_rmse <= predict_param_.max_cost) ? true : false;
        is_predicted_ = is_available.xyz_status[1];
        
        auto time_now = steady_clock_.now();
        auto dt = (time_now - time_start).nanoseconds();
        RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Fitting_time: %.2fms x_cost:%.2f x_rmse:%.2f k:%.2f b:%.2f", (dt / 1e6), x_cost, x_rmse, params[0], params[1]);

        double x_pred = 0.0;
        int64_t start_point = history_info_.front().timestamp;            
        if ((time_estimated - start_point) / 1e9 < target.period)
        {
            // x_pred = params[0] * ((time_estimated - st) / 1e3) + x0; // x(t)=k(t+dt)+x0
            x_pred = params[0] * ((time_estimated - st) / 1e9) + params[1]; // x(t)=k(t+dt)+d
        }
        else
        {
            if (history_origin_info_.size() == 2)
            {
                // double last_delta_x0 = history_origin_info_.at(history_origin_info_.size() - 2) - history_origin_info_.at(history_origin_info_.size() - 3);
                double cur_delta_x0 = history_origin_info_.at(history_origin_info_.size() - 1).y - history_origin_info_.at(history_origin_info_.size() - 2).y;
                // double cur_delta_y0 = history_origin_info_.at(history_origin_info_.size() - 1).x - history_origin_info_.at(history_origin_info_.size() - 2).x;
                // double delta_ave = (last_delta_x0 + cur_delta_x0) / 2.0;
                
                // double x_pred_sum = 0;
                // double ave_x_pred = 0;
                // if(history_delta_x_pred_.size() != 0)
                // {
                //     for(auto& delta_x_pred : history_delta_x_pred_)
                //         x_pred_sum += delta_x_pred;
                //     ave_x_pred = x_pred_sum / history_delta_x_pred_.size();
                // }
                
                if (!is_still_spinning)
                {
                    x_pred = params[0] * ((time_estimated - st) / 1e9 - target.period) + params[1] - cur_delta_x0; // x(t)=k(t+dt-T)+d
                }
                else
                {
                    x_pred = params[0] * ((time_estimated - st) / 1e9 - target.period) + params[1]; // x(t)=k(t+dt-T)+d
                }

                //若误差过大，则根据历史位点的中值进行枪管角度调整
                //判断误差的方法是判断预测点是否还在目标小陀螺的范围内，超出边界点的坐标即认为预测失败
                double x_origin = history_origin_info_.at(history_origin_info_.size() - 1).y;
                if ((target.is_clockwise && (x_pred > x_origin * 0.95))
                || (!target.is_clockwise && (x_pred < x_origin * 0.95)))
                {
                    x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[1];
                }
            }
            else
            {
                if (is_still_spinning)
                {
                    x_pred = params[0] * ((time_estimated - st) / 1e9 - target.period) + params[1]; // x(t)=k(t+dt-T)+d
                    if (history_origin_info_.size() != 0)
                    {
                        double x_origin = history_origin_info_.at(history_origin_info_.size() - 1).y;
                        if ((target.is_clockwise && (x_pred > x_origin * 1.05))
                        || (!target.is_clockwise && (x_pred < x_origin * 1.05)))
                        {
                            x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[1];
                        }
                    }
                }
                else
                {
                    x_pred = history_info_.at(history_info_.size() - 1).xyz[1];
                }
            }

            // double tt = (time_estimated - start_point) / 1e3 - target.period;
            // if (history_info_.size() > 6)
            // {
            //     int flag = -1;
            //     std::cout << "delta_t:" << tt << std::endl;
            //     for(int ii = 0; ii < history_info_.size(); ii++)
            //     {
            //         if(history_info_[ii].timestamp > (start_point + tt))
            //         {   
            //             flag = ii;
            //             break;
            //         }
            //         else
            //             continue;
            //     }
            //     std::cout << "id:" << flag << std::endl;
            //     x_pred = history_info_.at(flag).xyz[0];

            //     x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[0];
            // }
            // else
            // {
            //     x_pred = params[0] * ((target.timestamp - st) / 1e3 - target.period) + x0; // x(t)=k(t+dt-T)+x0
            //     x_pred = params[0] * ((target.timestamp - st) / 1e3 - target.period) + params[1]; // x(t)=k(t+dt-T)+d
            //     x_pred = target.xyz[0];
            //     x_pred = history_info_.at(3).xyz[0];
            // }
            //     x_pred = history_info_.at(history_info_.size() - 1).xyz[0];
        }

        std::memcpy(fitting_params_, params, sizeof(fitting_params_));

        double x_front = history_info_.front().xyz[1];
        double x_back = history_info_.back().xyz[1];
        last_end_x_ = x_back;

        // if(abs(x_pred) > abs(history_info_.front().xyz[0])) // Time backtracking.
        //     x_pred = history_info_.at(4).xyz[0];
    
        // double delta_x_to_pred = x_pred - target.xyz[0];
        // if(history_delta_x_pred_.size() < 1)
        // {
        //     history_delta_x_pred_.push_back(delta_x_to_pred);
        // }
        // else
        // {
        //     history_delta_x_pred_.pop_front();
        //     history_delta_x_pred_.push_back(delta_x_to_pred);
        // }
        // cout << "x_pred:" << x_pred << endl;
        result = {target.xyz[0], x_pred, target.xyz[2]}; 
        return is_available;
    }


    /**
     * @brief 基于CS模型的卡尔曼滤波初始化
     * 
     */
    void ArmorPredictor::kfInit()
    {
        //X-axis
        kfInit(0);
        //Y-axis
        kfInit(1);
        //Z-axis
        kfInit(2);
    }

    void ArmorPredictor::kfInit(int axis)
    {
        singer_kf_[axis].F_ = singer_model_[axis].F();
        singer_kf_[axis].H_ = singer_model_[axis].H();
        singer_kf_[axis].C_ = singer_model_[axis].C();
        singer_kf_[axis].P_ = singer_model_[axis].P();
        singer_kf_[axis].Q_ = singer_model_[axis].Q();
        singer_kf_[axis].R_ = singer_model_[axis].R();
    }
    
    bool ArmorPredictor::predictBasedSinger(int axis, double meas, double& result, double target_vel, double target_acc, int64_t timestamp)
    {
        bool is_available;
        if (!is_singer_init_[axis])
        {
            singer_kf_[axis].x_ << meas, 0, 0;
            is_available = false;
            is_singer_init_[axis] = true;
        }
        else
        {
            Eigen::VectorXd measurement = Eigen::VectorXd(1);
            measurement << meas;
            
            singer_kf_[axis].Predict();
            Eigen::MatrixXd stateCovPre = singer_kf_[axis].P();
            Eigen::MatrixXd statePre = singer_kf_[axis].x();
            singer_kf_[axis].Update(measurement);

            // Eigen::MatrixXd predictState(3, 1);
			Eigen::VectorXd State(3, 1);
            State << singer_kf_[axis].x_[0], singer_kf_[axis].x_[1], singer_kf_[axis].x_[2];

            double post_pos = State[0];

            predict_vel_[axis][3] = predict_vel_[axis][2];
            predict_vel_[axis][2] = predict_vel_[axis][1];
            predict_vel_[axis][1] = predict_vel_[axis][0];
            predict_vel_[axis][0] = State[1];
            
            predict_acc_[axis][3] = predict_acc_[axis][2];
            predict_acc_[axis][2] = predict_acc_[axis][1];
            predict_acc_[axis][1] = predict_acc_[axis][0];
            predict_acc_[axis][0] = State[2];

            double alpha = singer_param_[axis][0];
            double dt = singer_param_[axis][8] * singer_param_[axis][4];
            // dt = timestamp / 1e9;
            dt = timestamp / 1e9 * 60;
            // cout << dt << endl;
            // cout << dt << " " << timestamp / 1e9 << endl;
            
            Eigen::MatrixXd F(3, 3);
            singer_model_[axis].setF(F, dt, alpha);

            Eigen::MatrixXd control(3, 1);
            singer_model_[axis].setC(control, dt, alpha);

            // if (history_acc_[axis][0] == 0.0)
            //     singer_model_[axis].setQ(target_acc);
            // else
            //     singer_model_[axis].setQ(State[2]);

            VectorXd pred = F * State + control * State[2];
            result = pred[0];

            if (checkDivergence(statePre, stateCovPre, singer_kf_[axis].H_, singer_kf_[axis].R_, measurement) || abs(result - meas) > 0.85)
            {
                RCLCPP_WARN(logger_, "Filter is diverging...");
                // singer_kf_->P_ = singer_model_[axis].P();
                // is_singer_init_[axis] = false;
                is_available = false;   
            } 
            // else if (abs(result - meas) > 0.85)
            // {
            //     result = meas;
            //     is_available = true;
            // }
            // else
            // {
            //     result = post_pos;
            //     is_available = true;
            // }
             
            // if (abs(result - meas) > 0.75)
            // {
            //     is_singer_init_[axis] = false;
            //     result = meas;
            // }
            is_available = true;
        }
        return is_available;
    }

    /**
     * @brief 基于IMM模型的滤波预测函数
     * 
     * @param target 目标信息
     * @param result 预测结果
     * @param target_v 目标速度
     * @param ax 目标加速度
     * @param timestamp 时间提前量
     * @return PredictStatus 
     */
    PredictStatus ArmorPredictor::predictBasedImm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector3d& target_vel, Eigen::Vector3d& target_acc, int64_t timestamp)
    {
        PredictStatus is_available;
        double dt = singer_param_[0][4];   
        if(!is_imm_init_)
        {
            Eigen::VectorXd x(6);
            x << target.xyz[0], target.xyz[1], target_vel[0], target_vel[1], 0, 0;
            imm_ = model_generator_.generateIMMModel(x, dt);   
            is_imm_init_ = true;
        }
        else
        {
            Eigen::VectorXd measurement(4);
            measurement << target.xyz[0], target.xyz[1], target_vel[0], target_vel[1];
            imm_->updateOnce(measurement, dt);

            Eigen::VectorXd State(6);
            State = imm_->x();

            result[0] = State[0];
            result[1] = State[1];
            result[2] = target.xyz[2];
            is_available.xyz_status[0] = true;
            is_available.xyz_status[1] = true;
        }
        return is_available;
    }

    /**
     * @brief 滑窗滤波
     * 
     * @param start_idx 滑窗起始位点
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d ArmorPredictor::shiftWindowFilter(int start_idx)
    {
        //计算最大迭代次数
        auto max_iter = int(history_info_.size() - start_idx) - predict_param_.window_size + 1;
        Eigen::Vector3d total_sum = {0, 0, 0};
        if (max_iter == 0 || start_idx < 0)
            return history_info_.back().xyz;
        
        for (int i = 0; i < max_iter; i++)
        {
            Eigen::Vector3d sum = {0,0,0};
            for (int j = 0; j < predict_param_.window_size; j++)
                sum += history_info_.at(start_idx + i + j).xyz;
            total_sum += sum / predict_param_.window_size;
        }
        return total_sum / max_iter;
    }

    /**
     * @brief 粒子滤波预测函数
     * 
     * @param target 目标信息
     * @param result 预测信息
     * @param time_estimated 延迟时间量
     * @return PredictStatus 各个轴预测成功与否
     */
    PredictStatus ArmorPredictor::predictBasePF(TargetInfo target, Vector3d& result, int64_t time_estimated)
    {
        PredictStatus is_available;
        //采取中心差分法,使用 t, t-1, t-2时刻速度,计算t-1时刻的速度
        auto target_prev = history_info_.at(history_info_.size() - 3);
        auto target_next = target;
        auto v_xyz = (target_next.xyz - target_prev.xyz) / (target_next.timestamp - target_prev.timestamp) * 1e9;
        auto t = target_next.timestamp - history_info_.at(history_info_.size() - 2).timestamp;

        is_available.xyz_status[0] = pf_v.is_ready;
        is_available.xyz_status[1] = pf_v.is_ready;
        // cout<<v_xyz<<endl;

        //Update
        Eigen::VectorXd measure (2);
        measure << v_xyz[0], v_xyz[1];
        pf_v.update(measure);

        //Predict
        auto result_v = pf_v.predict();

        std::cout << measure << std::endl;

        // cout<<result_v<<endl;
        //TODO:恢复速度预测
        // auto predict_x = target.xyz[0];
        // auto predict_y = target.xyz[1];
        double predict_x;
        double predict_y;

        if (history_info_.size() > 6)
        {
            predict_x = target.xyz[0] + result_v[0] * (time_estimated + t) / 1e9;
            predict_y = target.xyz[1] + result_v[1] * (time_estimated + t) / 1e9;
        }
        else
        {
            predict_x = target.xyz[0];
            predict_y = target.xyz[1];       
        }

        result << predict_x, predict_y, target.xyz[2];
        return is_available;
    }

    void ArmorPredictor::updateVel(Eigen::Vector3d vel_3d)
    {
        // X-AXIS
        history_vel_[0][3] = history_vel_[0][2];
        history_vel_[0][2] = history_vel_[0][1];
        history_vel_[0][1] = history_vel_[0][0];
        history_vel_[0][0] = vel_3d[0];

        // Y-AXIS
        history_vel_[1][3] = history_vel_[1][2];
        history_vel_[1][2] = history_vel_[1][1];
        history_vel_[1][1] = history_vel_[1][0];
        history_vel_[1][0] = vel_3d[1];

        // Z-AXIS
        history_vel_[2][3] = history_vel_[2][2];
        history_vel_[2][2] = history_vel_[2][1];
        history_vel_[2][1] = history_vel_[2][0];
        history_vel_[2][0] = vel_3d[2];
        return;
    }
    
    void ArmorPredictor::updateAcc(Eigen::Vector3d acc_3d)
    {
        // X-AXIS
        history_acc_[0][3] = history_acc_[0][2];
        history_acc_[0][2] = history_acc_[0][1];
        history_acc_[0][1] = history_acc_[0][0];
        history_acc_[0][0] = acc_3d[0] > 5.0 ? 0.0 : acc_3d[0];

        // Y-AXIS
        history_acc_[1][3] = history_acc_[1][2];
        history_acc_[1][2] = history_acc_[1][1];
        history_acc_[1][1] = history_acc_[1][0];
        history_acc_[1][0] = acc_3d[1] > 5.0 ? 0.0 : acc_3d[1];

        // Z-AXIS
        history_acc_[2][3] = history_acc_[2][2];
        history_acc_[2][2] = history_acc_[2][1];
        history_acc_[2][1] = history_acc_[2][0];
        history_acc_[2][0] = acc_3d[2] > 5.0 ? 0.0 : acc_3d[2];
        return;
    }

    /**
     * @brief 计算RMSE指标
     * 
     * @param params 参数首地址指针
     * @return RMSE值 
     */
    double ArmorPredictor::evalRMSE(double* params)
    {
        double rmse_sum = 0;
        double rmse = 0;
        double pred = 0;
        double measure = 0;
        for (auto& target_info : history_info_)
        {
            auto t = (double)(target_info.timestamp) / 1e9;
            pred = params[0] * t + params[1]; //f(t)=kt+b
            measure = target_info.xyz[1];
        }
        rmse = sqrt(rmse_sum / history_info_.size());
        return rmse;
    }

    /**
     * @brief 前哨站旋转预测函数
     * 
     * @param is_controlled 我方是否处于控制区，此时前哨站转速减半
     * @param target 目标信息
     * @param result 预测结果
     * @param time_estimated 时间延迟量
     * @return PredictStatus 
     */
    PredictStatus ArmorPredictor::spinningPredict(bool is_controlled, TargetInfo& target, Eigen::Vector3d& result, int64_t time_estimated)
    {  
        /**
         * @brief 前哨站旋转装甲运动预测（已知量：旋转半径&转速），考虑我方占领控制区旋转装甲板转速减半，应加入条件判断。
         * 
         */
        //轨迹拟合
        auto time_start = steady_clock_.now();
        double x0, y0, theta;

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.max_num_iterations = 20;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        Eigen::Vector3d xyz_sum = {0, 0, 0};
        if(!is_controlled)
        {
            for(auto& target_info : history_info_)
            {   
                xyz_sum += target_info.xyz;
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 1)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 1)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
            }
        }
        else
        {
            for(auto& target_info : history_info_)
            {   
                xyz_sum += target_info.xyz;
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 0.5)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 0.5)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
            }
        }
        auto xyz_ave = (xyz_sum / history_info_.size());

        problem.SetParameterUpperBound(&x0, 0, xyz_ave[0] + 0.6);
        problem.SetParameterLowerBound(&x0, 0, xyz_ave[0] - 0.6);
        problem.SetParameterUpperBound(&y0, 0, xyz_ave[1] + 0.6);
        problem.SetParameterLowerBound(&y0, 0, xyz_ave[1] - 0.6);
        problem.SetParameterUpperBound(&theta, 0, M_PI / 2);
        problem.SetParameterLowerBound(&theta, 0, -M_PI / 2);

        ceres::Solve(options, &problem, &summary);

        auto time_now = steady_clock_.now();
        auto dt_ns = (time_now - time_start).nanoseconds();
        RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "fitting_time:%.2fs", (dt_ns / 1e9));

        PredictStatus is_available;
        // auto rmse = evalRMSE()
        is_available.xyz_status[0] = (summary.final_cost <= predict_param_.max_cost);
        is_available.xyz_status[1] = (summary.final_cost <= predict_param_.max_cost);
        double x_pred, y_pred;
        if(!is_controlled)
        {
            x_pred = x0 + 0.2765 * ceres::cos(0.8 * M_PI * (time_estimated / 1e3) + theta);
            y_pred = y0 + 0.2765 * ceres::sin(0.8 * M_PI * (time_estimated / 1e3) + theta);
        }
        else
        {
            x_pred = x0 + 0.2765 * ceres::cos(0.4 * M_PI * (time_estimated / 1e3) + theta);
            y_pred = y0 + 0.2765 * ceres::sin(0.4 * M_PI * (time_estimated / 1e3) + theta);
        }

        result = {x_pred, y_pred, history_info_.end()->xyz[2]};
        return is_available;
    }

    /**
     * @brief Draw curve.
    */
    void ArmorPredictor::curveDrawer(int axis, cv::Mat& src, double* params, cv::Point2i start_pos)
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
            RCLCPP_ERROR(logger_, "Error while drawing curve: %s", e.what());
        }
    }
} //namespace armor_processor