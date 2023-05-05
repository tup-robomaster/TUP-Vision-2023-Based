/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2023-05-05 19:36:47
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.hpp"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor()
    : logger_(rclcpp::get_logger("armor_prediction"))
    {
        // EKF initialized.
        singer_model_ = SingerModel(9, 3, 3);

        // 初始化滤波器状态
        resetPredictor();
    }

    ArmorPredictor::~ArmorPredictor(){}

    ArmorPredictor::ArmorPredictor(const PredictParam& predict_param, vector<double>* ekf_param, const DebugParam& debug_param)
    : predict_param_(predict_param), debug_param_(debug_param),
    logger_(rclcpp::get_logger("armor_prediction"))
    {
        // EKF initialized.
        singer_model_ = SingerModel(ekf_param, 9, 3, 3);
        
        // 初始化滤波器状态
        resetPredictor();
    }


    bool ArmorPredictor::resetPredictor()
    {
        is_init_ = false;
        is_imm_init_ = false;
        is_singer_init_ = false;
        filter_disabled_ = false;
        fitting_disabled_ = false;
        history_info_.clear();
        history_pred_.clear();
        history_losting_pred_.clear();
        return true;
    }

    bool ArmorPredictor::updatePredictor(bool is_spinning, TargetInfo target)
    {
        Eigen::Vector3d state = singer_model_.x();
        singer_model_.x_ << target.xyz(0), target.xyz(1), target.xyz(2),
                            state(3),      state(4),      state(5),
                            state(6),      state(7),      state(8);
        return true;
    }

    // PostProcessInfo&& ArmorPredictor::postProcess(AutoaimMsg& target_msg)
    // {
    //     PostProcessInfo post_process_info;
    //     double delay_time = 0.0;
    //     post_process_info.track_3d_pos = {target_msg.aiming_point_world.x, target_msg.aiming_point_world.y, target_msg.aiming_point_world.z};
    //     bool is_spinning = target_msg.is_spinning;
        
    //     post_process_info.pred_3d_pos = predict(target_msg, target_msg.timestamp, delay_time);
    //     // 1.根据目标速度判定是否击打或切换目标
    //     if ((history_vel_[1][0] > 2.0 || history_vel_[1][0] > 2.0) && (predict_vel_[1][0] > 2.0 || predict_vel_[1][0] > 2.0)
    //         && (history_vel_[0][0] > 2.0 || history_vel_[0][0] > 2.0) && (predict_vel_[0][0] > 2.0 || predict_vel_[0][0] > 2.0))
    //     {
    //         post_process_info.switch_target = true;
    //         post_process_info.is_shooting = false;
    //     }
    //     else if ((history_vel_[1][0] > 0.8 || history_vel_[1][0] > 0.8) && (predict_vel_[1][0] > 0.8 || history_vel_[1][0] > 0.8)
    //         && (history_vel_[0][0] > 0.8 || history_vel_[0][0] > 0.8) && (predict_vel_[0][0] > 0.8 || history_vel_[0][0] > 0.8))
    //     {
    //         post_process_info.switch_target = false;
    //         post_process_info.is_shooting = false;
    //     }
    //     else
    //     {
    //         // 2.根据目标的预测误差进行判断
    //         if (!filter_disabled_ && cur_pred_error_ != 0.0)
    //         {
    //             if(cur_pred_error_ > 0.2)
    //             {
    //                 post_process_info.switch_target = true;
    //                 post_process_info.is_shooting = false;
    //             }
    //             if(cur_pred_error_ > 0.1 && 
    //                 ((is_singer_init_[0] && is_singer_init_[1] && is_singer_init_[2]) || is_imm_init_))
    //             {
    //                 post_process_info.is_shooting = false;
    //                 post_process_info.switch_target = false;
    //             }
    //         }

    //         // 3.根据目标的预测位置信息
    //         double meas_y = post_process_info.track_3d_pos[0] + history_vel_[0][0] * delay_time / 1e9 + 0.5 * history_acc_[0][0] * pow(delay_time / 1e9, 2);
    //         double pred_y = post_process_info.track_3d_pos[0] + predict_vel_[0][0] * delay_time / 1e9 + 0.5 * predict_acc_[0][0] * pow(delay_time / 1e9, 2);
    //         double meas_x = post_process_info.track_3d_pos[1] + history_vel_[1][0] * delay_time / 1e9 + 0.5 * history_acc_[1][0] * pow(delay_time / 1e9, 2);
    //         double pred_x = post_process_info.track_3d_pos[1] + predict_vel_[1][0] * delay_time / 1e9 + 0.5 * predict_acc_[1][0] * pow(delay_time / 1e9, 2);
            
    //         double error = sqrt(pow(post_process_info.pred_3d_pos[1] - meas_x, 2) + pow(post_process_info.pred_3d_pos[1] - pred_x, 2)) / 2.0;
    //         if(post_process_info.pred_3d_pos[0] > 4.5 && error > 0.15)
    //         {
    //             post_process_info.is_shooting = false;
    //             post_process_info.switch_target = true;
    //         }
    //     }
    //     return std::move(post_process_info);
    // }

    /**
     * @brief 对目标位置进行预测
     * 
     * @param target_msg 目标message
     * @param timestamp 本帧对应的时间戳
     * @param delay_time 休眠时间，对应于预测延迟量 
     * @param src 图像数据
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d ArmorPredictor::predict(TargetInfo target, double bullet_speed, double dt, double& delay_time, cv::Mat* src)
    {
        auto t1 = steady_clock_.now();
        Eigen::Vector3d result = {0.0, 0.0, 0.0};
        Eigen::Vector3d target_vel = {0.0, 0.0, 0.0};
        Eigen::Vector3d target_acc = {0.0, 0.0, 0.0};
        result = target.xyz;
        // cout << "xyz:(" << result(0) << ", " << result(1) << ", " << result(1) << ")" << endl;
        
        double pred_dt = dt;
        double spin_dt = dt;
        if (!target.is_target_lost)
        {
            if((int)history_info_.size() > 100)
            {
                history_info_.pop_front();
                history_info_.push_back(target);
            }
            else
            {
                history_info_.push_back(target);
            }

            double last_dist = history_info_.back().dist;
            pred_dt = (last_dist / bullet_speed) + predict_param_.shoot_delay / 1e3;
            delay_time = pred_dt;
            spin_dt = (last_dist / bullet_speed) + predict_param_.spin_shoot_delay / 1e3;
            final_target_ = target;
        }
            
        bool is_target_lost = target.is_target_lost;
        bool is_spinning = target.is_spinning;
        if (predictor_state_ == PREDICTING)
        {   //预测器处于预测击打阶段
            predictBasedSinger(is_target_lost, is_spinning, target.xyz, result, target_vel, target_acc, dt, pred_dt);
            // TargetInfo target_pred;
            // target_pred.xyz = result;
            // target_pred.timestamp = now;

            // if ((int)history_pred_.size() > 100)
            // {
            //     history_pred_.pop_front();
            //     history_pred_.push_back(target_pred);
            // }
            // else
            // {
            //     history_pred_.push_back(target_pred);
            // }
            // RCLCPP_INFO(logger_, "111");
        }
        else if (predictor_state_ == LOSTING)
        {
            predictBasedSinger(is_target_lost, is_spinning, target.xyz, result, target_vel, target_acc, dt, pred_dt);
            // TargetInfo target_losting_pred;
            // target_losting_pred.xyz = result;
            // target_losting_pred.timestamp = now;
            // if ((int)history_pred_.size() > 100)
            // {
            //     history_losting_pred_.pop_front();
            //     history_losting_pred_.push_back(target_losting_pred);
            // }
            // else
            // {
            //     history_losting_pred_.push_back(target_losting_pred);
            // }
            // RCLCPP_INFO(logger_, "222");
        }
        if (predictor_state_ == TRACKING)
        {   //预测器当前处于跟踪阶段
            final_target_ = target;
            result = target.xyz;
            // RCLCPP_INFO(logger_, "333");
            // return result;
        }

        // if (fitting_disabled_ && filter_disabled_)
        // {   //滤波和拟合均失效，使用当前目标位置信息
        //     result = Vector3d{target.xyz[0], target.xyz[1], target.xyz[2]};
        // }

        auto t2 = steady_clock_.now();
        double dr_ns = (t2 - t1).nanoseconds();
        if (debug_param_.print_delay)
        {
            RCLCPP_INFO(logger_, "target: x:%lf y:%lf z:%lf", target.xyz[0], target.xyz[1], target.xyz[2]);
            RCLCPP_INFO(logger_, "Predict time:%lfms", (dr_ns / 1e6));
            RCLCPP_INFO(logger_, "predict: x:%lf y:%lf z:%lf", result[0], result[1], result[2]);
        }
        final_target_.xyz = result;
        
        // cout << "xyz:(" << result(0) << ", " << result(1) << ", " << result(1) << ")" << endl; 
        // string pred_state = (predictor_state_ == TRACKING) ? "TRACKING" : ((predictor_state_ == PREDICTING) ? "PREDICTION": ((predictor_state_ == LOSTING) ? "LOSTING" : "LOST"));
        // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 40, "Predictor_State:%s", pred_state.c_str());
        return result;
    }

    bool ArmorPredictor::predictBasedSinger(bool is_target_lost, bool is_spinning, Eigen::Vector3d meas, Eigen::Vector3d& result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, double dt, double pred_dt)
    {
        // cout << "xyz:(" << meas(0) << ", " << meas(1) << ", " << meas(1) << ")" << endl;
        bool is_available;
        if (!is_target_lost)
        {
            if (!is_singer_init_)
            {
                singer_model_.x_ << meas(0), meas(1), meas(2), 0, 0, 0, 0, 0, 0;
                is_available = false;
                // predictor_state_ = TRACKING;
                is_singer_init_ = true;
            }
            else
            {
                singer_model_.updateF(singer_model_.F_, dt);
                singer_model_.updateJf();
                singer_model_.updateQ(dt);
                singer_model_.Predict(dt);
                // Eigen::MatrixXd stateCovPre = singer_model_.P();
                // Eigen::MatrixXd statePre = singer_model_.x();
                
                Eigen::VectorXd measurement = Eigen::VectorXd(3);
                measurement << meas(0), meas(1), meas(2);
                singer_model_.updateH(singer_model_.H_, dt);
                singer_model_.updateJh();
                singer_model_.Update(measurement);

                // Eigen::MatrixXd predictState(9, 1);
                Eigen::VectorXd State = singer_model_.x();
                
                updateVel({State(3), State(4), State(5)});
                updateAcc({State(6), State(7), State(8)});

                Eigen::MatrixXd F(9, 9);
                singer_model_.updateF(F, pred_dt);
                Eigen::MatrixXd Control(9, 3);
                singer_model_.updateC(Control, pred_dt);

                // if (history_acc_[0] == 0.0)
                //     singer_model_.setQ(target_acc);
                // else
                //     singer_model_.setQ(State[2]);

                Eigen::Vector3d acc = {State(6), State(7), State(8)};
                VectorXd pred = F * State + Control * acc;
                result = {pred(0), pred(1), pred(2)};

                // cout << "xyz:(" << result(0) << ", " << result(1) << ", " << result(1) << ")" << endl;
                // if (checkDivergence(statePre, stateCovPre, singer_model_.H_, singer_model_.R_, measurement) || abs(result - meas) > 0.85)
                // {
                //     RCLCPP_WARN(logger_, "Filter is diverging...");
                //     // singer_model_->P_ = singer_model_.P();
                //     // is_singer_init_ = false;
                //     is_available = false;   
                // } 
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
                //     is_singer_init_ = false;
                //     result = meas;
                // }
                is_available = true;
            }
        }
        else if (predictor_state_ == LOSTING)
        {
            //对目标可能出现的位置进行预测
            singer_model_.updateF(singer_model_.F_, dt);
            singer_model_.updateJf();
            singer_model_.updateQ(dt);
            singer_model_.Predict(dt);

            Eigen::VectorXd State = singer_model_.x();
           
            updateVel({State(3), State(4), State(5)});
            updateAcc({State(6), State(7), State(8)});
            
            Eigen::MatrixXd F(9, 9);
            singer_model_.updateF(F, pred_dt);
            Eigen::MatrixXd Control(9, 3);
            singer_model_.updateC(Control, pred_dt);

            Eigen::Vector3d acc = {State(6), State(7), State(8)};
            VectorXd pred = F * State + Control * acc;
            
            result = {pred(0), pred(1), pred(2)};
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
    PredictStatus ArmorPredictor::predictBasedImm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, int64_t timestamp)
    {
        PredictStatus is_available;
        double dt = timestamp / 1e9;   
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
} //namespace armor_processor