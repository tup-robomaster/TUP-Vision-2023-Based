/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:50:43
 * @LastEditTime: 2023-06-04 00:04:04
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/predictor/predictor.cpp
 */
#include "../../include/predictor/predictor.hpp"

namespace buff_processor
{
    BuffPredictor::BuffPredictor()
    : logger_(rclcpp::get_logger("buff_predictor"))
    {
        is_params_confirmed_ = false;
        last_mode_ = mode_ = -1;
        angle_offset_ = 0.0;
        sign_ = 0;
        is_switched_ = false;
        error_cnt_ = 0;
        rmse_error_cnt_ = 0;
        last_pred_angle_ = 0.0;
        
        params_[0] = 0;
        params_[1] = 0; 
        params_[2] = 0; 
        params_[3] = 0;

        try
        {
            YAML::Node config = YAML::LoadFile(predictor_param_.pf_path);
            pf_param_loader_.initParam(config, "buff");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Error while initializing pf param: %s", e.what());
        }
    }

    BuffPredictor::~BuffPredictor()
    {
    }

    bool BuffPredictor::curveFitting(BuffMsg& buff_msg)
    {
        BuffAngleInfo target = 
        {
            buff_msg.target_switched, 
            buff_msg.angle,
            0.0,
            buff_msg.delta_angle,
            buff_msg.angle_offset,
            buff_msg.timestamp
        };

        if(mode_ != last_mode_)
        {   //模式切换重置预测
            last_mode_ = mode_;
            angle_offset_ = 0.0;
            sign_ = 0;
            is_switched_ = false;
            history_info_.clear();
            pf_.initParam(pf_param_loader_);
            is_params_confirmed_ = false;
            last_pred_angle_ = 0.0;
        }

        if (history_info_.size() == 0 || (target.timestamp - history_info_.front().timestamp) / 1e6 >= predictor_param_.max_timespan)
        {   //当时间跨度过长视作目标已更新，需清空历史信息队列
            history_info_.clear();
            sign_ = 0;
            angle_offset_ = 0.0;
            is_switched_ = false;
            error_cnt_ = 0;
            base_angle_ = target.abs_angle;
            target.relative_angle = 0.0;
            history_info_.push_back(target);
            last_pred_angle_ = 0.0;
            params_[0] = 0.01;
            params_[1] = 0.01; 
            params_[2] = 0.01; 
            params_[3] = 0.01;
            pf_.initParam(pf_param_loader_);
            last_target_ = target;
            is_params_confirmed_ = false;
            return false;
        }

        //输入数据前进行滤波
        // auto is_ready = pf_.is_ready;
        // Eigen::VectorXd measure(1);
        // measure << buff_msg.delta_angle;
        // pf_.update(measure);
        
        // if (is_ready)
        // {
        //     auto predict = pf_.predict();
        //     target.delta_angle = predict[0];
        // }
        // target.delta_angle = (target.delta_angle / buff_msg.delta_angle > 0) ? target.delta_angle : buff_msg.delta_angle;
        cout << "target.delta_angle: " << target.delta_angle << endl;

        int fitting_lens = 50;
        if (!is_direction_confirmed_)
        {
            if ((int)delta_angle_vec_.size() < 200)
                delta_angle_vec_.push_back(target.delta_angle);
            else
            {
                // delta_angle_vec_.resize((int)(delta_angle_vec_.size() / 2));
                delta_angle_vec_.pop_front();
                delta_angle_vec_.push_back(target.delta_angle);
            }
        }

        if((int)(history_info_.size()) < fitting_lens)
        {
            target.relative_angle = history_info_.back().relative_angle + abs(target.delta_angle);
            history_info_.push_back(target);
            last_target_ = target;
            return false;
        }
        else
        {
            history_info_.pop_front();
            double bAngle = history_info_[0].relative_angle;
            for(auto &target_info : history_info_)
                target_info.relative_angle -= bAngle;
            target.relative_angle = history_info_.back().relative_angle + abs(target.delta_angle);
            history_info_.push_back(target);
        }

        double pe = phase_;
        last_phase_ = pe;

        double rotate_speed_sum = 0.0;
        double rotate_speed_ave = 0.0;
        
        BuffAngleInfo origin_target = history_info_.front();
        int count = 0;

        // cout << "RSpeed:" << endl;

        for (auto target_info : history_info_)
        {
            double dAngle = (target_info.relative_angle - origin_target.relative_angle);
            double dt = (target_info.timestamp - origin_target.timestamp) / 1e9;
            if (!iszero(dAngle) && !iszero(dt))
            { 
                // cout << dAngle << " ";
                double rspeed = (dAngle / dt);
                // cout << rspeed << " ";
                rotate_speed_sum += rspeed;
                ++count;
            }
            origin_target = target_info;
        }
        // cout << endl;

        rotate_speed_ave = rotate_speed_sum / count;
        ave_speed_ = rotate_speed_ave;

        int dir_cnt = 0;
        if (!is_direction_confirmed_)
        {
            // cout << "delta_angle:";
            for (auto delta_angle : delta_angle_vec_)
            {
                // cout << delta_angle << " ";

                if (delta_angle > 0)
                    dir_cnt += 1;
                else if (delta_angle < 0)
                    dir_cnt -= 1;
            }
            sign_ = (dir_cnt > 0) ? 1 : -1;
            // if (delta_angle_vec_.size() > 300)
            //     is_direction_confirmed_ = true;
        }

        RCLCPP_INFO_THROTTLE(
            logger_, 
            steady_clock_, 
            500, 
            "mode: %d",
            mode_
        );

        //曲线拟合
        if (mode_ == SMALL_BUFF)
        {   //小符，计算平均角度差
            params_[3] = rotate_speed_ave;  //TODO:小能量机关转速10RPM
            is_params_confirmed_ = true;
            
            RCLCPP_INFO_THROTTLE(
                logger_, 
                steady_clock_,
                100,
                "Average rotate speed: %.3f", 
                rotate_speed_ave
            );
        }
        else if (mode_ == BIG_BUFF)
        {
            if(!is_params_confirmed_)
            {
                last_pred_angle_ = 0.0;

                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary;       // 优化信息
                options.max_num_iterations = 100;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = false;

                double params_fitting[4] = {params_[0], params_[1], params_[2], params_[3]};

                double origin_timestamp = history_info_.front().timestamp;
                // for(auto target_info : history_info_)
                for(int ii = 0; ii < (int)(history_info_.size()); ii += 1)
                {
                    // RCLCPP_INFO(logger_, "relative_angle: %lf timestamp: %lf", target_info.relative_angle, ((target_info.timestamp - origin_timestamp) / 1e9));

                    problem.AddResidualBlock
                    (
                        new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 4>
                        (
                            // new CurveFittingCost(target_info.relative_angle, (target_info.timestamp - origin_timestamp) / 1e9)
                            new CurveFittingCost(history_info_[ii].relative_angle, (history_info_[ii].timestamp - origin_timestamp) / 1e9)
                        ),
                        new ceres::CauchyLoss(0.5),
                        params_fitting
                    );
                }

                //设置上下限
                // problem.SetParameterLowerBound(params_fitting, 0, 0.2); //a(0.780~1.045)
                // problem.SetParameterUpperBound(params_fitting, 0, 1.5); 
                // problem.SetParameterLowerBound(params_fitting, 1, 1.2); //w(1.884~2.000)
                // problem.SetParameterUpperBound(params_fitting, 1, 2.7);
                // problem.SetParameterLowerBound(params_fitting, 2, -2 * CV_PI); //θ
                // problem.SetParameterUpperBound(params_fitting, 2, 2 * CV_PI);
                // problem.SetParameterLowerBound(params_fitting, 3, 0.4); //b=2.090-a
                // problem.SetParameterUpperBound(params_fitting, 3, 1.9);

                //参数求解
                ceres::Solve(options, &problem, &summary);
                
                //计算拟合后曲线的RMSE指标
                // auto last_rmse = evalRMSE(params_);
                auto rmse = evalRMSE(params_fitting);
                RCLCPP_INFO(logger_, "RMSE: %lf", rmse);

                if (rmse < 5.5)
                {
                    // mutex_.lock();
                    memcpy(params_, params_fitting, sizeof(params_));
                    phase_ = params_fitting[2];
                    is_params_confirmed_ = true;
                    for (auto param : params_)
                        cout << param << " ";
                    std::cout << std::endl;
                    // mutex_.unlock();
                }
            }   
            else
            {
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary; // 优化信息
                // options.max_num_iterations = 30;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = false;

                // double omega = params_[1];
                double phase = params_[2];
                // double const_term = params_[4];

                double origin_timestamp = history_info_.front().timestamp;
                // for (auto target_info : history_info_)
                for(int ii = 0; ii < (int)history_info_.size(); ii += 1)
                {
                    problem.AddResidualBlock( // 向问题中添加误差项
                    // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1> 
                        ( 
                            new CURVE_FITTING_COST_PHASE(history_info_[ii].relative_angle, (history_info_[ii].timestamp - origin_timestamp) / 1e9,
                            params_[0], 
                            params_[1],
                            params_[3])
                        ),
                        new ceres::CauchyLoss(0.5),
                        &phase
                        // &omega
                    );
                }

                //设置上下限
                // problem.SetParameterUpperBound(&phase, 0, 2 * CV_PI);
                // problem.SetParameterLowerBound(&phase, 0, -(2 * CV_PI));

                ceres::Solve(options, &problem, &summary);

                // mutex_.lock();
                double params__new[4] = {params_[0], params_[1], phase, params_[3]};
                
                double old_rmse = evalRMSE(params_);
                double new_rmse = evalRMSE(params__new);
                std::cout << "phase:" << phase << " new_rmse:" << new_rmse << " old_rmse:" << old_rmse << std::endl;
               
                params_[2] = phase;
                phase_ = phase;
                // is_params_confirmed_ = false;
                
                // mutex_.unlock();
            }
        }
        else
        {
            is_params_confirmed_ = false;
            return false;
        }
        return true;
    }

    bool BuffPredictor::predict(BuffMsg buff_msg, BuffInfo& buff_info, double &result)
    {
        Eigen::Vector3d xyz = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
        double dist = xyz.norm();
        // double pred_dt = ((double)dist / predictor_param_.bullet_speed) * 1e3 + predictor_param_.shoot_delay;
        // pred_dt *= predictor_param_.delay_coeff;

        double shoot_delay = mode_ == SMALL_BUFF ? predictor_param_.delay_small : predictor_param_.delay_big; 
        double pred_dt = dist / predictor_param_.bullet_speed * 1e3 + shoot_delay;
        
        // 调试使用，实测需注释掉
        pred_dt = 500;
        
        curveFitting(buff_msg);
        if (is_params_confirmed_)
        {
            if (mode_ == SMALL_BUFF)
            {
                if(sign_ == 1)
                    result = abs(params_[3] * (pred_dt / 1e3));
                else if(sign_ == -1)
                    result = -abs(params_[3] * (pred_dt / 1e3));
                
                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    500,
                    "rotate_direction: %d", 
                    sign_
                );
            }
            else if (mode_ == BIG_BUFF)
            {
                // double timespan = (buff_msg.timestamp - history_info_.front().timestamp) / 1e6;
                double timespan = buff_msg.timestamp / 1e6;
                buff_info.abs_meas_angle = history_info_.back().relative_angle;

                last_pred_angle_ = cur_pred_angle_;
                double cur_pre_angle = calPreAngle(params_, timespan / 1e3);
                cur_pred_angle_ = cur_pre_angle;
                buff_info.abs_fitting_angle = cur_pre_angle;

                //FIXME:测量角度增量需要与预测角度增量的时间戳对齐
                double delta_pre_angle = abs(cur_pred_angle_ - last_pred_angle_) * (180 / CV_PI);
                double error = 0.0;
                if ((int)pred_info_queue_.size() > 0)
                {
                    if (history_info_.back().timestamp / 1e9 >= pred_info_queue_.front().timestamp)
                    {
                        double delta_meas_angle = (history_info_.back().relative_angle - history_info_[history_info_.size() - 2].relative_angle) * (180 / CV_PI);
                        double meas_v = (delta_meas_angle / (180 / CV_PI)) / (history_info_.back().timestamp - history_info_[history_info_.size() - 2].timestamp) * 1e9;
                        double pred_v = (pred_info_queue_.front().angle_offset / pred_info_queue_.front().timestamp);
                        pred_info_queue_.pop();
                        error = abs(pred_v - meas_v);
                    }
                }
                if (error >= 0.1)
                    error_cnt_++;

                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "last_pred_angle: %.3f cur_pred_angle: %.3f delta_pre_angle: %.3f error:%.3f", 
                    last_pred_angle_, cur_pred_angle_, delta_pre_angle, error
                );

                RCLCPP_WARN_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "Prediction error cnt: %d", 
                    error_cnt_
                );

                if (error_cnt_ >= 10 || error >= 0.25)
                {
                    is_params_confirmed_ = false;
                    is_direction_confirmed_ = false;
                    error_cnt_ = 0;
                    result = 0.0;
                    last_pred_angle_ = 0.0;
                }

                double time_estimate = pred_dt + timespan;
                double pre_dt = (time_estimate / 1e3);
                double pre_angle = calPreAngle(params_, pre_dt);
                buff_info.abs_pred_angle = pre_angle;

                if (sign_ == 1)
                    result = abs(pre_angle - cur_pre_angle);
                else if (sign_ == -1)
                    result = -abs(pre_angle - cur_pre_angle);
                
                // double meas_pre_angle = meas_v * (pred_dt / 1e3);
                // cout << "meas_offset:" << meas_pre_angle << endl;
                // if (result < meas_pre_angle * 0.5 || result > meas_pre_angle * 2.0)
                //     result = meas_pre_angle;

                lost_cnt_ = 0;
                PredInfo pred_info = {pre_dt, result};
                pred_info_queue_.push(pred_info);
            }
        }
        return true;
    }

    double BuffPredictor::calPreAngle(double* params_, double timestamp)
    {
        // double pre_angle = -(params_[0] / params_[1]) * cos(params_[1] * (timestamp + params_[2])) + params_[3] * timestamp + (params_[0] / params_[1]) * cos(params_[1] * params_[2]);
        double pre_angle = -(params_[0] / params_[1]) * cos(params_[1] * timestamp + params_[2]) + params_[3] * timestamp + (params_[0] / params_[1]) * cos(params_[2]);
        return std::move(pre_angle);
    }

    /**
     * @brief 计算RMSE指标
     * 
     * @param params_ 参数首地址指针
     * @return RMSE值 
     */
    double BuffPredictor::evalRMSE(double params_[4])
    {
        double rmse_sum = 0;
        double rmse = 0;
        double origin_timestamp = history_info_.front().timestamp;
        double oriAngle = history_info_.front().relative_angle * (180 / CV_PI);
        double error = 0.0;
        error_cnt_ = 0;

        double last_pred_angle = 0.0;
        double last_meas_angle = 0.0;
        for(auto target_info : history_info_)
        {
            auto t = (float)(target_info.timestamp - origin_timestamp) / 1e9;
            auto pred = -(params_[0] / params_[1]) * cos(params_[1] * t + params_[2]) + params_[3] * t + (params_[0] / params_[1]) * cos(params_[2]);
            pred = pred * (180 / CV_PI);
            auto measure = target_info.relative_angle * (180 / CV_PI);
            
            auto delta_pred_angle = abs(pred - last_pred_angle);
            auto delta_meas_angle = abs(measure - last_meas_angle);
            error = abs(delta_pred_angle - delta_meas_angle);
            last_pred_angle = pred;
            last_meas_angle = measure;

            //角度误差大于1度超过10帧即认为拟合失败
            if(error > 5.5)
            {
                error_cnt_++;
                if(error_cnt_ > 10)
                {
                    error_cnt_ = 0;
                    is_params_confirmed_ = false;
                    return 1e2;
                }
                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "t: %.3f delta_pred_angle: %.3f delta_meas_angle: %.3f error: %.3f", 
                    t, delta_pred_angle, delta_meas_angle, error
                );
            }
            
            // rmse_sum += pow((pred - measure), 2);
            rmse_sum += pow(error, 2);
        }
        rmse = sqrt(rmse_sum / (history_info_.size() - 1));
        return rmse;
    }

    /**
     * @brief 设置弹速
     * 
     * @param speed 传入弹速
     * @return true 
     * @return false 
     */
    bool BuffPredictor::setBulletSpeed(double speed)
    {
        predictor_param_.bullet_speed = speed;
        return true;
    }
} //namespace buff_processor