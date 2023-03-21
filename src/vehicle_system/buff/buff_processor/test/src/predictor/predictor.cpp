/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:50:43
 * @LastEditTime: 2023-03-20 20:40:43
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/test/src/predictor/predictor.cpp
 */
#include "../../include/predictor/predictor.hpp"

namespace buff_processor
{
    BuffPredictor::BuffPredictor()
    : logger_(rclcpp::get_logger("buff_predictor"))
    {
        is_params_confirmed = false;
        last_mode = mode = -1;
        angle_offset_ = 0.0;
        sign_ = 0;
        is_switched_ = false;
        error_cnt_ = 0;
        rmse_error_cnt_ = 0;
        last_pred_angle_ = 0.0;
        
        params[0] = 0;
        params[1] = 0; 
        params[2] = 0; 
        params[3] = 0;

        try
        {
            YAML::Node config = YAML::LoadFile(predictor_param_.pf_path);
            pf_param_loader.initParam(config, "buff");
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
        TargetInfo target = 
        {
            buff_msg.target_switched, 
            buff_msg.angle,
            0.0,
            buff_msg.delta_angle,
            buff_msg.angle_offset,
            buff_msg.timestamp
        };

        if(mode != last_mode)
        {   //模式切换重置预测
            int local_mode = mode;
            last_mode = local_mode;
            angle_offset_ = 0.0;
            sign_ = 0;
            is_switched_ = false;
            history_info.clear();
            pf.initParam(pf_param_loader);
            is_params_confirmed = false;
            last_pred_angle_ = 0.0;
        }

        if (history_info.size() == 0 || (target.timestamp - history_info.front().timestamp) / 1e6 >= predictor_param_.max_timespan)
        {   //当时间跨度过长视作目标已更新，需清空历史信息队列
            history_info.clear();
            sign_ = 0;
            angle_offset_ = 0.0;
            is_switched_ = false;
            error_cnt_ = 0;
            base_angle_ = target.abs_angle;
            target.relative_angle = 0.0;
            history_info.push_back(target);
            origin_timestamp_ = history_info.front().timestamp;
            last_pred_angle_ = 0.0;
            params[0] = 0.01;
            params[1] = 0.01; 
            params[2] = 0.01; 
            params[3] = 0.01;
            pf.initParam(pf_param_loader);
            last_target = target;
            is_params_confirmed = false;
            return false;
        }

        //输入数据前进行滤波
        auto is_ready = pf.is_ready;
        Eigen::VectorXd measure(1);
        measure << buff_msg.delta_angle;
        pf.update(measure);
        
        if (is_ready)
        {
            auto predict = pf.predict();
            target.delta_angle = predict[0];
        }
        target.delta_angle = (target.delta_angle / buff_msg.delta_angle > 0) ? target.delta_angle : buff_msg.delta_angle;
        
        // mutex_.lock();
        int fitting_lens = 45;
        if (!is_direction_confirmed)
        {
            if ((int)delta_angle_vec_.size() < 100)
                delta_angle_vec_.push_back(target.delta_angle);
            else
            {
                delta_angle_vec_.resize((int)(delta_angle_vec_.size() / 2));
            }
        }

        if((int)(history_info.size()) < fitting_lens)
        {
            
            target.relative_angle = history_info.back().relative_angle + abs(target.delta_angle);
            history_info.push_back(target);
            last_target = target;
            return false;
        }
        else
        {
            history_info.pop_front();
            double bAngle = history_info[0].relative_angle;
            for(auto &target_info : history_info)
                target_info.relative_angle -= bAngle;
            target.relative_angle = history_info.back().relative_angle + abs(target.delta_angle);
            history_info.push_back(target);
        }
        // mutex_.unlock();

        double pe = phase_;
        last_phase_ = pe;

        double rotate_speed_sum = 0.0;
        double rotate_speed_ave = 0.0;
        
        auto origin_target = history_info[0];
        for(auto target_info : history_info)
        {
            double dAngle = (target_info.relative_angle - origin_target.relative_angle);
            if((target_info.timestamp - origin_target.timestamp) != 0)
                rotate_speed_sum += (dAngle / (target_info.timestamp - origin_target.timestamp) * 1e9);
            origin_target = target_info;
        }
       
        rotate_speed_ave = rotate_speed_sum / ((history_info.size() - 1));
        ave_speed_ = rotate_speed_ave;

        int dir_cnt = 0;
        if (!is_direction_confirmed)
        {
            for (auto delta_angle : delta_angle_vec_)
            {
                // cout << "dAngle:" << delta_angle << " ";
                if (delta_angle > 0)
                    dir_cnt += 1;
                else if (delta_angle < 0)
                    dir_cnt -= 1;
            }
            // cout << endl;
            sign_ = (dir_cnt > 0) ? 1 : -1;
            // if (delta_angle_vec_.size() > 300)
            //     is_direction_confirmed = true;
        }
        
        int lmode = mode;
        RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "mode: %d", lmode);

        //曲线拟合
        if (mode == 0)
        {   //小符，计算平均角度差
            params[3] = rotate_speed_ave;  //TODO:小能量机关转速10RPM
            is_params_confirmed = true;
            RCLCPP_INFO(logger_, "Average rotate speed: %lf", rotate_speed_ave);
        }
        else if (mode == 1)
        {
            if(!is_params_confirmed)
            {
                last_pred_angle_ = 0.0;

                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary;       // 优化信息
                // options.max_num_iterations = 100;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = false;

                double params_fitting[4] = {params[0], params[1], params[2], params[3]};

                double origin_timestamp = origin_timestamp_;
                origin_timestamp = history_info.front().timestamp;

                // for(auto target_info : history_info)
                for(int ii = 0; ii < (int)(history_info.size()); ii += 1)
                {
                    // RCLCPP_INFO(logger_, "relative_angle: %lf timestamp: %lf", target_info.relative_angle, ((target_info.timestamp - origin_timestamp) / 1e9));

                    problem.AddResidualBlock
                    (
                        new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 4>
                        (
                            // new CurveFittingCost(target_info.relative_angle, (target_info.timestamp - origin_timestamp) / 1e9)
                            new CurveFittingCost(history_info[ii].relative_angle, (history_info[ii].timestamp - origin_timestamp) / 1e9)
                        ),
                        new ceres::CauchyLoss(0.5),
                        params_fitting
                    );
                }

                //设置上下限
                problem.SetParameterLowerBound(params_fitting, 0, 0.1); //a(0.780~1.045)
                problem.SetParameterUpperBound(params_fitting, 0, 1.9); 
                problem.SetParameterLowerBound(params_fitting, 1, 1.0); //w(1.884~2.000)
                problem.SetParameterUpperBound(params_fitting, 1, 2.8);
                problem.SetParameterLowerBound(params_fitting, 2, -2 * CV_PI); //θ
                problem.SetParameterUpperBound(params_fitting, 2, 2 * CV_PI);
                problem.SetParameterLowerBound(params_fitting, 3, 0.4); //b=2.090-a
                problem.SetParameterUpperBound(params_fitting, 3, 1.9);

                //参数求解
                ceres::Solve(options, &problem, &summary);
                
                //计算拟合后曲线的RMSE指标
                // auto last_rmse = evalRMSE(params);
                auto rmse = evalRMSE(params_fitting);
                RCLCPP_INFO(logger_, "RMSE: %lf", rmse);

                if (rmse < 5.5)
                {
                    // mutex_.lock();
                    memcpy(params, params_fitting, sizeof(params));
                    phase_ = params_fitting[2];
                    is_params_confirmed = true;
                    for (auto param : params)
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

                // double omega = params[1];
                double phase = params[2];
                // double const_term = params[4];

                double origin_timestamp = origin_timestamp_;
                origin_timestamp = history_info.front().timestamp;
                // for (auto target_info : history_info)
                for(int ii = 0; ii < history_info.size(); ii += 1)
                {
                    problem.AddResidualBlock( // 向问题中添加误差项
                    // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1> 
                        ( 
                            new CURVE_FITTING_COST_PHASE(history_info[ii].relative_angle, (history_info[ii].timestamp - origin_timestamp) / 1e9,
                            params[0], 
                            params[1],
                            params[3])
                        ),
                        new ceres::CauchyLoss(0.5),
                        &phase
                        // &omega
                    );
                }

                //设置上下限
                // problem.SetParameterLowerBound(&omega, 0, 0.2); //w(1.884~2.000)
                // problem.SetParameterUpperBound(&omega, 0, 2.9);
                // problem.SetParameterUpperBound(&phase, 0, 2 * CV_PI);
                // problem.SetParameterLowerBound(&phase, 0, -(2 * CV_PI));

                ceres::Solve(options, &problem, &summary);

                // mutex_.lock();
                double params_new[4] = {params[0], params[1], phase, params[3]};
                
                // std::cout << "omega:" << omega << " phase:" << phase << std::endl;
                
                // auto old_rmse = evalRMSE(params);
                auto new_rmse = evalRMSE(params_new);
                // if(new_rmse < old_rmse)
                // if(new_rmse < 5.5)
                // {   
                    // params[1] = omega;
                    params[2] = phase;
                    phase_ = phase;
                    // std::cout << "phase:" << phase << " const_term:" << const_term << " new_rmse:" << new_rmse << std::endl;
                    // std::cout << "omega:" << omega << " phase:" << phase << " new_rmse:" << new_rmse << std::endl;
                    std::cout << "phase:" << phase << " new_rmse:" << new_rmse << std::endl;
                // }
                // else
                //     is_params_confirmed = false;

                // mutex_.unlock();
            }
        }
        else
        {
            // mutex_.lock();
            is_params_confirmed = false;
            // mutex_.unlock();
            
            return false;
        }
        return true;
    }

    bool BuffPredictor::predict(BuffMsg buff_msg, double dist, double &result)
    {
        curveFitting(buff_msg);
        double delay = (mode == 1 ? predictor_param_.delay_big : predictor_param_.delay_small);
        double delta_time_estimate = ((double)dist / predictor_param_.bullet_speed) * 1e3 + delay;
        delta_time_estimate = 500;
        
        if (is_params_confirmed)
        {
            if (mode == 0)
            {
                mutex_.lock();
                if(sign_ == 1)
                    result = abs(params[3] * (delta_time_estimate / 1e3));
                else if(sign_ == -1)
                    result = -abs(params[3] * (delta_time_estimate / 1e3));
                int direction = sign_;
                // RCLCPP_INFO(logger_, "rotate_direction: %d", direction);
                mutex_.unlock();
            }
            else if (mode == 1)
            {
                double timespan = (buff_msg.timestamp - origin_timestamp_) / 1e6;
                timespan = (buff_msg.timestamp - history_info.front().timestamp) / 1e6;

                last_pred_angle_ = cur_pred_angle_;
                double cur_pre_angle = calPreAngle(params, timespan / 1e3);
                cur_pred_angle_ = cur_pre_angle;
                
                //FIXME:测量角度增量需要与预测角度增量的时间戳对齐
                double delta_pre_angle = abs(cur_pred_angle_ - last_pred_angle_) * (180 / CV_PI);
                double error = 0.0;
                if ((int)pred_info_queue_.size() > 0)
                {
                    if (history_info.back().timestamp / 1e9 >= pred_info_queue_.front().timestamp)
                    {
                        double delta_meas_angle = (history_info.back().relative_angle - history_info[history_info.size() - 2].relative_angle) * (180 / CV_PI);
                        double meas_v = (delta_meas_angle / (180 / CV_PI)) / (history_info.back().timestamp - history_info[history_info.size() - 2].timestamp) * 1e9;
                        double pred_v = (pred_info_queue_.front().angle_offset / pred_info_queue_.front().timestamp);
                        pred_info_queue_.pop();
                        error = abs(pred_v - meas_v);
                    }
                }
                if (error > 0.15)
                    error_cnt_++;

                // RCLCPP_INFO(logger_, "last_pred_angle:%lf cur_pred_angle:%lf\n delta_meas_angle:%lf delta_pre_angle:%lf error:%lf", last_pred_angle_, cur_pred_angle_, delta_meas_angle, delta_pre_angle, error);
                RCLCPP_WARN(logger_, "Prediction error cnt:%d", error_cnt_);
                if (error_cnt_ > 5 || error > 0.25)
                {
                    is_params_confirmed = false;
                    is_direction_confirmed = false;
                    error_cnt_ = 0;
                    result = 0.0;
                    last_pred_angle_ = 0.0;
                    return true;
                }

                // double meas_v = (delta_meas_angle / (180 / CV_PI)) / (history_info.back().timestamp - history_info[history_info.size() - 2].timestamp) * 1e9;
                // cout << "meas_v:" << meas_v << endl;
                double time_estimate = delta_time_estimate + timespan;
                double pre_dt = (time_estimate / 1e3);
                double pre_angle = calPreAngle(params, pre_dt);
                if (sign_ == 1)
                    result = abs(pre_angle - cur_pre_angle);
                else if (sign_ == -1)
                    result = -abs(pre_angle - cur_pre_angle);
                // double meas_pre_angle = meas_v * (delta_time_estimate / 1e3);
                // cout << "meas_offset:" << meas_pre_angle << endl;
                // if (result < meas_pre_angle * 0.5 || result > meas_pre_angle * 2.0)
                //     result = meas_pre_angle;

                is_last_result_exist_ = false;
                last_last_result_ = last_result_;
                last_result_ = result;
                lost_cnt_ = 0;
                is_last_result_exist_ = true;
                PredInfo pred_info = {pre_dt, result};
                pred_info_queue_.push(pred_info);

                // if(last_angle_offset_ != 0)
                // {
                //     double angle_shift = 0.0;
                //     double time_shift = 0.0;
                //     if(predict_info_.size() > 1)
                //     {
                //         angle_shift = predict_info_.back().relative_angle -predict_info_[predict_info_.size() - 2].relative_angle;
                //         time_shift = (predict_info_.back().timestamp - predict_info_[predict_info_.size() - 2].timestamp) / 1e6;
                //     }
                //     else
                //     {
                //         angle_shift = history_info.back().relative_angle - history_info[history_info.size() - 2].relative_angle;
                //         time_shift = (history_info.back().timestamp - history_info[history_info.size() - 2].timestamp) / 1e6;
                //     }

                //     if(abs(result - last_angle_offset_) < (2.2 * (delta_time_estimate / time_shift) * angle_shift))
                //         last_angle_offset_ = result;
                //     else
                //         result = last_angle_offset_;
                // }
                // else
                //      last_angle_offset_ = result;

                // int direction = sign_;
                // RCLCPP_INFO(logger_, "sign: %d cur_time: %lf pre_time: %lf cur_angle: %lf pre_angle: %lf", direction, (timespan / 1e3), pre_dt, history_info.back().relative_angle * (180 / CV_PI), pre_angle * (180 / CV_PI));
                // RCLCPP_INFO(logger_, "sign: %d cur_time: %lf pre_time: %lf cur_angle: %lf pre_angle: %lf", direction, (timespan / 1e3), pre_dt, history_info.back().relative_angle * (CV_PI / 180), pre_angle * (CV_PI / 180));
                // mutex_.unlock();
                // if(result < 0.0)
                //     return false;
            }
        }
        else
        {
            // if(history_info.size() > 4)
            // {
            //     // double delta_time = (history_info[history_info.size() - 1].timestamp - history_info[history_info.size() - 2].timestamp) / 1e9;
            //     // if(sign_ == 1)
            //     // {
            //     //     result = abs(ave_speed_ * (delta_time_estimate / 1e3));
            //     // }
            //     // else if(sign_ == -1)
            //     //     result = -abs(ave_speed_ * (delta_time_estimate / 1e3));
            //     // last_result_ = result;

            //     // printf("last_offset:%lf cur_offset:%lf", last_result_, result);
            // }
            // else
            //     return false;
            // if(is_last_result_exist_ && lost_cnt_ < 2)
            // {
            //     result = (last_last_last_result_ * 0.45 + last_result_ * 0.45 + last_last_result_ * 0.10) / 6.0;
            //     lost_cnt_++;            
            // }
            // else
                // return false;
        }
        return true;
    }

    double BuffPredictor::calPreAngle(double* params, double timestamp)
    {
        // double pre_angle = -(params[0] / params[1]) * cos(params[1] * (timestamp + params[2])) + params[3] * timestamp + (params[0] / params[1]) * cos(params[1] * params[2]);
        double pre_angle = -(params[0] / params[1]) * cos(params[1] * timestamp + params[2]) + params[3] * timestamp + (params[0] / params[1]) * cos(params[2]);
        return std::move(pre_angle);
    }

    /**
     * @brief 计算RMSE指标
     * 
     * @param params 参数首地址指针
     * @return RMSE值 
     */
    double BuffPredictor::evalRMSE(double params[4])
    {
        double rmse_sum = 0;
        double rmse = 0;
        double origin_timestamp = history_info.front().timestamp;
        // origin_timestamp = origin_timestamp_;
        double oriAngle = history_info.front().relative_angle * (180 / CV_PI);
        double error = 0.0;
        error_cnt_ = 0;

        double last_pred_angle = 0.0;
        double last_meas_angle = 0.0;
        for(auto target_info : history_info)
        {
            auto t = (float)(target_info.timestamp - origin_timestamp) / 1e9;
            auto pred = -(params[0] / params[1]) * cos(params[1] * t + params[2]) + params[3] * t + (params[0] / params[1]) * cos(params[2]);
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
                    is_params_confirmed = false;
                    return 1e2;
                }
                RCLCPP_INFO(logger_, "t: %lf delta_pred_angle: %lf delta_meas_angle: %lf error: %lf", t, delta_pred_angle, delta_meas_angle, error);
            }
            
            // rmse_sum += pow((pred - measure), 2);
            rmse_sum += pow(error, 2);
        }
        rmse = sqrt(rmse_sum / (history_info.size() - 1));
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