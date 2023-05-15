/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2023-05-10 17:28:51
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.hpp"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor()
    : logger_(rclcpp::get_logger("armor_prediction"))
    {
    }

    ArmorPredictor::~ArmorPredictor()
    {
    }

    void ArmorPredictor::initPredictor(const vector<double>* uniform_ekf_param, const vector<double>* singer_ekf_param)
    {
        // EKF initialized.
        uniform_ekf_ = UniformModel(uniform_ekf_param, 6, 4, 0);
        singer_ekf_ = SingerModel(singer_ekf_param, 9, 3, 3);

        // 初始化滤波器状态
        resetPredictor();
    }

    void ArmorPredictor::initPredictor()
    {
        // EKF initialized.
        uniform_ekf_ = UniformModel(6, 4, 0);
        singer_ekf_ = SingerModel(9, 3, 3);

        // 初始化滤波器状态
        resetPredictor();
    }

    bool ArmorPredictor::resetPredictor()
    {
        is_singer_init_ = false;
        is_ekf_init_ = false;
        // history_state_vec_.clear();
        history_switched_state_vec_.clear();
        return true;
    }

    bool ArmorPredictor::updatePredictor(Eigen::VectorXd meas)
    {
        double pred_yaw = uniform_ekf_.x_(4);
        if (abs(pred_yaw - meas(3)) > 0.8)
        {
            Eigen::VectorXd uniform_state = uniform_ekf_.x();
            Vector6d switched_state = {uniform_state(0), uniform_state(1), uniform_state(2), uniform_state(3), uniform_state(4), uniform_state(5)};
            history_switched_state_vec_.clear();
            history_switched_state_vec_.emplace_front(switched_state);

            // Eigen::Vector2d circle3d = calcCircleCenter(meas);
            // Eigen::Vector2d pred_circle3d = {uniform_ekf_.x_(0), uniform_ekf_.x_(1)};
            // if ((pred_circle3d - circle3d).norm() > 0.15)
            // {
            //     uniform_ekf_.x_(0) = circle3d(0);
            //     uniform_ekf_.x_(1) = circle3d(1);
            // }
            uniform_ekf_.x_(2) = meas(2);
            uniform_ekf_.x_(4) = meas(3);
            // uniform_ekf_.x_(5) = 0.0;

            Eigen::VectorXd singer_state = singer_ekf_.x();
            singer_ekf_.x_ << uniform_ekf_.x_(0), uniform_ekf_.x_(1), uniform_ekf_.x_(2),
                              0.0,    0.0,    0.0,
                              0.0,    0.0,    0.0;
        }
        return true;
    }

    bool ArmorPredictor::predict(TargetInfo target, double dt, double pred_dt, double &delay_time, Eigen::Vector3d &pred_point3d, vector<Eigen::Vector4d> &armor3d_vec, cv::Mat *src)
    {
        SpinHeading spin_state = target.is_spinning ? (target.is_clockwise ? CLOCKWISE : COUNTER_CLOCKWISE) : UNKNOWN;

        // Eigen::Vector4d meas = {target.xyz(1), -target.xyz(0), target.xyz(2), (target.rangle > 0 ? (target.rangle - CV_PI / 2) : (CV_PI * 1.5 + target.rangle ))};
        Eigen::Vector4d meas = {target.xyz(0), target.xyz(1), target.xyz(2), target.rangle};
        
        // if (!target.is_spinning_switched)
        // {
        //     if (meas(3) > 1.2 && meas(3) < 1.8)
        //     {
        //         if (abs(meas(3) - last_meas_(3)) > 0.25)
        //         {
        //             meas = last_meas_;
        //         }
        //     }
        // }

        // if (is_ekf_init_)
        // {
        //     double rangle_diff = (last_rangle - target.rangle);
        //     if (abs(rangle_diff) > 0.6)
        //     {
        //         // cur_rangle_ += rangle_diff;
        //         cur_rangle_ += (rangle_diff > 0 ? CV_PI / 2 : -CV_PI / 2);
        //     }
        //     meas(3) += cur_rangle_;
        // }
        // else
        // {
        //     cur_rangle_ = 0.0;
        // }

        // cout << "cur_rangle:" << cur_rangle_ << " last_rangle:" << last_rangle << " target_rangle:" << target.rangle << endl;
        // last_rangle = target.rangle; 
        last_meas_ = meas;
        if (!predictBasedUniformModel(target.is_target_lost, spin_state, meas, dt, pred_dt, target.period, pred_point3d, armor3d_vec))
        {
            pred_point3d = target.xyz;
            return false;
        }

        return true;
    }

    bool ArmorPredictor::predictBasedUniformModel(bool is_target_lost, SpinHeading spin_state, Eigen::VectorXd meas, double dt, double pred_dt, double spinning_period, Eigen::Vector3d &result, vector<Eigen::Vector4d> &armor3d_vec)
    {
        bool is_pred_success = false;
        if (!is_ekf_init_)
        {   // 滤波器初始化
            Eigen::Vector2d circle_center = calcCircleCenter(meas);
            uniform_ekf_.x_ << circle_center(0), circle_center(1), meas(2), uniform_ekf_.radius_, meas(3), 0;
            is_ekf_init_ = true;
            is_pred_success = false;
            result = {circle_center(0), circle_center(1), meas(2)};
            // is_pred_success = predictBasedSinger(is_target_lost, result, result, {0, 0, 0}, {0, 0, 0}, dt, pred_dt);
            predictor_state_ = TRACKING;
            
            return is_pred_success;
        }

        // 预测
        uniform_ekf_.updateF(uniform_ekf_.F_, dt);
        uniform_ekf_.updateJf(uniform_ekf_.Jf_, dt);
        uniform_ekf_.Predict(dt);
        if (is_ekf_init_ && !is_target_lost)
        {
            Eigen::Vector3d cur_pos = {meas(0), meas(1), meas(2)};
            Eigen::Vector3d pred_pos = {uniform_ekf_.x_(0) + uniform_ekf_.x_(3) * sin(uniform_ekf_.x_(4)), uniform_ekf_.x_(1) - uniform_ekf_.x_(3) * cos(uniform_ekf_.x_(4)), uniform_ekf_.x_(2)};
            Eigen::Vector2d circle_center_sum = {0.0, 0.0};
            // int count = 0;
            double dis_diff = (pred_pos - pred_pos).norm();
            if (dis_diff > 0.30)
            {
                predictor_state_ = TRACKING;
                double radius = uniform_ekf_.x_(3);
                Eigen::Vector2d center2d = calcCircleCenter(meas);
                // uniform_ekf_.x_(0) = cur_pos(0) - radius * sin(meas(3));
                // uniform_ekf_.x_(1) = cur_pos(1) + radius * cos(meas(3));
                uniform_ekf_.x_(0) = center2d(0);
                uniform_ekf_.x_(1) = center2d(1);
                uniform_ekf_.x_(2) = (uniform_ekf_.x_(2) + last_state_(2)) / 2.0;
                uniform_ekf_.x_(4) = meas(3);

                // is_predictor_update_ = true;
                // circle_center_sum(0) += cur_pos(0) - radius * sin(meas(3));
                // circle_center_sum(1) += cur_pos(1) + radius * cos(meas(3));
                // ++count;
            }
            else if (dis_diff > 0.60)
            {
                is_singer_init_ = false;
                is_ekf_init_ = false;
                result = {meas(0), meas(1), meas(2)};
                predictor_state_ = TRACKING;
                return false;
            }

            // Eigen::Vector2d circle3d = calcCircleCenter(meas);
            // Eigen::Vector2d pred_circle3d = {uniform_ekf_.x_(0), uniform_ekf_.x_(1)};
            // if ((pred_circle3d - circle3d).norm() > 0.35)
            // {
            //     circle_center_sum(0) += circle3d(0);
            //     circle_center_sum(1) += circle3d(1);
            //     ++count;
            // }
            // if (count)
            // {
            //     Eigen::Vector2d circle_center_ave = (circle_center_sum / count);
            //     uniform_ekf_.x_(0) = circle_center_ave(0);
            //     uniform_ekf_.x_(1) = circle_center_ave(1);
            // }
        }

        if (is_target_lost && predictor_state_ == LOSTING)
        {   
            // cout << "losting..." << endl;
            Eigen::VectorXd state = uniform_ekf_.x();
            double radius = state(3);
            double rangle = state(4);
            double omega = state(5);
            if (radius < 0.18)
            {
                radius = last_state_(3) < 0.18 ? 0.18 : last_state_(3);
                uniform_ekf_.radius_ = radius;
                uniform_ekf_.x_(3) = radius;
            }
            else if (radius > 0.35)
            {
                radius = last_state_(3) > 0.35 ? 0.35 : last_state_(3);
                uniform_ekf_.radius_ = radius;
                uniform_ekf_.x_(3) = radius;
            }
            
            if (abs(state(2) - last_state_(2)) > 0.05)
            {
                uniform_ekf_.x_(2) = (state(2) + last_state_(2)) / 2.0;
                state(2) = uniform_ekf_.x_(2);
            }

            Eigen::Vector3d circle_center = {state(0), state(1), state(2)};
            double pred_rangle = rangle;

            result = {circle_center(0), circle_center(1), circle_center(2)};
            // Eigen::Vector4d circle_center3d = {circle_center(0), circle_center(1), circle_center(2), 0.0};
            // armor3d_vec.emplace_back(circle_center3d);

            for (int ii = 0; ii < 4; ii++)
            {
                Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                double pred_radius = radius;
                double pred_next_rangle = (pred_rangle + CV_PI / 2 * ii);
                double pred_x = result(0) + pred_radius * sin(pred_next_rangle);
                double pred_y = result(1) - pred_radius * cos(pred_next_rangle);
                double pred_z = result(2);
                if (history_switched_state_vec_.size() > 0)
                {
                    pred_radius = (ii % 2 == 0) ? radius : history_switched_state_vec_.front()(3);
                    pred_x = result(0) + pred_radius * sin(pred_next_rangle);
                    pred_y = result(1) - pred_radius * cos(pred_next_rangle);
                    pred_z = (ii % 2 == 0) ? result(2) : history_switched_state_vec_.front()(2);
                }
                armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                armor3d_vec.emplace_back(armor3d);
            }
            
            last_state_(0) = state(0);
            last_state_(1) = state(1);
            last_state_(2) = state(2);
            last_state_(3) = radius;
            last_state_(4) = state(4);
            last_state_(5) = state(5);
            
            RCLCPP_WARN_THROTTLE(
                logger_, 
                steady_clock_, 
                100, 
                "circle_center:(%.3f, %.3f %.3f) radius:%.3f theta:%.3f omega:%.3f",
                state(0), state(1), state(2), state(3), state(4), state(5)
            );
            is_pred_success = true;
        }
        else
        {   // 更新
            Eigen::VectorXd state = uniform_ekf_.x();
            double radius = state(3);
            double rangle = state(4);
            double omega = state(5);
            
            uniform_ekf_.updateH(uniform_ekf_.H_, dt);
            uniform_ekf_.updateJh(uniform_ekf_.Jh_, dt);
            uniform_ekf_.Update(meas);

            state = uniform_ekf_.x();
            radius = state(3);
            rangle = state(4);
            omega = state(5);

            if (radius < 0.18)
            {
                radius = last_state_(3) < 0.18 ? 0.18 : last_state_(3);
                uniform_ekf_.radius_ = radius;
                uniform_ekf_.x_(3) = radius;
            }
            else if (radius > 0.35)
            {
                radius = last_state_(3) > 0.35 ? 0.35 : last_state_(3);
                uniform_ekf_.radius_ = radius;
                uniform_ekf_.x_(3) = radius;
            }

            if (abs(state(2) - last_state_(2)) > 0.05)
            {
                uniform_ekf_.x_(2) = (state(2) + last_state_(2)) / 2.0;
                state(2) = uniform_ekf_.x_(2);
            }

            last_state_(0) = state(0);
            last_state_(1) = state(1);
            last_state_(2) = state(2);
            last_state_(3) = radius;
            last_state_(4) = state(4);
            last_state_(5) = state(5);

            Eigen::MatrixXd F(6, 6);
            uniform_ekf_.updateF(F, pred_dt);
            Eigen::VectorXd pred = F * state;
            
            radius = state(3);
            rangle = pred(4);
            omega = pred(5);

            Eigen::Vector3d circle_center = {state(0), state(1), state(2)};
            // Eigen::Vector3d meas_center = circle_center;
            // if (predictBasedSinger(is_target_lost, meas_center, circle_center, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, dt, pred_dt))
            // {
            //     // uniform_ekf_.x_(0) = circle_center(0);
            //     // uniform_ekf_.x_(1) = circle_center(1);
            //     // uniform_ekf_.x_(2) = state(2);
                
            //     // 超前预测
            //     Eigen::MatrixXd F(9, 9);
            //     singer_ekf_.updateF(F, pred_dt);
            //     Eigen::MatrixXd Control(9, 3);
            //     singer_ekf_.updateC(Control, pred_dt);

            //     Eigen::VectorXd State = singer_ekf_.x();
            //     Eigen::Vector3d acc = {State(6), State(7), State(8)};
            //     VectorXd pred = F * State + Control * acc;
            //     circle_center = {pred(0), pred(1), pred(2)};
            // }
            double pred_rangle = rangle;

            result = {circle_center(0), circle_center(1), circle_center(2)};
            // Eigen::Vector4d circle_center3d = {circle_center(0), circle_center(1), circle_center(2), 0.0};
            // armor3d_vec.emplace_back(circle_center3d);

            for (int ii = 0; ii < 4; ii++)
            {
                Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                double pred_radius = radius;
                double pred_next_rangle = (pred_rangle + CV_PI / 2 * ii);
                double pred_x = result(0) + pred_radius * sin(pred_next_rangle);
                double pred_y = result(1) - pred_radius * cos(pred_next_rangle);
                double pred_z = result(2);
                if (history_switched_state_vec_.size() > 0)
                {
                    pred_radius = (ii % 2 == 0) ? radius : history_switched_state_vec_.front()(3);
                    pred_x = result(0) + pred_radius * sin(pred_next_rangle);
                    pred_y = result(1) - pred_radius * cos(pred_next_rangle);
                    pred_z = (ii % 2 == 0) ? result(2) : history_switched_state_vec_.front()(2);
                }
                armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                armor3d_vec.emplace_back(armor3d);
            }

            RCLCPP_WARN_THROTTLE(
                logger_, 
                steady_clock_, 
                100, 
                "circle_center:(%.3f, %.3f %.3f) radius:%.3f theta:%.3f omega:%.3f \n meas:(%.3f, %.3f %.3f %.3f)",
                state(0), state(1), state(2), state(3), state(4), state(5),
                meas(0), meas(1), meas(2), meas(3)
            );
            is_pred_success = true;
        }
        return is_pred_success;
    }

    Eigen::Vector2d ArmorPredictor::calcCircleCenter(Eigen::VectorXd meas)
    {
        return Eigen::Vector2d{meas(0) - uniform_ekf_.radius_ * sin(meas(3)), meas(1) + uniform_ekf_.radius_ * cos(meas(3))};
    }

    double ArmorPredictor::calcCircleRadius(Eigen::Vector3d p1, Eigen::Vector3d p2)
    {
        return sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2) + pow(p1(2) - p2(2), 2));
    }

    bool ArmorPredictor::predictBasedSinger(bool is_target_lost, Eigen::Vector3d meas, Eigen::Vector3d& result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, double dt, double pred_dt)
    {
        bool is_available;
        if (!is_target_lost)
        {
            if (!is_singer_init_)
            {
                singer_ekf_.x_ << meas(0), meas(1), meas(2), 0, 0, 0, 0, 0, 0;
                is_available = false;
                is_singer_init_ = true;
            }
            else
            {
                singer_ekf_.updateF(singer_ekf_.F_, dt);
                singer_ekf_.updateJf();
                singer_ekf_.updateQ(dt);
                singer_ekf_.Predict(dt);
                // Eigen::MatrixXd stateCovPre = singer_ekf_.P();
                // Eigen::MatrixXd statePre = singer_ekf_.x();
                
                Eigen::VectorXd measurement = Eigen::VectorXd(3);
                measurement << meas(0), meas(1), meas(2);
                singer_ekf_.updateH(singer_ekf_.H_, dt);
                singer_ekf_.updateJh();
                singer_ekf_.Update(measurement);

                // Eigen::MatrixXd predictState(9, 1);
                Eigen::VectorXd State = singer_ekf_.x();
                result = {State(0), State(1), State(2)};

                updateVel({State(3), State(4), State(5)});
                updateAcc({State(6), State(7), State(8)});
                
                // Eigen::MatrixXd F(9, 9);
                // singer_ekf_.updateF(F, pred_dt);
                // Eigen::MatrixXd Control(9, 3);
                // singer_ekf_.updateC(Control, pred_dt);

                // if (history_acc_[0] == 0.0)
                //     singer_ekf_.setQ(target_acc);
                // else
                //     singer_ekf_.setQ(State[2]);

                // Eigen::Vector3d acc = {State(6), State(7), State(8)};
                // VectorXd pred = F * State + Control * acc;
                // result = {pred(0), pred(1), pred(2)};

                // if (checkDivergence(statePre, stateCovPre, singer_ekf_.H_, singer_ekf_.R_, measurement) || abs(result - meas) > 0.85)
                // {
                //     RCLCPP_WARN(logger_, "Filter is diverging...");
                //     // singer_kf_->P_ = singer_ekf_.P();
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
            return is_available;
        }
        else if (predictor_state_ == LOSTING)
        {
            //对目标可能出现的位置进行预测
            singer_ekf_.updateF(singer_ekf_.F_, dt);
            singer_ekf_.updateJf();
            singer_ekf_.updateQ(dt);
            singer_ekf_.Predict(dt);

            Eigen::VectorXd State = singer_ekf_.x();
            result = {State(0), State(1), State(2)};
           
            updateVel({State(3), State(4), State(5)});
            updateAcc({State(6), State(7), State(8)});
            
            // Eigen::MatrixXd F(9, 9);
            // singer_ekf_.updateF(F, pred_dt);
            // Eigen::MatrixXd Control(9, 3);
            // singer_ekf_.updateC(Control, pred_dt);

            // Eigen::Vector3d acc = {State(6), State(7), State(8)};
            // VectorXd pred = F * State + Control * acc;
            // result = {pred(0), pred(1), pred(2)};
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
     * @return bool
     */
    bool ArmorPredictor::predictBasedImm(TargetInfo target, Eigen::Vector3d &result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, int64_t timestamp)
    {
        bool is_available;
        double dt = timestamp / 1e9;
        if (!is_imm_init_)
        {
            Eigen::VectorXd x(6);
            x << target.xyz[0], target.xyz[1], target_vel[0], target_vel[1], 0, 0;
            imm_ = model_generator_.generateIMMModel(x, dt);
            is_available = false;
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
            is_available = true;
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
        // 计算最大迭代次数
        auto max_iter = int(history_info_.size() - start_idx) - predict_param_.window_size + 1;
        Eigen::Vector3d total_sum = {0, 0, 0};
        if (max_iter == 0 || start_idx < 0)
            return history_info_.back().xyz;

        for (int i = 0; i < max_iter; i++)
        {
            Eigen::Vector3d sum = {0, 0, 0};
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
    double ArmorPredictor::evalRMSE(double *params)
    {
        double rmse_sum = 0;
        double rmse = 0;
        double pred = 0;
        double measure = 0;
        for (auto &target_info : history_info_)
        {
            auto t = (double)(target_info.timestamp) / 1e9;
            pred = params[0] * t + params[1]; // f(t)=kt+b
            measure = target_info.xyz[1];
        }
        rmse = sqrt(rmse_sum / history_info_.size());
        return rmse;
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
            for (int ii = 0; ii < (int)history_info_.size(); ii++)
            {
                if (ii != (int)(history_info_.size() - 1))
                {
                    if (pre_info.timestamp >= history_info_[ii].timestamp && pre_info.timestamp < history_info_[ii + 1].timestamp)
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
                else if (pre_info.timestamp < history_info_[ii].timestamp)
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
     * @brief 前哨站旋转预测函数
     *
     * @param is_controlled 我方是否处于控制区，此时前哨站转速减半
     * @param target 目标信息
     * @param result 预测结果
     * @param time_estimated 时间延迟量
     * @return bool
     */
    bool ArmorPredictor::spinningPredict(bool is_controlled, TargetInfo &target, Eigen::Vector3d &result, int64_t time_estimated)
    {
        /**
         * @brief 前哨站旋转装甲运动预测（已知量：旋转半径&转速），考虑我方占领控制区旋转装甲板转速减半，应加入条件判断。
         *
         */
        // 轨迹拟合
        auto time_start = steady_clock_.now();
        double x0, y0, theta;

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.max_num_iterations = 20;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        Eigen::Vector3d xyz_sum = {0, 0, 0};
        if (!is_controlled)
        {
            for (auto &target_info : history_info_)
            {
                xyz_sum += target_info.xyz;
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 1)),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 1)),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta);
            }
        }
        else
        {
            for (auto &target_info : history_info_)
            {
                xyz_sum += target_info.xyz;
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 0.5)),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta);
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 0.5)),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta);
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

        bool is_available[2];
        // auto rmse = evalRMSE()
        is_available[0] = (summary.final_cost <= predict_param_.max_cost);
        is_available[1] = (summary.final_cost <= predict_param_.max_cost);
        double x_pred, y_pred;
        if (!is_controlled)
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
        return (is_available[0] && is_available[1]);
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
} // namespace armor_processor