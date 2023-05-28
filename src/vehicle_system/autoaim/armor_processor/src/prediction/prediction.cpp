/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2023-05-28 20:53:43
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.hpp"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor(const PredictParam& predict_param, const DebugParam& debug_param)
    : logger_(rclcpp::get_logger("armor_prediction"))
    {
        predict_param_ = predict_param;
        debug_param_ = debug_param;
    }

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
        pred_state_vec_.clear();
        return true;
    }

    bool ArmorPredictor::updatePredictor(bool is_spinning, Eigen::VectorXd meas)
    {
        if (!is_spinning)
        {
            Eigen::VectorXd singer_state = singer_ekf_.x();
            singer_ekf_.x_ << meas(0),         meas(1),         meas(2),
                              singer_state(3), singer_state(4), singer_state(5),
                              singer_state(6), singer_state(7), singer_state(8);
            return true;
        }
        else
        {
            return updatePredictor(meas);
        }
    }

    bool ArmorPredictor::updatePredictor(Eigen::VectorXd meas)
    {
        double pred_yaw = uniform_ekf_.x_(4);
        if (abs(pred_yaw - meas(3)) > 0.6)
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
            
            if ((int)pred_state_vec_.size() > 0)
            {
                Eigen::VectorXd pred_state = pred_state_vec_.back();
                int scale = pred_state(3) / (2 * CV_PI);
                double pred_rangle = pred_state(3) - scale * (2 * CV_PI);
                
                uniform_ekf_.x_(2) = pred_state(2);
                uniform_ekf_.x_(4) = pred_rangle;

                RCLCPP_WARN_THROTTLE(
                    logger_, 
                    steady_clock_,
                    100,
                    "pred_rangle: (%.3f %3f) meas_rangle: %.3f",
                    pred_rangle, pred_state(3), meas(3)
                );
            }
            else
            {
                uniform_ekf_.x_(2) = meas(2);
                uniform_ekf_.x_(4) = meas(3);
                // uniform_ekf_.x_(5) = 0.0;
            }

            Eigen::VectorXd singer_state = singer_ekf_.x();
            singer_ekf_.x_ << singer_state(0), singer_state(1), uniform_ekf_.x_(2),
                              singer_state(3), singer_state(4), singer_state(5),
                              singer_state(6), singer_state(7), singer_state(8);
        }
        return true;
    }

    bool ArmorPredictor::predict(TargetInfo target, double dt, double pred_dt, double &delay_time, Eigen::Vector3d &pred_point3d, vector<Eigen::Vector4d> &armor3d_vec, cv::Mat *src)
    {
        SpinHeading spin_state = target.is_spinning ? (target.is_clockwise ? CLOCKWISE : COUNTER_CLOCKWISE) : UNKNOWN;
        Eigen::Vector4d meas = {target.xyz(0), target.xyz(1), target.xyz(2), target.rangle};
        bool is_target_lost = target.is_target_lost;
        
        is_outpost_mode_ = target.is_outpost_mode;
        outpost_angular_speed_ = spin_state == CLOCKWISE ? -outpost_angular_speed_ : outpost_angular_speed_;
        // cout << "meas_world:" << meas(0) << " " << meas(1) << " " << meas(2) << " " << meas(3) << endl;
        
        if ((last_spin_state_ == UNKNOWN && spin_state != UNKNOWN) || (last_spin_state_ != UNKNOWN && spin_state == UNKNOWN))
        {
            resetPredictor();
        }

        last_meas_ = meas;
        last_rangle = target.rangle;
        last_spin_state_ = spin_state;
        if (!target.is_spinning)
        {
            Eigen::Vector3d armor3d = {meas(0), meas(1), meas(2)};
            Eigen::Vector3d post_state = {0.0, 0.0, 0.0};

            pred_dt = predict_param_.delay_coeff * pred_dt;
            RCLCPP_WARN_ONCE(logger_, "delay_coeff:%.3f", predict_param_.delay_coeff);
            if (predictBasedSinger(is_target_lost, armor3d, post_state, {0, 0, 0}, {0, 0, 0}, dt, pred_dt))
            {
                pred_point3d = post_state;
            }
            else
            {
                RCLCPP_WARN(logger_, "KF based singer model prediction failed!!!");
                pred_point3d = armor3d;
                return false;
            }
        }
        else
        {
            Vector6d post_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // updatePredictor(meas);
            // meas = {meas(0), meas(1), meas(2), meas(3)};
            // cout << "meas_trans_world:" << meas(0) << " " << meas(1) << " " << meas(2) << " " << meas(3) << endl;

            if (predictBasedUniformModel(is_target_lost, spin_state, meas, dt, pred_dt, target.period, post_state))
            {
                Eigen::Vector3d center3d = {post_state(0), post_state(1), post_state(2)}; 
                pred_point3d = center3d;
                double radius = post_state(3);
                double pred_rangle = post_state(4);
                if (!is_outpost_mode_)
                {
                    if (!predictBasedSinger(is_target_lost, center3d, pred_point3d, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, dt, pred_dt))
                    {
                        RCLCPP_WARN(logger_, "KF based singer model prediction failed!!!");
                    }
                }
                pred_point3d(2) = meas(2);

                Eigen::Vector4d circle_center3d = {pred_point3d(0), pred_point3d(1), pred_point3d(2), 0.0};
                armor3d_vec.emplace_back(circle_center3d);
                if (!is_outpost_mode_)
                {
                    for (int ii = 0; ii < 4; ii++)
                    {
                        Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                        double pred_radius = radius;
                        double pred_next_rangle = (pred_rangle + CV_PI * 0.5 * ii);
                        double pred_x = pred_point3d(0) + pred_radius * sin(pred_next_rangle);
                        double pred_y = pred_point3d(1) - pred_radius * cos(pred_next_rangle);
                        double pred_z = pred_point3d(2);
                        if (history_switched_state_vec_.size() > 0)
                        {
                            pred_radius = (ii % 2 == 0) ? radius : history_switched_state_vec_.front()(3);
                            pred_x = pred_point3d(0) + pred_radius * sin(pred_next_rangle);
                            pred_y = pred_point3d(1) - pred_radius * cos(pred_next_rangle);
                            pred_z = (ii % 2 == 0) ? pred_point3d(2) : history_switched_state_vec_.front()(2);
                        }
                        armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                        armor3d_vec.emplace_back(armor3d);
                    }
                }
                else
                {
                    for (int ii = 0; ii < 3; ii++)
                    {
                        Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                        double pred_radius = radius;
                        double pred_next_rangle = (pred_rangle + (CV_PI * 2 / 3) * ii);
                        double pred_x = pred_point3d(0) + pred_radius * sin(pred_next_rangle);
                        double pred_y = pred_point3d(1) - pred_radius * cos(pred_next_rangle);
                        double pred_z = pred_point3d(2);
                        armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                        armor3d_vec.emplace_back(armor3d);
                    }
                }
            }
        }
        return true;
    }

    bool ArmorPredictor::predictBasedUniformModel(bool is_target_lost, SpinHeading spin_state, Eigen::VectorXd meas, double dt, double pred_dt, double spinning_period, Vector6d& post_state)
    {
        bool is_pred_success = false;
        if (!is_ekf_init_)
        {   // 滤波器初始化
            Eigen::Vector2d circle_center = calcCircleCenter(meas);
            uniform_ekf_.x_ << circle_center(0), circle_center(1), meas(2), uniform_ekf_.radius_, meas(3), 0;
            is_ekf_init_ = true;
            is_pred_success = false;
            post_state = {circle_center(0), circle_center(1), meas(2), uniform_ekf_.radius_, meas(3), 0.0};
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
            Eigen::Vector2d center2d = calcCircleCenter(meas);
            double radius = uniform_ekf_.x_(3);
            double rangle = uniform_ekf_.x_(4);
            double omega = uniform_ekf_.x_(5);
            
            double dis_diff = (pred_pos - cur_pos).norm();
            if (dis_diff >= 0.60)
            {
                is_singer_init_ = false;
                is_ekf_init_ = false;
                post_state = {meas(0), meas(1), meas(2), radius, rangle, omega};
                predictor_state_ = TRACKING;
                return false;
            }
            else if (dis_diff >= 0.30)
            {
                predictor_state_ = TRACKING;
                // uniform_ekf_.x_(0) = cur_pos(0) - radius * sin(meas(3));
                // uniform_ekf_.x_(1) = cur_pos(1) + radius * cos(meas(3));
                uniform_ekf_.x_(0) = center2d(0);
                uniform_ekf_.x_(1) = center2d(1);
                uniform_ekf_.x_(2) = (uniform_ekf_.x_(2) + last_state_(2)) / 2.0;
                uniform_ekf_.x_(4) = meas(3);
            }
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

            state = uniform_ekf_.x();
            Eigen::Vector3d circle_center3d = {state(0), state(1), state(2)};
            pred_state_vec_.clear();
            
            if (!is_outpost_mode_)
            {
                for (int ii = 0; ii < 4; ii++)
                {
                    Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                    double pred_radius = radius;
                    double pred_next_rangle = (rangle + CV_PI / 2 * ii);
                    double pred_x = circle_center3d(0) + pred_radius * sin(pred_next_rangle);
                    double pred_y = circle_center3d(1) - pred_radius * cos(pred_next_rangle);
                    double pred_z = circle_center3d(2);
                    if (history_switched_state_vec_.size() > 0)
                    {
                        pred_radius = (ii % 2 == 0) ? radius : history_switched_state_vec_.front()(3);
                        pred_x = circle_center3d(0) + pred_radius * sin(pred_next_rangle);
                        pred_y = circle_center3d(1) - pred_radius * cos(pred_next_rangle);
                        pred_z = (ii % 2 == 0) ? circle_center3d(2) : history_switched_state_vec_.front()(2);
                    }
                    armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                    pred_state_vec_.emplace_back(armor3d);
                }
            }
            else
            {
                for (int ii = 0; ii < 3; ii++)
                {
                    Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                    double pred_radius = radius;
                    double pred_next_rangle = (rangle + CV_PI * 2 / 3 * ii);
                    double pred_x = circle_center3d(0) + pred_radius * sin(pred_next_rangle);
                    double pred_y = circle_center3d(1) - pred_radius * cos(pred_next_rangle);
                    double pred_z = circle_center3d(2);
                    armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                    pred_state_vec_.emplace_back(armor3d);
                }
            }

            Eigen::MatrixXd F(6, 6);
            uniform_ekf_.updateF(F, pred_dt);
            Eigen::VectorXd pred = F * state;
            radius = state(3);
            rangle = pred(4);
            omega = pred(5);

            post_state = {state(0), state(1), state(2), radius, rangle, omega};
            // Eigen::Vector4d circle_center3d = {post_state(0), post_state(1), post_state(2), 0.0};
            // armor3d_vec.emplace_back(circle_center3d);

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
                state(0), state(1), state(2), radius, rangle, omega 
            );
            is_pred_success = true;
        }
        else
        {   // 更新
            uniform_ekf_.updateH(uniform_ekf_.H_, dt);
            uniform_ekf_.updateJh(uniform_ekf_.Jh_, dt);
            uniform_ekf_.Update(meas);

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

            if (is_outpost_mode_)
            {
                // cout << "is_outpost_mode" << endl;
                if (abs(omega - outpost_angular_speed_) >= 0.50)
                {
                    uniform_ekf_.x_(5) = outpost_angular_speed_;
                }
                else
                {
                    uniform_ekf_.x_(5) = (omega + outpost_angular_speed_) / 2.0;
                }
            }
            // cout << "omega1:" << uniform_ekf_.x_(5) << endl;
            state = uniform_ekf_.x();

            Eigen::Vector3d circle_center3d = {state(0), state(1), state(2)};
            pred_state_vec_.clear();
            if (!is_outpost_mode_)
            {
                for (int ii = 0; ii < 4; ii++)
                {
                    Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                    double pred_radius = radius;
                    double pred_next_rangle = (rangle + CV_PI / 2 * ii);
                    double pred_x = circle_center3d(0) + pred_radius * sin(pred_next_rangle);
                    double pred_y = circle_center3d(1) - pred_radius * cos(pred_next_rangle);
                    double pred_z = circle_center3d(2);
                    if (history_switched_state_vec_.size() > 0)
                    {
                        pred_radius = (ii % 2 == 0) ? radius : history_switched_state_vec_.front()(3);
                        pred_x = circle_center3d(0) + pred_radius * sin(pred_next_rangle);
                        pred_y = circle_center3d(1) - pred_radius * cos(pred_next_rangle);
                        pred_z = (ii % 2 == 0) ? circle_center3d(2) : history_switched_state_vec_.front()(2);
                    }
                    armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                    pred_state_vec_.emplace_back(armor3d);
                }
            }
            else
            {
                for (int ii = 0; ii < 3; ii++)
                {
                    Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
                    double pred_radius = radius;
                    double pred_next_rangle = (rangle + CV_PI * 2 / 3 * ii);
                    double pred_x = circle_center3d(0) + pred_radius * sin(pred_next_rangle);
                    double pred_y = circle_center3d(1) - pred_radius * cos(pred_next_rangle);
                    double pred_z = circle_center3d(2);
                    armor3d = {pred_x, pred_y, pred_z, pred_next_rangle};
                    pred_state_vec_.emplace_back(armor3d);
                }
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

            post_state = {state(0), state(1), state(2), radius, rangle, omega};
            // Eigen::Vector4d circle_center3d = {post_state(0), post_state(1), post_state(2), 0.0};
            // armor3d_vec.emplace_back(circle_center3d);
            // cout << "omega2:" << uniform_ekf_.x_(5) << endl;
            
            RCLCPP_WARN_THROTTLE(
                logger_, 
                steady_clock_, 
                100, 
                "circle_center:(%.3f, %.3f %.3f) radius:%.3f theta:%.3f omega:%.3f \n meas:(%.3f, %.3f %.3f %.3f)",
                state(0), state(1), state(2), radius, rangle, omega,
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
        if (!is_singer_init_)
        {
            singer_ekf_.x_ << meas(0), meas(1), meas(2), 0, 0, 0, 0, 0, 0;
            result = meas;
            is_singer_init_ = true;
            is_available = true;
            return is_available;
        }

        // 预测
        singer_ekf_.updateF(singer_ekf_.F_, dt);
        singer_ekf_.updateJf();
        singer_ekf_.updateQ(dt);
        singer_ekf_.Predict(dt);
        if (!is_target_lost)
        {
            // 如果预测量偏离观测量较大，则修正状态量
            Eigen::VectorXd statePre = singer_ekf_.x();
            // Eigen::MatrixXd stateCovPre = singer_ekf_.P();

            Eigen::Vector3d pred_center3d = {statePre(0), statePre(1), statePre(2)};
            double pos_diff = (pred_center3d - meas).norm();
            if (pos_diff >= 0.60)
            {
                result = meas;
                is_singer_init_ = false;
                is_available = false;
                return is_available;
            }
            else if (pos_diff >= 0.30)
            {
                singer_ekf_.x_(0) = meas(0);
                singer_ekf_.x_(1) = meas(1);
                singer_ekf_.x_(2) = meas(2);
            }
        }

        if (is_singer_init_ && !is_target_lost)
        {
            // 更新
            singer_ekf_.updateH(singer_ekf_.H_, dt);
            singer_ekf_.updateJh();
            singer_ekf_.Update(meas);

            // Eigen::MatrixXd predictState(9, 1);
            Eigen::VectorXd State = singer_ekf_.x();
            result = {State(0), State(1), State(2)};

            updateVel({State(3), State(4), State(5)});
            updateAcc({State(6), State(7), State(8)});

            // if (predict_acc_[0] == 0.0)
            //     singer_ekf_.setQ(target_acc);
            // else
            //     singer_ekf_.setQ(State[2]);

            //超前预测
            Eigen::MatrixXd F(9, 9);
            singer_ekf_.updateF(F, pred_dt);
            Eigen::MatrixXd Control(9, 3);
            singer_ekf_.updateC(Control, pred_dt);

            Eigen::Vector3d acc = {State(6), State(7), State(8)};
            VectorXd x_pred = F * State + Control * acc;
            result = {x_pred(0), x_pred(1), x_pred(2)};

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
        else if (predictor_state_ == LOSTING)
        {
            Eigen::VectorXd State = singer_ekf_.x();
            result = {State(0), State(1), State(2)};
           
            updateVel({State(3), State(4), State(5)});
            updateAcc({State(6), State(7), State(8)});
            
            // Eigen::MatrixXd F(9, 9);
            // singer_ekf_.updateF(F, pred_dt);
            // Eigen::MatrixXd Control(9, 3);
            // singer_ekf_.updateC(Control, pred_dt);

            // Eigen::Vector3d acc = {State(6), State(7), State(8)};
            // VectorXd x_pred = F * State + Control * acc;
            // result = {x_pred(0), x_pred(1), x_pred(2)};
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

    void ArmorPredictor::updateVel(Eigen::Vector3d vel_3d)
    {
        // X-AXIS
        predict_vel_[0][3] = predict_vel_[0][2];
        predict_vel_[0][2] = predict_vel_[0][1];
        predict_vel_[0][1] = predict_vel_[0][0];
        predict_vel_[0][0] = vel_3d[0];

        // Y-AXIS
        predict_vel_[1][3] = predict_vel_[1][2];
        predict_vel_[1][2] = predict_vel_[1][1];
        predict_vel_[1][1] = predict_vel_[1][0];
        predict_vel_[1][0] = vel_3d[1];

        // Z-AXIS
        predict_vel_[2][3] = predict_vel_[2][2];
        predict_vel_[2][2] = predict_vel_[2][1];
        predict_vel_[2][1] = predict_vel_[2][0];
        predict_vel_[2][0] = vel_3d[2];
        return;
    }

    void ArmorPredictor::updateAcc(Eigen::Vector3d acc_3d)
    {
        // X-AXIS
        predict_acc_[0][3] = predict_acc_[0][2];
        predict_acc_[0][2] = predict_acc_[0][1];
        predict_acc_[0][1] = predict_acc_[0][0];
        predict_acc_[0][0] = acc_3d[0] > 5.0 ? 0.0 : acc_3d[0];

        // Y-AXIS
        predict_acc_[1][3] = predict_acc_[1][2];
        predict_acc_[1][2] = predict_acc_[1][1];
        predict_acc_[1][1] = predict_acc_[1][0];
        predict_acc_[1][0] = acc_3d[1] > 5.0 ? 0.0 : acc_3d[1];

        // Z-AXIS
        predict_acc_[2][3] = predict_acc_[2][2];
        predict_acc_[2][2] = predict_acc_[2][1];
        predict_acc_[2][1] = predict_acc_[2][0];
        predict_acc_[2][0] = acc_3d[2] > 5.0 ? 0.0 : acc_3d[2];
        return;
    }

    // /**
    //  * @brief 计算RMSE指标
    //  *
    //  * @param params 参数首地址指针
    //  * @return RMSE值
    //  */
    // double ArmorPredictor::evalRMSE(double *params)
    // {
    //     double rmse_sum = 0;
    //     double rmse = 0;
    //     double pred = 0;
    //     double measure = 0;
    //     for (auto &target_info : history_info_)
    //     {
    //         auto t = (double)(target_info.timestamp) / 1e9;
    //         pred = params[0] * t + params[1]; // f(t)=kt+b
    //         measure = target_info.xyz[1];
    //     }
    //     rmse = sqrt(rmse_sum / history_info_.size());
    //     return rmse;
    // }

    // /**
    //  * @brief 计算滤波预测值与测量值的误差，判断滤波是否发散
    //  *
    //  * @return 返回预测值与测量值之间的误差
    //  */
    // double ArmorPredictor::calcError()
    // {
    //     bool flag = false;
    //     Eigen::Vector3d pred_error = {0.0, 0.0, 0.0};
    //     double error = 0.0;
    //     if ((int)history_pred_.size() > 0)
    //     {
    //         TargetInfo pre_info = history_pred_.front();
    //         for (int ii = 0; ii < (int)history_info_.size(); ii++)
    //         {
    //             if (ii != (int)(history_info_.size() - 1))
    //             {
    //                 if (pre_info.timestamp >= history_info_[ii].timestamp && pre_info.timestamp < history_info_[ii + 1].timestamp)
    //                 {
    //                     // RCLCPP_INFO(logger_, "pred_timestamp:%lfs meas_timestamp:%lfs", pre_info.timestamp / 1e9, history_info_[ii].timestamp / 1e9);
    //                     double dt = (history_info_[ii + 1].timestamp - history_info_[ii].timestamp) / 1e9;
    //                     double ddt = (pre_info.timestamp - history_info_[ii].timestamp) / 1e9;
    //                     auto weight = ddt / dt;
    //                     auto meas_pred = history_info_[ii].xyz * (1 - weight) + history_info_[ii].xyz * weight;
    //                     pred_error[0] = abs(meas_pred[0] - pre_info.xyz[0]);
    //                     pred_error[1] = abs(meas_pred[1] - pre_info.xyz[1]);
    //                     pred_error[2] = abs(meas_pred[2] - pre_info.xyz[2]);
    //                     history_pred_.pop_front();
    //                     flag = true;
    //                     break;
    //                 }
    //             }
    //             else if (pre_info.timestamp < history_info_[ii].timestamp)
    //             {
    //                 // RCLCPP_INFO(logger_, "pred_timestamp:%lfs meas_timestamp:%lfs", pre_info.timestamp / 1e9, history_info_[ii].timestamp / 1e9);
    //                 pred_error[0] = abs(history_info_[ii].xyz[0] - pre_info.xyz[0]);
    //                 pred_error[1] = abs(history_info_[ii].xyz[1] - pre_info.xyz[1]);
    //                 pred_error[2] = abs(history_info_[ii].xyz[2] - pre_info.xyz[2]);
    //                 history_pred_.pop_front();
    //                 flag = true;
    //                 break;
    //             }
    //         }

    //         if (flag)
    //         {
    //             error = sqrt(pow(pred_error[0], 2) + pow(pred_error[1], 2) + pow(pred_error[2], 2)) / 3.0;
    //             RCLCPP_INFO(logger_, "Prediction error:%lf", error);
    //         }
    //     }
    //     return std::move(error);
    // }



    // /**
    //  * @brief 前哨站旋转预测函数
    //  *
    //  * @param is_controlled 我方是否处于控制区，此时前哨站转速减半
    //  * @param target 目标信息
    //  * @param result 预测结果
    //  * @param time_estimated 时间延迟量
    //  * @return bool
    //  */
    // bool ArmorPredictor::spinningPredict(bool is_controlled, TargetInfo &target, Eigen::Vector3d &result, int64_t time_estimated)
    // {
    //     /**
    //      * @brief 前哨站旋转装甲运动预测（已知量：旋转半径&转速），考虑我方占领控制区旋转装甲板转速减半，应加入条件判断。
    //      *
    //      */
    //     // 轨迹拟合
    //     auto time_start = steady_clock_.now();
    //     double x0, y0, theta;

    //     ceres::Problem problem;
    //     ceres::Solver::Options options;
    //     ceres::Solver::Summary summary;

    //     options.max_num_iterations = 20;
    //     options.linear_solver_type = ceres::DENSE_QR;
    //     options.minimizer_progress_to_stdout = false;

    //     Eigen::Vector3d xyz_sum = {0, 0, 0};
    //     if (!is_controlled)
    //     {
    //         for (auto &target_info : history_info_)
    //         {
    //             xyz_sum += target_info.xyz;
    //             problem.AddResidualBlock(
    //                 new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
    //                     new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 1)),
    //                 new ceres::CauchyLoss(0.5),
    //                 &x0,
    //                 &y0,
    //                 &theta);
    //             problem.AddResidualBlock(
    //                 new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
    //                     new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 1)),
    //                 new ceres::CauchyLoss(0.5),
    //                 &x0,
    //                 &y0,
    //                 &theta);
    //         }
    //     }
    //     else
    //     {
    //         for (auto &target_info : history_info_)
    //         {
    //             xyz_sum += target_info.xyz;
    //             problem.AddResidualBlock(
    //                 new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
    //                     new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 0.5)),
    //                 new ceres::CauchyLoss(0.5),
    //                 &x0,
    //                 &y0,
    //                 &theta);
    //             problem.AddResidualBlock(
    //                 new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>(
    //                     new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e9, 0.5)),
    //                 new ceres::CauchyLoss(0.5),
    //                 &x0,
    //                 &y0,
    //                 &theta);
    //         }
    //     }
    //     auto xyz_ave = (xyz_sum / history_info_.size());

    //     problem.SetParameterUpperBound(&x0, 0, xyz_ave[0] + 0.6);
    //     problem.SetParameterLowerBound(&x0, 0, xyz_ave[0] - 0.6);
    //     problem.SetParameterUpperBound(&y0, 0, xyz_ave[1] + 0.6);
    //     problem.SetParameterLowerBound(&y0, 0, xyz_ave[1] - 0.6);
    //     problem.SetParameterUpperBound(&theta, 0, M_PI / 2);
    //     problem.SetParameterLowerBound(&theta, 0, -M_PI / 2);

    //     ceres::Solve(options, &problem, &summary);

    //     auto time_now = steady_clock_.now();
    //     auto dt_ns = (time_now - time_start).nanoseconds();
    //     RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "fitting_time:%.2fs", (dt_ns / 1e9));

    //     bool is_available[2];
    //     // auto rmse = evalRMSE()
    //     is_available[0] = (summary.final_cost <= predict_param_.max_cost);
    //     is_available[1] = (summary.final_cost <= predict_param_.max_cost);
    //     double x_pred, y_pred;
    //     if (!is_controlled)
    //     {
    //         x_pred = x0 + 0.2765 * ceres::cos(0.8 * M_PI * (time_estimated / 1e3) + theta);
    //         y_pred = y0 + 0.2765 * ceres::sin(0.8 * M_PI * (time_estimated / 1e3) + theta);
    //     }
    //     else
    //     {
    //         x_pred = x0 + 0.2765 * ceres::cos(0.4 * M_PI * (time_estimated / 1e3) + theta);
    //         y_pred = y0 + 0.2765 * ceres::sin(0.4 * M_PI * (time_estimated / 1e3) + theta);
    //     }

    //     result = {x_pred, y_pred, history_info_.end()->xyz[2]};
    //     return (is_available[0] && is_available[1]);
    // }
    
    // /**
    //  * @brief 滑窗滤波
    //  *
    //  * @param start_idx 滑窗起始位点
    //  * @return Eigen::Vector3d
    //  */
    // Eigen::Vector3d ArmorPredictor::shiftWindowFilter(int start_idx)
    // {
    //     // 计算最大迭代次数
    //     auto max_iter = int(history_info_.size() - start_idx) - predict_param_.window_size + 1;
    //     Eigen::Vector3d total_sum = {0, 0, 0};
    //     if (max_iter == 0 || start_idx < 0)
    //         return history_info_.back().xyz;

    //     for (int i = 0; i < max_iter; i++)
    //     {
    //         Eigen::Vector3d sum = {0, 0, 0};
    //         for (int j = 0; j < predict_param_.window_size; j++)
    //             sum += history_info_.at(start_idx + i + j).xyz;
    //         total_sum += sum / predict_param_.window_size;
    //     }
    //     return total_sum / max_iter;
    // }
} // namespace armor_processor