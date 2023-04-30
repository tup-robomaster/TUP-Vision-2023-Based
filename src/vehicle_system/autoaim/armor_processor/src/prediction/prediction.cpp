/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2023-04-30 03:21:23
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.hpp"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor()
    : logger_(rclcpp::get_logger("armor_prediction"))
    {
        resetPredictor();
    }

    ArmorPredictor::~ArmorPredictor()
    {
        resetPredictor();
    }

    void ArmorPredictor::initPredictor(const vector<double>* uniform_param)
    {
        uniform_ekf_.kf_param_.process_noise_params = uniform_param[0];
        uniform_ekf_.kf_param_.measure_noise_params = uniform_param[1];
        uniform_ekf_.kf_param_.singer_params = uniform_param[2];
        RCLCPP_INFO_ONCE(logger_, "uniform_process_noise_param:[%.2f %.2f]", uniform_param[0][0], uniform_param[0][1]);
        RCLCPP_INFO_ONCE(logger_, "uniform_meas_noise_param:[%.2f %.2f %.2f %.2f]", uniform_param[1][0], uniform_param[1][1], uniform_param[1][2], uniform_param[1][3]);
        RCLCPP_INFO_ONCE(logger_, "uniform_singer_param:[%.2f %.2f %.2f %.2f %.2f]", uniform_param[2][0], uniform_param[2][1], uniform_param[2][2], uniform_param[2][3], uniform_param[2][4]);
    }

    bool ArmorPredictor::resetPredictor()
    {
        is_ekf_init_ = false;
        return true;
    }

    bool ArmorPredictor::updatePredictor(Eigen::VectorXd meas)
    {
        uniform_ekf_.x_(4) = meas(3);
        // Eigen::Vector2d circle_center = calcCircleCenter(meas);
        // uniform_ekf_.x_(0) = circle_center(0);
        // uniform_ekf_.x_(1) = circle_center(1);
        uniform_ekf_.x_(2) = meas(2);
        return true;
    }

    bool ArmorPredictor::predict(TargetInfo target, double bullet_speed, double dt, double& delay_time, Eigen::Vector3d& pred_point3d, vector<Eigen::Vector4d>& armor3d_vec, cv::Mat* src)
    {
        double pred_dt = target.dist / bullet_speed + delay_time_;
        // RCLCPP_WARN(logger_, "rangle:%.3f", target.rangle);

        SpinHeading spin_state = target.is_spinning ? (target.is_clockwise ? CLOCKWISE : COUNTER_CLOCKWISE) : UNKNOWN;
        Eigen::Vector4d meas = {target.xyz(0), target.xyz(1), target.xyz(2), target.rangle};
        if (target.is_spinning)
            meas(3) += (spin_state == CLOCKWISE ? (-CV_PI / 2) : (CV_PI / 2));
        
        if (!predictBasedUniformModel(target.is_target_lost, spin_state, meas, dt, pred_dt, target.period, pred_point3d, armor3d_vec))
        {
            // cout << 6 << endl;
            pred_point3d = target.xyz;
            return false;
        }
        return true;
    }

    bool ArmorPredictor::predictBasedUniformModel(bool is_target_lost, SpinHeading spin_state, Eigen::VectorXd meas, double dt, double pred_dt, double spinning_period, Eigen::Vector3d& result, vector<Eigen::Vector4d>& armor3d_vec)
    {
        bool is_pred_success = false;
        if (!is_ekf_init_)
        {
            Eigen::Vector2d circle_center = calcCircleCenter(meas);
            uniform_ekf_.x_ << circle_center(0), circle_center(1), meas(2), uniform_ekf_.radius_, meas(3), 0, 0, 0, 0, 0, 0;
            is_ekf_init_ = true;
            is_pred_success = false;
            result = {meas(0), meas(1), meas(2)};
        }
        else if (is_target_lost && predictor_state_ == LOSTING)
        {   //预测
            uniform_ekf_.Predict(dt);
            // uniform_ekf_.x_(3) = 0.15;
            Eigen::VectorXd state = uniform_ekf_.x();
            Eigen::Vector3d circle_center = {state(0), state(1), state(2)};
            double radius = state(3) = 0.15;
            double rangle = state(4);
            
            double pred_rangle = rangle;
            if (spin_state == CLOCKWISE)
            {
                pred_rangle = rangle - (2 * CV_PI / spinning_period) * pred_dt;
            }
            else if (spin_state == COUNTER_CLOCKWISE)
            {
                pred_rangle = rangle + (2 * CV_PI / spinning_period) * pred_dt;
            }
            result = {circle_center(0) + radius * sin(pred_rangle), circle_center(1) + radius * cos(pred_rangle), circle_center(2)};
            
            Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
            for (int ii = 0; ii < 4; ii++)
            {
                armor3d = {circle_center(0) + radius * sin(pred_rangle + CV_PI / 2 * ii), circle_center(1) + radius * cos(pred_rangle + CV_PI / 2 * ii), circle_center(2), (pred_rangle + CV_PI / 2 * ii)};
                armor3d_vec.emplace_back(armor3d);
            }
            
            is_pred_success = true;
        }
        else
        {   //预测+更新
            uniform_ekf_.Predict(dt);
            uniform_ekf_.Update(meas, meas(3));
            // uniform_ekf_.x_(3) = 0.15;
            Eigen::VectorXd state = uniform_ekf_.x();
            Eigen::MatrixXd F(11, 11);
            uniform_ekf_.setF(F, pred_dt);
            Eigen::VectorXd pred = F * state;
            Eigen::Vector3d circle_center = {pred(0), pred(1), pred(2)};
            double radius = pred(3) = 0.15;
            double rangle = pred(4);

            double pred_rangle = rangle;
            if (spin_state == CLOCKWISE)
            {
                pred_rangle = rangle - (2 * CV_PI / spinning_period) * pred_dt;
            }
            else if (spin_state == COUNTER_CLOCKWISE)
            {
                pred_rangle = rangle + (2 * CV_PI / spinning_period) * pred_dt;
            }  
            result = {circle_center(0) + radius * sin(pred_rangle), circle_center(1) + radius * cos(pred_rangle), circle_center(2)};
            
            Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
            for (int ii = 0; ii < 4; ii++)
            {
                armor3d = {circle_center(0) + radius * sin(pred_rangle + CV_PI / 2 * ii), circle_center(1) + radius * cos(pred_rangle + CV_PI / 2 * ii), circle_center(2), (pred_rangle + CV_PI / 2 * ii)};
                armor3d_vec.emplace_back(armor3d);
            }

            is_pred_success = true;
        }
        return is_pred_success;
    }

    Eigen::Vector2d ArmorPredictor::calcCircleCenter(Eigen::VectorXd meas)
    {
        return Eigen::Vector2d{meas(0) + uniform_ekf_.radius_ * sin(meas(3)), meas(1) + uniform_ekf_.radius_ * cos(meas(3))};
    }

    // /**
    //  * @brief 对目标位置进行预测
    //  * 
    //  * @param target_msg 目标message
    //  * @param timestamp 本帧对应的时间戳
    //  * @param delay_time 休眠时间，对应于预测延迟量 
    //  * @param src 图像数据
    //  * @return Eigen::Vector3d 
    //  */
    // Eigen::Vector3d ArmorPredictor::predict(TargetInfo target, uint64_t timestamp, double& delay_time, cv::Mat* src)
    // {
    //     auto t1 = steady_clock_.now();

    //     Eigen::Vector3d result = {0.0, 0.0, 0.0};
    //     result = target.xyz;

    //     // auto d_xyz = target.xyz - final_target_.xyz;
    //     int64_t delta_time_estimate = 0;
    //     if ((int)history_info_.size() > 0)
    //     {
    //         // auto delta_t = timestamp - final_target_.timestamp;
    //         // int64_t time_estimate = delta_time_estimate + history_info_.back().timestamp;

    //         double last_dist = history_info_.back().dist;
    //         delta_time_estimate = (int64_t)((last_dist / predict_param_.bullet_speed) * 1e9) + (int64_t)(predict_param_.shoot_delay * 1e6);
    //         delay_time = delta_time_estimate;
    //         final_target_ = target;
    //     }
    //     else
    //     {
    //         if((int)history_info_.size() > 100)
    //         {
    //             history_info_.pop_front();
    //             history_info_.push_back(target);
    //         }
    //         else
    //         {
    //             history_info_.push_back(target);
    //         }
    //         return result;
    //     }
        
    //     bool is_target_lost = target.is_target_lost;
    //     bool is_spinning = target.is_spinning;
    //     if (predictor_state_ == PREDICTING)
    //     {   //预测器处于预测击打阶段
    //         asyncPrediction(!filter_disabled_, is_target_lost, is_spinning, target.xyz, delta_time_estimate, result);
    //         TargetInfo target_pred;
    //         target_pred.xyz = result;
    //         target_pred.timestamp = timestamp;

    //         if ((int)history_pred_.size() > 100)
    //         {
    //             history_pred_.pop_front();
    //             history_pred_.push_back(target_pred);
    //         }
    //         else
    //         {
    //             history_pred_.push_back(target_pred);
    //         }
    //         // RCLCPP_INFO(logger_, "111");
    //     }
    //     else if (predictor_state_ == LOSTING)
    //     {
    //         asyncPrediction(!filter_disabled_, is_target_lost, is_spinning, target.xyz, delta_time_estimate, result);
    //         TargetInfo target_losting_pred;
    //         target_losting_pred.xyz = result;
    //         target_losting_pred.timestamp = timestamp;
    //         if ((int)history_pred_.size() > 100)
    //         {
    //             history_losting_pred_.pop_front();
    //             history_losting_pred_.push_back(target_losting_pred);
    //         }
    //         else
    //         {
    //             history_losting_pred_.push_back(target_losting_pred);
    //         }
    //         // RCLCPP_INFO(logger_, "222");
    //     }
    //     if (predictor_state_ == TRACKING)
    //     {   //预测器当前处于跟踪阶段
    //         final_target_ = target;
    //         result = target.xyz;
    //         // RCLCPP_INFO(logger_, "333");
    //         return result;
    //     }

    //     // if (fitting_disabled_ && filter_disabled_)
    //     // {   //滤波和拟合均失效，使用当前目标位置信息
    //     //     result = Vector3d{target.xyz[0], target.xyz[1], target.xyz[2]};
    //     // }

    //     auto t2 = steady_clock_.now();
    //     double dr_ns = (t2 - t1).nanoseconds();
    //     if (debug_param_.print_delay)
    //     {
    //         RCLCPP_INFO(logger_, "target: x:%lf y:%lf z:%lf", target.xyz[0], target.xyz[1], target.xyz[2]);
    //         RCLCPP_INFO(logger_, "Predict time:%lfms", (dr_ns / 1e6));
    //         RCLCPP_INFO(logger_, "predict: x:%lf y:%lf z:%lf", result[0], result[1], result[2]);
    //     }
    //     final_target_.xyz = result;

    //     return result;
    // }

    // bool ArmorPredictor::asyncPrediction(bool is_filtering, bool is_target_lost, bool is_spinning, Eigen::Vector3d meas, int64_t dt, Eigen::Vector3d& result)
    // {
    //     PredictStatus is_singer_available;
    //     PredictStatus is_fitting_available;
    //     if (is_filtering)
    //     {   // 目标机动预测
    //         // 基于CS模型的卡尔曼滤波
    //         RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "CS model is predicting...");
    //         std::future<void> xyz_future[3];
    //         // is_ekf_available = predictBasedSinger(target, result_ekf, target_vel, target_acc, delta_time_estimate);
    //         if (debug_param_.x_axis_filter)
    //         {
    //             // is_singer_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);
    //             xyz_future[0] = std::async(std::launch::async, [&](){
    //                 is_singer_available.xyz_status[0] = predictBasedSinger(is_target_lost, is_spinning, 0, meas[0], result[0], 0.0, 0.0, dt);});
    //         }
    //         if (debug_param_.y_axis_filter)
    //         {
    //             // is_singer_available.xyz_status[1] = predictBasedSinger(1, target.xyz[1], result_singer[1], target_vel[1], target_acc[1], delta_time_estimate);
    //             xyz_future[1] = std::async(std::launch::async, [&](){
    //                 is_singer_available.xyz_status[1] = predictBasedSinger(is_target_lost, is_spinning, 1, meas[1], result[1], 0.0, 0.0, dt);});
    //         }
    //         if (debug_param_.z_axis_filter)
    //         {
    //             // is_singer_available.xyz_status[2] = predictBasedSinger(2, target.xyz[2], result_singer[2], target_vel[2], target_acc[2], delta_time_estimate);
    //             xyz_future[2] = std::async(std::launch::async, [&](){
    //                 is_singer_available.xyz_status[2] = predictBasedSinger(is_target_lost, is_spinning, 2, meas[2], result[2], 0.0, 0.0, dt);});
    //         }
    //         if (debug_param_.x_axis_filter && xyz_future[0].wait_for(8ms) == std::future_status::timeout)
    //         {
    //             RCLCPP_WARN(logger_, "Filter _X_AXIS prediction timeout...");
    //         }
    //         if (debug_param_.y_axis_filter && xyz_future[1].wait_for(8ms) == std::future_status::timeout)
    //         {
    //             RCLCPP_WARN(logger_, "Filter Y_AXIS prediction timeout...");
    //         }
    //         if (debug_param_.z_axis_filter && xyz_future[2].wait_for(8ms) == std::future_status::timeout)
    //         {
    //             RCLCPP_WARN(logger_, "Filter Z_AXIS prediction timeout...");
    //         }
    //     }
    //     else
    //     {   
    //         // 反陀螺模式
    //         if (is_spinning)
    //         {
    //             // is_fitting_available = coupleFittingPredict(true, target, result_fitting, time_estimate);  
    //             std::future<void> xyz_future[3];
    //             if (debug_param_.x_axis_filter)
    //             {
    //                 // is_singer_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);
    //                 xyz_future[0] = std::async(std::launch::async, [&](){
    //                     is_fitting_available.xyz_status[0] = predictBasedSinger(is_target_lost, is_spinning, 0, meas[0], result[0], 0.0, 0.0, dt);}
    //                 );
    //             }
    //             if (debug_param_.y_axis_filter)
    //             {
    //                 xyz_future[1] = std::async(std::launch::async, [&](){
    //                         // is_fitting_available = coupleFittingPredict(true, target, result_fitting, time_estimate);
    //                         is_fitting_available.xyz_status[1] = predictBasedSinger(is_target_lost, is_spinning, 1, meas[1], result[1], 0.0, 0.0, dt);
    //                     }
    //                 );
    //             }
    //             if (debug_param_.z_axis_filter)
    //             {
    //                 xyz_future[2] = std::async(std::launch::async, [&](){
    //                     is_fitting_available.xyz_status[2] = predictBasedSinger(is_target_lost, is_spinning, 2, meas[2], result[2], 0.0, 0.0, dt);}
    //                 );
    //             }

    //             if (debug_param_.x_axis_filter && xyz_future[0].wait_for(8ms) == std::future_status::timeout)
    //             {
    //                 RCLCPP_WARN(logger_, "Fitting X_AXIS prediction timeout...");
    //             }
    //             if (debug_param_.y_axis_filter && xyz_future[1].wait_for(8ms) == std::future_status::timeout)
    //             {
    //                 RCLCPP_WARN(logger_, "Fitting Y_AXIS prediction timeout...");
    //             }
    //             if (debug_param_.z_axis_filter && xyz_future[2].wait_for(8ms) == std::future_status::timeout)
    //             {
    //                 RCLCPP_WARN(logger_, "Fitting Z_AXIS prediction timeout...");
    //             }
    //         }
    //     }
    //     return true;
    // }

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
        singer_kf_[0][axis].F_ = singer_model_[0][axis].F();
        singer_kf_[0][axis].H_ = singer_model_[0][axis].H();
        singer_kf_[0][axis].C_ = singer_model_[0][axis].C();
        singer_kf_[0][axis].P_ = singer_model_[0][axis].P();
        singer_kf_[0][axis].Q_ = singer_model_[0][axis].Q();
        singer_kf_[0][axis].R_ = singer_model_[0][axis].R();

        singer_kf_[1][axis].F_ = singer_model_[1][axis].F();
        singer_kf_[1][axis].H_ = singer_model_[1][axis].H();
        singer_kf_[1][axis].C_ = singer_model_[1][axis].C();
        singer_kf_[1][axis].P_ = singer_model_[1][axis].P();
        singer_kf_[1][axis].Q_ = singer_model_[1][axis].Q();
        singer_kf_[1][axis].R_ = singer_model_[1][axis].R();
    }
    
    bool ArmorPredictor::predictBasedSinger(bool is_target_lost, bool is_spinning, int axis, double meas, double& result, double target_vel, double target_acc, int64_t timestamp)
    {
        bool is_available;
        if (!is_target_lost)
        {
            if (!is_singer_init_[is_spinning][axis])
            {
                singer_kf_[is_spinning][axis].x_ << meas, 0, 0;
                is_available = false;
                // predictor_state_ = TRACKING;
                is_singer_init_[is_spinning][axis] = true;
            }
            else
            {
                Eigen::VectorXd measurement = Eigen::VectorXd(1);
                measurement << meas;
                
                singer_kf_[is_spinning][axis].Predict();
                // Eigen::MatrixXd stateCovPre = singer_kf_[is_spinning][axis].P();
                // Eigen::MatrixXd statePre = singer_kf_[is_spinning][axis].x();
                singer_kf_[is_spinning][axis].Update(measurement);

                // Eigen::MatrixXd predictState(3, 1);
                Eigen::VectorXd State(3, 1);
                State << singer_kf_[is_spinning][axis].x_[0], singer_kf_[is_spinning][axis].x_[1], singer_kf_[is_spinning][axis].x_[2];

                double post_pos = State[0];

                // predict_vel_[is_spinning][axis][3] = predict_vel_[is_spinning][axis][2];
                // predict_vel_[is_spinning][axis][2] = predict_vel_[is_spinning][axis][1];
                // predict_vel_[is_spinning][axis][1] = predict_vel_[is_spinning][axis][0];
                // predict_vel_[is_spinning][axis][0] = State[1];
                
                // predict_acc_[is_spinning][axis][3] = predict_acc_[is_spinning][axis][2];
                // predict_acc_[is_spinning][axis][2] = predict_acc_[is_spinning][axis][1];
                // predict_acc_[is_spinning][axis][1] = predict_acc_[is_spinning][axis][0];
                // predict_acc_[is_spinning][axis][0] = State[2];

                double alpha = singer_param_[is_spinning][axis][0];
                double dt = singer_param_[is_spinning][axis][8] * singer_param_[is_spinning][axis][4];
                if (is_spinning)
                    dt = timestamp / 1e9;
                else
                    dt = timestamp / 1e9 * 60;
                
                Eigen::MatrixXd F(3, 3);
                singer_model_[is_spinning][axis].setF(F, dt, alpha);

                Eigen::MatrixXd control(3, 1);
                singer_model_[is_spinning][axis].setC(control, dt, alpha);

                // if (history_acc_[axis][0] == 0.0)
                //     singer_model_[axis].setQ(target_acc);
                // else
                //     singer_model_[axis].setQ(State[2]);

                VectorXd pred = F * State + control * State[2];
                result = pred[0];

                // if (checkDivergence(statePre, stateCovPre, singer_kf_[is_spinning][axis].H_, singer_kf_[is_spinning][axis].R_, measurement) || abs(result - meas) > 0.85)
                // {
                //     RCLCPP_WARN(logger_, "Filter is diverging...");
                //     // singer_kf_->P_ = singer_model_[axis].P();
                //     // is_singer_init_[axis] = false;
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
                //     is_singer_init_[axis] = false;
                //     result = meas;
                // }
                is_available = true;
            }
            return is_available;
        }
        else if (predictor_state_ == LOSTING)
        {
            //对目标可能出现的位置进行预测
            singer_kf_[is_spinning][axis].Predict();

            Eigen::VectorXd State(3, 1);
            State << singer_kf_[is_spinning][axis].x_[0], singer_kf_[is_spinning][axis].x_[1], singer_kf_[is_spinning][axis].x_[2];
            double post_pos = State[0];
            double alpha = singer_param_[is_spinning][axis][0];
            double dt = singer_param_[is_spinning][axis][8] * singer_param_[is_spinning][axis][4];
            if (is_spinning)
                dt = timestamp / 1e9;
            else
                dt = timestamp / 1e9 * 60;
            
            Eigen::MatrixXd F(3, 3);
            singer_model_[is_spinning][axis].setF(F, dt, alpha);

            Eigen::MatrixXd control(3, 1);
            singer_model_[is_spinning][axis].setC(control, dt, alpha);

            VectorXd pred = F * State + control * State[2];
            result = pred[0];
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

    // /**
    //  * @brief 粒子滤波预测函数
    //  * 
    //  * @param target 目标信息
    //  * @param result 预测信息
    //  * @param time_estimated 延迟时间量
    //  * @return PredictStatus 各个轴预测成功与否
    //  */
    // PredictStatus ArmorPredictor::predictBasePF(TargetInfo target, Vector3d& result, int64_t time_estimated)
    // {
    //     PredictStatus is_available;
    //     //采取中心差分法,使用 t, t-1, t-2时刻速度,计算t-1时刻的速度
    //     auto target_prev = history_info_.at(history_info_.size() - 3);
    //     auto target_next = target;
    //     auto v_xyz = (target_next.xyz - target_prev.xyz) / (target_next.timestamp - target_prev.timestamp) * 1e9;
    //     auto t = target_next.timestamp - history_info_.at(history_info_.size() - 2).timestamp;

    //     is_available.xyz_status[0] = pf_v.is_ready;
    //     is_available.xyz_status[1] = pf_v.is_ready;
    //     // cout<<v_xyz<<endl;

    //     //Update
    //     Eigen::VectorXd measure (2);
    //     measure << v_xyz[0], v_xyz[1];
    //     pf_v.update(measure);

    //     //Predict
    //     auto result_v = pf_v.predict();

    //     std::cout << measure << std::endl;

    //     // cout<<result_v<<endl;
    //     //TODO:恢复速度预测
    //     // auto predict_x = target.xyz[0];
    //     // auto predict_y = target.xyz[1];
    //     double predict_x;
    //     double predict_y;

    //     if (history_info_.size() > 6)
    //     {
    //         predict_x = target.xyz[0] + result_v[0] * (time_estimated + t) / 1e9;
    //         predict_y = target.xyz[1] + result_v[1] * (time_estimated + t) / 1e9;
    //     }
    //     else
    //     {
    //         predict_x = target.xyz[0];
    //         predict_y = target.xyz[1];       
    //     }

    //     result << predict_x, predict_y, target.xyz[2];
    //     return is_available;
    // }

    void ArmorPredictor::updateVel(bool is_spinning, Eigen::Vector3d vel_3d)
    {
        // X-AXIS
        history_vel_[is_spinning][0][3] = history_vel_[is_spinning][0][2];
        history_vel_[is_spinning][0][2] = history_vel_[is_spinning][0][1];
        history_vel_[is_spinning][0][1] = history_vel_[is_spinning][0][0];
        history_vel_[is_spinning][0][0] = vel_3d[0];

        // Y-AXIS
        history_vel_[is_spinning][1][3] = history_vel_[is_spinning][1][2];
        history_vel_[is_spinning][1][2] = history_vel_[is_spinning][1][1];
        history_vel_[is_spinning][1][1] = history_vel_[is_spinning][1][0];
        history_vel_[is_spinning][1][0] = vel_3d[1];

        // Z-AXIS
        history_vel_[is_spinning][2][3] = history_vel_[is_spinning][2][2];
        history_vel_[is_spinning][2][2] = history_vel_[is_spinning][2][1];
        history_vel_[is_spinning][2][1] = history_vel_[is_spinning][2][0];
        history_vel_[is_spinning][2][0] = vel_3d[2];
        return;
    }
    
    void ArmorPredictor::updateAcc(bool is_spinning, Eigen::Vector3d acc_3d)
    {
        // X-AXIS
        history_acc_[is_spinning][0][3] = history_acc_[is_spinning][0][2];
        history_acc_[is_spinning][0][2] = history_acc_[is_spinning][0][1];
        history_acc_[is_spinning][0][1] = history_acc_[is_spinning][0][0];
        history_acc_[is_spinning][0][0] = acc_3d[0] > 5.0 ? 0.0 : acc_3d[0];

        // Y-AXIS
        history_acc_[is_spinning][1][3] = history_acc_[is_spinning][1][2];
        history_acc_[is_spinning][1][2] = history_acc_[is_spinning][1][1];
        history_acc_[is_spinning][1][1] = history_acc_[is_spinning][1][0];
        history_acc_[is_spinning][1][0] = acc_3d[1] > 5.0 ? 0.0 : acc_3d[1];

        // Z-AXIS
        history_acc_[is_spinning][2][3] = history_acc_[is_spinning][2][2];
        history_acc_[is_spinning][2][2] = history_acc_[is_spinning][2][1];
        history_acc_[is_spinning][2][1] = history_acc_[is_spinning][2][0];
        history_acc_[is_spinning][2][0] = acc_3d[2] > 5.0 ? 0.0 : acc_3d[2];
        return;
    }
} //namespace armor_processor