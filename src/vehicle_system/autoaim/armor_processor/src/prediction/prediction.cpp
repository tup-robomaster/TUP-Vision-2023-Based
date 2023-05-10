/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2023-05-06 23:15:24
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.hpp"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor()
    : logger_(rclcpp::get_logger("armor_prediction"))
    {
        // cout << "init_model" << endl;

        // Singer model initialized.
        singer_model_[0] = SingerModel(3, 1, 1);
        singer_model_[1] = SingerModel(3, 1, 1);
        singer_model_[2] = SingerModel(3, 1, 1);
        resetPredictor();

    }

    ArmorPredictor::~ArmorPredictor()
    {
    }

    void ArmorPredictor::initPredictor(const vector<double>* ekf_param)
    {
        uniform_ekf_.kf_param_.process_noise_params = ekf_param[0];
        uniform_ekf_.kf_param_.measure_noise_params = ekf_param[1];
        uniform_ekf_.init();

        singer_model_[0].singer_param_ = ekf_param[2];
        singer_model_[1].singer_param_ = ekf_param[3];
        singer_model_[2].singer_param_ = ekf_param[4];

        RCLCPP_INFO_ONCE(logger_, "uniform_process_noise_param:[%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]", ekf_param[0][0], ekf_param[0][1], ekf_param[0][2], ekf_param[0][3],
                        ekf_param[0][4], ekf_param[0][5], ekf_param[0][6], ekf_param[0][7]);
        RCLCPP_INFO_ONCE(logger_, "uniform_meas_noise_param:[%.2f %.2f %.2f %.2f]", ekf_param[1][0], ekf_param[1][1], ekf_param[1][2], ekf_param[1][3]);
        RCLCPP_INFO_ONCE(logger_, "singer_x_axis_param:[%.2f %.2f %.2f %.2f %.2f]", ekf_param[2][0], ekf_param[2][1], ekf_param[2][2], ekf_param[2][3], ekf_param[2][4]);
        RCLCPP_INFO_ONCE(logger_, "singer_y_axis_param:[%.2f %.2f %.2f %.2f %.2f]", ekf_param[3][0], ekf_param[3][1], ekf_param[3][2], ekf_param[3][3], ekf_param[3][4]);
        RCLCPP_INFO_ONCE(logger_, "singer_z_axis_param:[%.2f %.2f %.2f %.2f %.2f]", ekf_param[4][0], ekf_param[4][1], ekf_param[4][2], ekf_param[4][3], ekf_param[4][4]);
    }

    bool ArmorPredictor::resetPredictor()
    {
        is_singer_init_[0] = false;
        is_singer_init_[1] = false;
        is_singer_init_[2] = false;

        is_ekf_init_ = false;
        return true;
    }

    bool ArmorPredictor::updatePredictor(Eigen::VectorXd meas)
    {
        double pred_yaw = uniform_ekf_.x_(4);
        if (abs(pred_yaw - meas(3)) > 0.6)
        {
            uniform_ekf_.x_(2) = meas(2);
            uniform_ekf_.x_(4) = meas(3);
        }

        // Eigen::Vector2d circle3d = calcCircleCenter(meas);
        // Eigen::Vector2d pred_circle3d = {uniform_ekf_.x_(0), uniform_ekf_.x_(1)};
        // if ((pred_circle3d - circle3d).norm() > 0.15)
        // {
        //     uniform_ekf_.x_(0) = circle3d(0);
        //     uniform_ekf_.x_(1) = circle3d(1);
        // }
        return true;
    }

    bool ArmorPredictor::predict(TargetInfo target, double dt, double pred_dt, double &delay_time, Eigen::Vector3d &pred_point3d, vector<Eigen::Vector4d> &armor3d_vec, cv::Mat *src)
    {
        SpinHeading spin_state = target.is_spinning ? (target.is_clockwise ? CLOCKWISE : COUNTER_CLOCKWISE) : UNKNOWN;
        Eigen::Vector4d meas = {target.xyz(0), target.xyz(1), target.xyz(2), target.rangle};
        
        // if (is_ekf_init_)
        // {
        //     if (abs(target.rangle - last_rangle) > 1.0)
        //     {
        //         // cur_rangle_ += CV_PI / 2;
        //         cur_rangle_ += (last_rangle - target.rangle);
        //     }
        //     meas(3) += cur_rangle_;
        // }
        // else
        // {
        //     cur_rangle_ = 0.0;
        // }
        // last_rangle = target.rangle;

        // cout << "cur_rangle:" << cur_rangle_ << " last_rangle:" << last_rangle << " cur_rangle:" << target.rangle << endl;
        // last_rangle = target.rangle; 
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
            result = {meas(0), meas(1), meas(2)};
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
                uniform_ekf_.x_(2) = (uniform_ekf_.x_(2) + last_state_(2)) / 2.0;
                circle_center_sum(0) += cur_pos(0) - radius * sin(meas(3));
                circle_center_sum(1) += cur_pos(1) + radius * cos(meas(3));
                // ++count;
            }
            else if (dis_diff > 0.60)
            {
                is_singer_init_[0] = false;
                is_singer_init_[1] = false;
                is_singer_init_[2] = false;
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
            Eigen::VectorXd state = uniform_ekf_.x();
            double radius = state(3);
            double rangle = state(4);
            double omega = state(5);
            if (radius < 0.18)
            {
                radius = last_state_(3);
                uniform_ekf_.radius_ = last_state_(3);
                uniform_ekf_.x_(3) = last_state_(3);
            }
            else if (radius > 0.32)
            {
                radius = last_state_(3);
                uniform_ekf_.radius_ = last_state_(3);
                uniform_ekf_.x_(3) = last_state_(3);
            }
            
            if (abs(state(2) - last_state_(2)) > 0.10)
            {
                uniform_ekf_.x_(2) = (state(2) + last_state_(2)) / 2.0;
                state(2) = uniform_ekf_.x_(2);
            }

            Eigen::Vector3d circle_center = {state(0), state(1), state(2)};
            double pred_rangle = rangle;

            // if (spin_state == CLOCKWISE)
            // {
            //     pred_rangle = rangle - ((2 * CV_PI / spinning_period) * pred_dt);
            // }
            // else if (spin_state == COUNTER_CLOCKWISE)
            // {
            //     pred_rangle = rangle + (2 * CV_PI / spinning_period) * pred_dt;
            // }

            result = {circle_center(0), circle_center(1), state(2)};
            Eigen::Vector4d circle_center3d = {circle_center(0), circle_center(1), state(2), 0.0};
            armor3d_vec.emplace_back(circle_center3d);

            Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
            for (int ii = 0; ii < 4; ii++)
            {
                armor3d = {circle_center(0) + radius * sin(pred_rangle + CV_PI / 2 * ii), circle_center(1) - radius * cos(pred_rangle + CV_PI / 2 * ii), state(2), (pred_rangle + CV_PI / 2 * ii)};
                armor3d_vec.emplace_back(armor3d);
            }

            last_state_(0) = circle_center(0);
            last_state_(1) = circle_center(1);
            last_state_(2) = circle_center(2);
            last_state_(3) = radius;
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
            uniform_ekf_.updateH(uniform_ekf_.H_, dt);
            uniform_ekf_.updateJh(uniform_ekf_.Jh_, dt);
            uniform_ekf_.Update(meas);
            Eigen::VectorXd state = uniform_ekf_.x();
            double radius = state(3);
            double rangle = state(4);
            double omega = state(5);

            if (radius < 0.18)
            {
                radius = last_state_(3);
                uniform_ekf_.radius_ = last_state_(3);
                uniform_ekf_.x_(3) = last_state_(3);
            }
            else if (radius > 0.32)
            {
                radius = last_state_(3);
                uniform_ekf_.radius_ = last_state_(3);
                uniform_ekf_.x_(3) = last_state_(3);
            }

            if (abs(state(2) - last_state_(2)) > 0.10)
            {
                uniform_ekf_.x_(2) = (state(2) + last_state_(2)) / 2.0;
                state(2) = uniform_ekf_.x_(2);
            }

            last_state_(0) = state(0);
            last_state_(1) = state(1);
            last_state_(2) = state(2);
            last_state_(3) = radius;

            Eigen::MatrixXd F(6, 6);
            uniform_ekf_.updateF(F, pred_dt);
            Eigen::VectorXd pred = F * state;
            state = pred;
            
            // Eigen::MatrixXd Control(11, 3);
            // uniform_ekf_.setC(Control, pred_dt);
            // Eigen::MatrixXd acc(3, 1);
            // acc << uniform_ekf_.x_(8), uniform_ekf_.x_(9), uniform_ekf_.x_(10);
            // Eigen::VectorXd pred = F * state + Control * acc;
            
            radius = pred(3);
            rangle = pred(4);
            omega = pred(5);

            // Eigen::Vector3d circle_center = {pred(0), pred(1), pred(2)};
            Eigen::Vector3d circle_center = {state(0), state(1), state(2)};
            // Eigen::Vector3d meas_center = circle_center;
            // asyncPrediction(is_target_lost, meas_center, dt, pred_dt, circle_center);
            // uniform_ekf_.x_(0) = circle_center(0);
            // uniform_ekf_.x_(1) = circle_center(1);
            // uniform_ekf_.x_(2) = circle_center(2);
            double pred_rangle = rangle;

            // if (spin_state == CLOCKWISE)
            // {
            //     pred_rangle = rangle - (2 * CV_PI / spinning_period) * pred_dt;
            // }
            // else if (spin_state == COUNTER_CLOCKWISE)
            // {
            //     pred_rangle = rangle + (2 * CV_PI / spinning_period) * pred_dt;
            // }
            
            result = {circle_center(0), circle_center(1), state(2)};
            Eigen::Vector4d circle_center3d = {circle_center(0), circle_center(1), state(2), 0.0};
            armor3d_vec.emplace_back(circle_center3d);

            // Eigen::Vector2d circle3d = calcCircleCenter(meas);
            // Eigen::Vector4d circle4d = {circle3d(0), circle3d(1), meas(2), meas(3)};
            // armor3d_vec.emplace_back(circle4d);               
            // cout << "circle_x:" << circle4d(0) << " " << circle4d(1) << " " << circle4d(2) << endl;     

            // cout << "pred_rangle:" << rangle << " x:" << result(1) << endl;
            Eigen::Vector4d armor3d = {0.0, 0.0, 0.0, 0.0};
            for (int ii = 0; ii < 4; ii++)
            {
                armor3d = {circle_center(0) + radius * sin(pred_rangle + CV_PI / 2 * ii), circle_center(1) - radius * cos(pred_rangle + CV_PI / 2 * ii), state(2), (pred_rangle + CV_PI / 2 * ii)};
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

    bool ArmorPredictor::asyncPrediction(bool is_target_lost, Eigen::Vector3d meas, double dt, double pred_dt, Eigen::Vector3d& result)
    {
        PredictStatus is_singer_available;
        PredictStatus is_fitting_available;
        
        // 目标机动预测
        // 基于CS模型的卡尔曼滤波
        RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "CS model is predicting...");
        std::future<void> xyz_future[3];
        // is_ekf_available = predictBasedSinger(target, result_ekf, target_vel, target_acc, delta_time_estimate);
        if (debug_param_.x_axis_filter)
        {
            // is_singer_available.xyz_status[0] = predictBasedSinger(0, target.xyz[0], result_singer[0], target_vel[0], target_acc[0], delta_time_estimate);
            xyz_future[0] = std::async(std::launch::async, [&](){
                is_singer_available.xyz_status[0] = predictBasedSinger(is_target_lost, 0, meas[0], result[0], 0.0, 0.0, dt, pred_dt);});
        }
        if (debug_param_.y_axis_filter)
        {
            // is_singer_available.xyz_status[1] = predictBasedSinger(1, target.xyz[1], result_singer[1], target_vel[1], target_acc[1], delta_time_estimate);
            xyz_future[1] = std::async(std::launch::async, [&](){
                is_singer_available.xyz_status[1] = predictBasedSinger(is_target_lost, 1, meas[1], result[1], 0.0, 0.0, dt, pred_dt);});
        }
        if (debug_param_.z_axis_filter)
        {
            // is_singer_available.xyz_status[2] = predictBasedSinger(2, target.xyz[2], result_singer[2], target_vel[2], target_acc[2], delta_time_estimate);
            xyz_future[2] = std::async(std::launch::async, [&](){
                is_singer_available.xyz_status[2] = predictBasedSinger(is_target_lost, 2, meas[2], result[2], 0.0, 0.0, dt, pred_dt);});
        }
        if (debug_param_.x_axis_filter && xyz_future[0].wait_for(8ms) == std::future_status::timeout)
        {
            RCLCPP_WARN(logger_, "Filter _X_AXIS prediction timeout...");
        }
        if (debug_param_.y_axis_filter && xyz_future[1].wait_for(8ms) == std::future_status::timeout)
        {
            RCLCPP_WARN(logger_, "Filter Y_AXIS prediction timeout...");
        }
        if (debug_param_.z_axis_filter && xyz_future[2].wait_for(8ms) == std::future_status::timeout)
        {
            RCLCPP_WARN(logger_, "Filter Z_AXIS prediction timeout...");
        }
        return true;
    }


    /**
     * @brief 基于CS模型的卡尔曼滤波初始化
     *
     */
    void ArmorPredictor::kfInit()
    {
        // X-axis
        kfInit(0);
        // Y-axis
        kfInit(1);
        // Z-axis
        kfInit(2);
    }

    void ArmorPredictor::kfInit(int axis)
    {
        
    }

    bool ArmorPredictor::predictBasedSinger(bool is_target_lost, int axis, double meas, double& result, double target_vel, double target_acc, double dt, double pred_dt)
    {
        bool is_available;
        if (!is_target_lost)
        {
            if (!is_singer_init_[axis])
            {
                singer_model_[axis].x_ << meas, 0, 0;
                is_available = false;
                is_singer_init_[axis] = true;
            }
            else
            {
                Eigen::VectorXd measurement = Eigen::VectorXd(1);
                measurement << meas;

                singer_model_[axis].updateF(singer_model_[axis].F_, dt);
                singer_model_[axis].updateJf(singer_model_[axis].Jf_, dt);
                singer_model_[axis].Predict();

                // Eigen::MatrixXd stateCovPre = singer_model_[axis].P();
                // Eigen::MatrixXd statePre = singer_model_[axis].x();
                singer_model_[axis].updateH(singer_model_[axis].H_, dt);
                singer_model_[axis].updateJh(singer_model_[axis].Jh_, dt); 
                singer_model_[axis].Update(measurement);

                // Eigen::MatrixXd predictState(3, 1);
                Eigen::VectorXd State(3, 1);
                State << singer_model_[axis].x_(0), singer_model_[axis].x_(1), singer_model_[axis].x_(2);
                double post_pos = State[0];

                // predict_vel_[is_spinning][axis][3] = predict_vel_[is_spinning][axis][2];
                // predict_vel_[is_spinning][axis][2] = predict_vel_[is_spinning][axis][1];
                // predict_vel_[is_spinning][axis][1] = predict_vel_[is_spinning][axis][0];
                // predict_vel_[is_spinning][axis][0] = State[1];

                // predict_acc_[is_spinning][axis][3] = predict_acc_[is_spinning][axis][2];
                // predict_acc_[is_spinning][axis][2] = predict_acc_[is_spinning][axis][1];
                // predict_acc_[is_spinning][axis][1] = predict_acc_[is_spinning][axis][0];
                // predict_acc_[is_spinning][axis][0] = State[2];

                double alpha = singer_model_[axis].singer_param_[0];
                Eigen::MatrixXd F(3, 3);
                singer_model_[axis].setF(F, pred_dt, alpha);
                Eigen::MatrixXd control(3, 1);
                singer_model_[axis].setC(control, pred_dt, alpha);

                // if (history_acc_[axis][0] == 0.0)
                //     singer_model_[axis].setQ(target_acc);
                // else
                //     singer_model_[axis].setQ(State[2]);

                VectorXd pred = F * State + control * State[2];
                result = pred[0];

                // if (checkDivergence(statePre, stateCovPre, singer_model_[axis].H_, singer_model_[axis].R_, measurement) || abs(result - meas) > 0.85)
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
            // 对目标可能出现的位置进行预测
            singer_model_[axis].updateF(singer_model_[axis].F_, dt);
            singer_model_[axis].updateJf(singer_model_[axis].Jf_, dt);
            singer_model_[axis].Predict();

            Eigen::VectorXd State(3, 1);
            State << singer_model_[axis].x_[0], singer_model_[axis].x_[1], singer_model_[axis].x_[2];
            double post_pos = State[0];
            result = post_pos;

            // double alpha = singer_model_[axis].singer_param_[0];
            // Eigen::MatrixXd F(3, 3);
            // singer_model_[axis].setF(F, pred_dt, alpha);

            // Eigen::MatrixXd control(3, 1);
            // singer_model_[axis].setC(control, pred_dt, alpha);

            // VectorXd pred = F * State + control * State[2];
            // result = pred[0];
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
    PredictStatus ArmorPredictor::predictBasedImm(TargetInfo target, Eigen::Vector3d &result, Eigen::Vector3d target_vel, Eigen::Vector3d target_acc, int64_t timestamp)
    {
        PredictStatus is_available;
        double dt = timestamp / 1e9;
        if (!is_imm_init_)
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
     * @return PredictStatus
     */
    PredictStatus ArmorPredictor::spinningPredict(bool is_controlled, TargetInfo &target, Eigen::Vector3d &result, int64_t time_estimated)
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

        PredictStatus is_available;
        // auto rmse = evalRMSE()
        is_available.xyz_status[0] = (summary.final_cost <= predict_param_.max_cost);
        is_available.xyz_status[1] = (summary.final_cost <= predict_param_.max_cost);
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
} // namespace armor_processor