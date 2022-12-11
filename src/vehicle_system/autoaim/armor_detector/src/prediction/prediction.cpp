/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2022-12-08 21:44:15
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.h"

namespace armor_detector
{
    ArmorPredictor::ArmorPredictor()
    {
        int cnt = 0;
        pic_x = cv::Mat::zeros(500, 2000, CV_8UC3);
        pic_y = cv::Mat::zeros(500, 2000, CV_8UC3);
        pic_z = cv::Mat::zeros(500, 2000, CV_8UC3);

        //kf initializing
        kalman_filter_.Init(3, 1, 1);

        // SingerModel singer;
        kfInit();

        filter_disabled_ = false;
        fitting_disabled_ = false;
        is_init = false;
        is_ekf_init = false;
        is_predicted = false;
    }

    ArmorPredictor::~ArmorPredictor(){}

    // ArmorPredictor::ArmorPredictor(const PredictParam& predict_param, const SingerModelParam& singer_model_param, const DebugParam& debug_param, const std::string filter_param_path)
    // : predict_param_(predict_param), singer_model_param_(singer_model_param), debug_param_(debug_param), filter_param_path_(filter_param_path)
    // {
    //     int cnt = 0;
    //     pic_x = cv::Mat::zeros(500, 2000, CV_8UC3);
    //     pic_y = cv::Mat::zeros(500, 2000, CV_8UC3);
    //     pic_z = cv::Mat::zeros(500, 2000, CV_8UC3);

    //     // config_ = YAML::LoadFile(coord_file);
    //     // pf_pos.initParam(config_, "pos");
    //     // pf_v.initParam(config_, "v");

    //     //kf initializing
    //     kalman_filter_.Init(3, 1, 1);

    //     SingerModel singer;
    //     kfInit(singer);
        
    //     fitting_disabled_ = false;
    //     is_init = false;
    //     is_ekf_init = false;
    // }

    ArmorPredictor::ArmorPredictor(const PredictParam& predict_param, const SingerModel& singer_model_param, const DebugParam& debug_param, const std::string filter_param_path)
    : predict_param_(predict_param), singer_param_(singer_model_param), debug_param_(debug_param), filter_param_path_(filter_param_path)
    {
            int cnt = 0;
            pic_x = cv::Mat::zeros(500, 2000, CV_8UC3);
            pic_y = cv::Mat::zeros(500, 2000, CV_8UC3);
            pic_z = cv::Mat::zeros(500, 2000, CV_8UC3);

            // config_ = YAML::LoadFile(coord_file);
            // pf_pos.initParam(config_, "pos");
            // pf_v.initParam(config_, "v");

            //kf initializing
            kalman_filter_.Init(3, 1, 1);

            // SingerModel singer;
            kfInit();
            
            filter_disabled_ = debug_param_.disable_filter;
            fitting_disabled_ = false;
            is_init = false;
            is_ekf_init = false;
    }

    void ArmorPredictor::init(bool target_switched)
    {
        if(target_switched)
        {
            if(history_origin_info_.size() != 0)
                history_origin_info_.clear();
            // history_delta_x_pred_.clear();
            is_predicted = false;
            return;
        }
    }

    Eigen::Vector3d ArmorPredictor::predict(cv::Mat& src, TargetInfoPtr target_ptr, int timestamp, int& sleep_time)
    {
        // std::cout << 1 << std::endl;

        if(!is_init)
        {
            config_ = YAML::LoadFile(filter_param_path_);
            pf_pos.initParam(config_, "pos");
            pf_v.initParam(config_, "v");
            is_init = true;
        }

        auto t1 = std::chrono::steady_clock::now();
        TargetInfo target = 
        {
            target_ptr->xyz, (int)target_ptr->xyz.norm(), timestamp,
            target_ptr->period, target_ptr->is_target_switched, target_ptr->is_spinning, target_ptr->is_sentry_mode,
            target_ptr->is_clockwise, target_ptr->spinning_status, target_ptr->sentry_armor_status, target_ptr->system_model
        };

        // std::cout << "x_:" << target_ptr->xyz[0] << std::endl;

        // -----------------对位置进行粒子滤波,以降低测距噪声影响-------------------------------------
        // Eigen::VectorXd measure (2);

        // // Y-axis matches to camera frame.
        // measure << target.xyz[0], target.xyz[2];
        
        // bool is_pos_filter_ready = pf_pos.update(measure);
        // Eigen::VectorXd predict_pos_xy = pf_pos.predict();
        // Eigen::Vector3d predict_pos = {predict_pos_xy[0], predict_pos_xy[1], target.xyz[2]};

        Eigen::Vector3d result = {0, 0, 0};
        Eigen::Vector3d result_pf = {0, 0, 0};
        Eigen::Vector3d result_fitting = {0, 0, 0};
        Eigen::Vector3d result_ekf = {0, 0, 0};
        Eigen::Vector3d result_imm = {0, 0, 0};
        PredictStatus is_pf_available;
        PredictStatus is_fitting_available;
        PredictStatus is_ekf_available;
        PredictStatus is_imm_available;

        // if(is_pos_filter_ready || abs(target.xyz[2] - predict_pos[1]) < 0.20)
        // {   //对位置进行粒子滤波,以降低测距噪声影响
        //     target.xyz[2] = predict_pos[1];
        // }
        if(!fitting_disabled_)
        {
            if(!target.is_target_switched)
            {
                history_info_.push_back(target);
            }
            else
            {
                if(history_info_.size() != 0)
                {
                    history_origin_info_.push_back(cv::Point2d(history_info_.front().xyz[0], history_info_.front().xyz[2]));
                    history_info_.clear();
                }
                history_info_.push_back(target);
            }
        }
        // std::cout << std::endl;
        // std::cout << "target_switched: " << target.is_target_switched << std::endl;
        // std::cout << std::endl;
        // // //若位置粒子滤波器未完成初始化或滤波结果与目前位置相距过远,则本次不对目标位置做滤波,直接向队列压入原值
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
        
        //-----------------进行滑窗滤波,备选方案,暂未使用-------------------------------------
        auto d_xyz = target.xyz - final_target_.xyz;
        auto delta_t = timestamp - final_target_.timestamp;
        auto last_dist = history_info_.back().dist;
        auto delta_time_estimate = (last_dist / predict_param_.bullet_speed) * 1e3 + predict_param_.shoot_delay;
        auto time_estimate = delta_time_estimate + history_info_.back().timestamp;

        sleep_time = delta_time_estimate;
        
        if (history_info_.size() < 4)
        {
            final_target_ = target;

            if(!fitting_disabled_)
            {
                if(is_predicted)
                {
                    if(history_info_.size() != 0)
                    {
                        double last_to_now_timestamp = history_info_.back().timestamp - last_start_timestamp_;
                        double tt = time_estimate - last_start_timestamp_;
                        
                        double last_pred_x = fitting_params_[0] * (last_to_now_timestamp / 1e3) + fitting_params_[1];
                    
                        double delta_y = last_pred_x - target.xyz[0]; 
                        // double tt = time_estimate / 1e3;
                        // result[0] = fitting_params_[0] * (delta_time_estimate * history_info_.size() / 1e3) + history_info_.front().xyz[0]; // x(t)=kt+x0
                        result[0] = fitting_params_[0] * (tt / 1e3) + fitting_params_[1] - delta_y; // x(t)=kt+d
                        result[1] = target.xyz[1];
                        result[2] = target.xyz[2];
                        return result;
                    }
                        
                    // std::cout << "deque_size: " << history_info_.size() << std::endl;
                    // std::cout << "time: " << tt << std::endl;
                    // std::cout << std::endl;
                    // std::cout << "x:" << target.xyz[0] << " x_pred:" << result[0] << std::endl;
                    // std::cout << std::endl;
                }
            }

            return target.xyz;
        }

        if(history_origin_info_.size() > 2)
        {   // Reserve history fitting front element.
            history_origin_info_.pop_front();    
        }

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

        //---------------根据目前队列长度选用合适的滤波器------------------------------------
        //当队列长度小于3，仅更新队列
        // if (history_info_.size() < 5)
        // {
        //     final_target_ = target;
        //     return target.xyz;
        // }
        // else if (history_info_.size() < predict_param_.min_fitting_lens - 1)
        // {  //当队列长度不足时不使用拟合
        //     fitting_disabled_ = true;
        // }
        // else if (target.timestamp - history_info_.front().timestamp >= predict_param_.max_time_delta)
        // {  //当队列时间跨度过长时不使用拟合
        //     history_info_.pop_front();
        //     fitting_disabled_ = true;
        // }
        // else
        // {   //其余状况下皆可以进行拟合
        //     fitting_disabled_ = false;
        //     history_deque_lens_ = 10;
        //     //若队列过长，移除首元素
        //     if(history_info_.size() > history_deque_lens_)
        //     {
        //         history_info_.pop_front();
        //     }
        // }
        // std::cout << "size: " << history_info_.size() << std::endl;
        // 计算目标速度、加速度 
        // 取目标t-2、t-1、t时刻的坐标信息
        auto delta_x_last = (history_info_.at(history_info_.size() - 2).xyz[0] - history_info_.at(history_info_.size() - 3).xyz[0]);
        auto delta_y_last = (history_info_.at(history_info_.size() - 2).xyz[1] - history_info_.at(history_info_.size() - 3).xyz[1]);
        auto delta_t_last = (history_info_.at(history_info_.size() - 2).timestamp - history_info_.at(history_info_.size() - 3).timestamp);
        auto vx_last = delta_x_last / delta_t_last;
        auto vy_last = delta_y_last / delta_t_last;

        auto delta_x_now = (target.xyz[0] + history_info_.at(history_info_.size() - 2).xyz[0]);
        auto delta_y_now = (target.xyz[1] + history_info_.at(history_info_.size() - 2).xyz[1]);

        auto delta_t_now = (target.timestamp - history_info_.at(history_info_.size() - 2).timestamp);
        auto vx_now = delta_x_now / delta_t_now;
        auto vy_now = delta_y_now / delta_t_now;

        auto ax = (vx_now - vx_last) / ((delta_t_now + delta_t_last) / 2);
        auto ay = (vy_now - vy_last) / ((delta_t_now + delta_t_last) / 2);
        
        Eigen::VectorXd measure_vel(2), measure_acc(2);
        measure_vel << vx_now, vy_now;
        measure_acc << ax, ay;
        // bool is_v_filter_ready = pf_v.update(measure_v);
        // Eigen::Vector2d predict_v_xy = pf_v.predict();

        Eigen::Vector2d target_vel = measure_vel;
        Eigen::Vector2d target_acc = measure_acc;
        // if(is_v_filter_ready)
        // {   //若速度粒子滤波器已完成初始化且预测值大小恰当，则对目标速度做滤波
        //     target_v[0] = predict_v_xy[0];
        //     target_v[1] = predict_v_xy[1];
        // }

        if(debug_param_.disable_fitting)
        {
            fitting_disabled_ = true;
        }

        // Eigen::Vector3d result = {0, 0, 0};
        // Eigen::Vector3d result_pf = {0, 0, 0};
        // Eigen::Vector3d result_fitting = {0, 0, 0};
        // Eigen::Vector3d result_ekf = {0, 0, 0};
        // Eigen::Vector3d result_imm = {0, 0, 0};
        // PredictStatus is_pf_available;
        // PredictStatus is_fitting_available;
        // PredictStatus is_ekf_available;
        // PredictStatus is_imm_available;

        // std::cout << "fitting_status: " << fitting_disabled_ << std::endl;
        // std::cout << "filter_status: " << filter_disabled_ << std::endl;
        if(!filter_disabled_ && fitting_disabled_)
        {   //禁用曲线拟合
            //需注意粒子滤波使用相对时间（自上一次检测时所经过ms数），拟合使用自首帧所经过时间
            // auto is_pf_available = predict_pf_run(target, result_pf, delta_time_estimate);
            // std::cout << "Model_based:" << target_ptr->system_model << std::endl; 
            if(target_ptr->system_model == CSMODEL)
            {   // 基于CS模型的卡尔曼滤波
                is_ekf_available = predict_ekf_run(CAMERA_Z_DIRECTION, target, result_ekf, target_vel, target_acc, delta_time_estimate);
                std::cout << "cs pred..." << std::endl;
            }
            else if(target_ptr->system_model == IMMMODEL)
            {   //IMM
                is_imm_available = predict_based_imm(target, result_imm, target_vel, ax, delta_time_estimate);
                std::cout << "imm pred..." << std::endl;
            }

            if(is_ekf_available.xyz_status[0])
            {   //目前SingerModel仅对x方向机动做预测
                result[0] = result_ekf[0];
            }
            else if(is_imm_available.xyz_status[0])
                result[0] = result_imm[0];
            else
                result[0] = target.xyz[0];
            
            if(is_imm_available.xyz_status[1])
                result[1] = result_imm[1];
            else if(is_ekf_available.xyz_status[1])
                result[1] = result_ekf[1];
            else
                result[1] = target.xyz[1];
                       
            if(is_imm_available.xyz_status[2])
                result[2] = result_imm[2];
            else if(is_ekf_available.xyz_status[2])
                result[2] = result_ekf[2];
            else
                result[2] = target.xyz[2];
        }

        if(filter_disabled_ && !fitting_disabled_)
        {   //禁用滤波

            // std::cout << "fitting pred..." << std::endl;

            // 击打前哨站模式
            if(target_ptr->is_sentry_mode && target_ptr->sentry_armor_status == NORMAL)
            {
                is_fitting_available = spinningPredict(false, target, result_fitting, time_estimate);  
            }
            else if(target_ptr->is_sentry_mode && target_ptr->sentry_armor_status == CONTROLLED)
            {
                is_fitting_available = spinningPredict(true, target, result_fitting, time_estimate);  
            }

            // 反陀螺模式
            if(target_ptr->is_spinning && target_ptr->spinning_status == STILL_SPINNING)
            {   
                // std::cout << "STILL_SPINNING"  << std::endl;
                is_fitting_available = couple_fitting_predict(true, target, result_fitting, time_estimate);  
            }
            else if(target_ptr->is_spinning && target_ptr->spinning_status == MOVEMENT_SPINNING)
            {
                // std::cout << "MOVEMENT_SPINNING" << std::endl;

                is_fitting_available = couple_fitting_predict(false, target, result_fitting, time_estimate);  
            }

            if(is_fitting_available.xyz_status[0])
                result[0] = result_fitting[0];
            else
                result[0] = target.xyz[0];

            if(is_fitting_available.xyz_status[1])
                result[1] = result_fitting[1];
            else
                result[1] = target.xyz[1];

            if(is_fitting_available.xyz_status[2])
                result[2] = result_fitting[2];
            else
                result[2] = target.xyz[2];
        }

        // std::cout << 1 << std::endl;
        // if(!fitting_disabled_ && !filter_disabled_)
        // {   //卡尔曼滤波和曲线拟合异步运行，对二者预测结果进行融合

        //     // auto get_pf_available = std::async(std::launch::async, [=, &result_pf](){return predict_pf_run(target, result_pf, delta_time_estimate);});
        //     auto get_ekf_available = std::async(std::launch::async, [=, &result_ekf](){return predict_ekf_run(target, result_ekf, target_v, ax, delta_time_estimate);});
        
        //     // 轨迹拟合（解耦）
        //     // auto get_fitting_available = std::async(std::launch::async, [=, &result_fitting](){return uncouple_fitting_predict(result_fitting, time_estimate);});

        //     //小陀螺轨迹拟合（耦合）
        //     auto get_fitting_available = std::async(std::launch::async, [=, &result_fitting](){return couple_fitting_predict(result_fitting, time_estimate);});
        
        //     // is_pf_available = get_pf_available.get();
        //     is_ekf_available = get_ekf_available.get();
        //     is_fitting_available = get_fitting_available.get();

        //     // 进行融合
        //     if(is_fitting_available.xyz_status[0])
        //     {
        //         result[0] = result_fitting[0];
        //     }
        //     // else if(is_pf_available.xyz_status[0])
        //     // {
        //     //     result[0] = result_pf[0];
        //     // }
        //     else if(is_ekf_available.xyz_status[0])
        //     {
        //         result[0] = result_ekf[0];
        //     }
        //     else
        //     {
        //         result[0] = target.xyz[0];
        //     }

        //     if(is_fitting_available.xyz_status[1])
        //     {
        //         result[1] = result_fitting[1];
        //     }
        //     // else if(is_pf_available.xyz_status[1])
        //     // {
        //     //     result[1] = result_pf[1];
        //     // }
        //     else
        //     {
        //         result[1] = target.xyz[1];
        //     }

        //     if(is_fitting_available.xyz_status[2] && !fitting_disabled_)
        //     {
        //         result[2] = result_fitting[2];
        //     }
        //     else
        //     {
        //         result[2] = target.xyz[2];
        //     }
        // }


        if(fitting_disabled_ && filter_disabled_)
        {
            result = target.xyz;
        }
        
        // std::cout << 3 << std::endl;

        // result = result_pf;
        auto t2 = std::chrono::steady_clock::now();
        double dr_ms = std::chrono::duration<double, std::milli>(t2-t1).count();
        // if(timestamp % 10 == 0)
        delta_time_estimate = 0;
        // result_pf = target.xyz;

        if(debug_param_.draw_predict && is_ekf_available.xyz_status[0])
        {
            // double x_offset = 400;
            // double y_offset = 400;
            // double z_offset = 200;
            // if (cnt < 2000)
            // {
            //     auto x = cnt * 5;
            //     cv::circle(pic_x, cv::Point2f((timestamp) / 25, xyz[0] * 90 + x_offset), 1, cv::Scalar(0, 0, 255), 1);
            //     // cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[0] * 100 + x_offset),1,cv::Scalar(0,255,0),1);
            //     cv::circle(pic_x, cv::Point2f((timestamp + delta_time_estimate) / 25, result_ekf[0] * 90 + x_offset), 1, cv::Scalar(255, 255, 255), 1);
            //     // cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[0] * 100 + x_offset),1,cv::Scalar(255,255,0),1);
            //     // cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result[0]+ 200),1,cv::Scalar(255,255,255),1);


            //     // cv::circle(pic_y,cv::Point2f((timestamp) / 10,xyz[1] * 100 + y_offset),1,cv::Scalar(0,0,255),1);
            //     // cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[1] * 100 + y_offset),1,cv::Scalar(0,255,0),1);
            //     // cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[1] * 100 + y_offset),1,cv::Scalar(255,255,0),1);
            //     // // cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result[1]+ 200),1,cv::Scalar(255,255,255),1);

            //     // cv::circle(pic_z,cv::Point2f((timestamp) / 10,xyz[2] * 100 + z_offset),1,cv::Scalar(0,0,255),1);
            //     // cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[2] * 100  + z_offset),1,cv::Scalar(0,255,0),1);
            //     // cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[2] * 100 + z_offset),1,cv::Scalar(255,255,0),1);
            //     // cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result[2]),1,cv::Scalar(255,255,255),1);
            //     cnt++;
            // }
            // cv::imshow("result_x",pic_x);
            // // cv::imshow("result_y",pic_y);
            // // cv::imshow("result_z",pic_z);
            // cv::waitKey(1);

            //draw curve
            float mean_vx = (predict_vx_[0] + predict_vx_[1] + predict_vx_[2] + predict_vx_[3]) / 4.0;
            char ch1[10];
            sprintf(ch1, "%.4f", mean_vx);
            std::string str1 = ch1;
            float k1 = 50;
            cv::line(src, cv::Point(0, 250), cv::Point(300, 250), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::line(src, cv::Point(0, 250 + mean_vx * k1), cv::Point(300, 250 + mean_vx * k1), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
            cv::putText(src, str1, cv::Point(310, 250 + mean_vx * k1), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 0));
            cv::line(src, cv::Point(60, 250 + predict_vx_[3] * k1), cv::Point(140, 250 + predict_vx_[2] * k1), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            cv::line(src, cv::Point(140, 250 + predict_vx_[2] * k1), cv::Point(210, 250 + predict_vx_[1] * k1), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            cv::line(src, cv::Point(210, 250 + predict_vx_[1] * k1), cv::Point(300, 250 + predict_vx_[0] * k1), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            
            // std::cout << "his_vx: " << history_vx_[0] << " " << history_vx_[1] << " " << history_vx_[2] << " " << history_vx_[3] << std::endl;

            float his_mean_vx = (history_vx_[0] + history_vx_[1] + history_vx_[2] + history_vx_[3]) / 4.0;
            char ch3[10];
            sprintf(ch3, "%.4f", his_mean_vx);
            std::string str3 = ch3;
            float k3 = 30;
            cv::line(src, cv::Point(300, 250), cv::Point(600, 250), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::line(src, cv::Point(300, 250 + his_mean_vx * k3), cv::Point(600, 250 + his_mean_vx * k3), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
            cv::putText(src, str3, cv::Point(610, 250 + his_mean_vx * k3), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 0));
            cv::line(src, cv::Point(360, 250 + history_vx_[3] * k3), cv::Point(440, 250 + history_vx_[2] * k3), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            cv::line(src, cv::Point(440, 250 + history_vx_[2] * k3), cv::Point(510, 250 + history_vx_[1] * k3), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            cv::line(src, cv::Point(510, 250 + history_vx_[1] * k3), cv::Point(600, 250 + history_vx_[0] * k3), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
       
            // float mean_acc = (predict_acc_[0] + predict_acc_[1] + predict_acc_[2] + predict_acc_[3]) / 4.0;
            // char ch2[10];
            // sprintf(ch2, "%.5f", mean_acc);
            // std::string str2 = ch2;
            // float k2 = 20000;
            // cv::line(src, cv::Point(300, 250), cv::Point(600, 250), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            // cv::line(src, cv::Point(300, 250 + mean_acc * k2), cv::Point(600, 250 + mean_acc * k2), cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
            // cv::putText(src, str2, cv::Point(610, 250 + mean_acc * k2), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 0));
            // cv::line(src, cv::Point(360, 250 + predict_acc_[3] * k2), cv::Point(440, 250 + predict_acc_[2] * k2), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            // cv::line(src, cv::Point(440, 250 + predict_acc_[2] * k2), cv::Point(510, 250 + predict_acc_[1] * k2), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
            // cv::line(src, cv::Point(510, 250 + predict_acc_[1] * k2), cv::Point(600, 250 + predict_acc_[0] * k2), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
        }

        final_target_ = target;
        // return target.xyz;
        return result;
    }

    bool ArmorPredictor::setBulletSpeed(double speed)
    {
        predict_param_.bullet_speed = speed;
        return true;
    }

    Eigen::Vector3d ArmorPredictor::shiftWindowFilter(int start_idx)
    {
        //计算最大迭代次数
        auto max_iter = int(history_info_.size() - start_idx) - predict_param_.window_size + 1;
        Eigen::Vector3d total_sum = {0, 0, 0};
        // cout << history_info.size() << endl;
        // cout << max_iter << endl;
        // cout << start_idx << endl;
        if (max_iter == 0 || start_idx < 0)
            return history_info_.back().xyz;
        
        for (int i = 0; i < max_iter; i++)
        {
            Eigen::Vector3d sum = {0,0,0};
            for (int j = 0; j < predict_param_.window_size; j++)
                sum += history_info_.at(start_idx + i + j).xyz;
            total_sum += sum / predict_param_.window_size;
        }
        // cout << history_info.back().xyz << endl;
        // cout << total_sum / max_iter << endl;
        // cout << endl;
        return total_sum / max_iter;
    }

    PredictStatus ArmorPredictor::predict_pf_run(TargetInfo target, Vector3d& result, int time_estimated)
    {
        PredictStatus is_available;
        //采取中心差分法,使用 t, t-1, t-2时刻速度,计算t-1时刻的速度
        auto target_prev = history_info_.at(history_info_.size() - 3);
        auto target_next = target;
        auto v_xyz = (target_next.xyz - target_prev.xyz) / (target_next.timestamp - target_prev.timestamp) * 1e3;
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
            predict_x = target.xyz[0] + result_v[0] * (time_estimated + t) / 1e3;
            predict_y = target.xyz[1] + result_v[1] * (time_estimated + t) / 1e3;
        }
        else
        {
            predict_x = target.xyz[0];
            predict_y = target.xyz[1];       
        }

        result << predict_x, predict_y, target.xyz[2];
        // cout<<result<<endl;

        return is_available;
    }

    PredictStatus ArmorPredictor::uncouple_fitting_predict(Eigen::Vector3d& result, int time_estimated)
    {
        //0.1的位置使用0初始化会导致拟合结果出错
        double params_x[4] = {0,0,0,0};            // 参数的估计值
        double params_y[4] = {0,0,0,0};            // 参数的估计值

        ceres::Problem problem_x;
        ceres::Problem problem_y;

        ceres::Solver::Options options_x;
        ceres::Solver::Options options_y;

        ceres::Solver::Summary summary_x;                // 优化信息
        ceres::Solver::Summary summary_y;                // 优化信息

        options_x.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
        options_y.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解

        //求直流分量
        Eigen::Vector3d sum = {0,0,0};
        for (auto target_info : history_info_)
        {
            sum += target_info.xyz;
        }
        auto dc = sum / history_info_.size();
        // auto dc = history_info.at(history_info.size() - 1).xyz;
        params_x[0] = dc[0];
        params_y[0] = dc[1];
        
        for (auto target_info : history_info_)
        {
            // std::cout << "T : " << target_info.timestamp / 1e3 << " X:" << target_info.xyz[0] << " Y:" << target_info.xyz[1] << std::endl;
            problem_x.AddResidualBlock (     // 向问题中添加误差项
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 2> ( 
                    new CurveFittingCost (0, target_info.timestamp / 1e3, 0, (target_info.xyz[0] - params_x[0]), 0)
                ),
                new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
                &params_x[1]                 // 待估计参数
            );
            problem_y.AddResidualBlock(     // 向问题中添加误差项 
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 2> ( 
                    new CurveFittingCost (0, target_info.timestamp / 1e3, 0, (target_info.xyz[1] - params_y[0]), 0)
                ),
            new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
                &params_y[1]                 // 待估计参数
            );
        }

        // cout<<endl;
        // problem_x.SetParameterUpperBound(&params_x[1],0,20);
        // problem_x.SetParameterLowerBound(&params_x[1],0,-20);
        // problem_x.SetParameterUpperBound(&params_x[1],1,0.5);
        // problem_x.SetParameterLowerBound(&params_x[1],1,-0.5);
        // problem_x.SetParameterUpperBound(&params_x[1],2,CV_PI);
        // problem_x.SetParameterLowerBound(&params_x[1],2,-CV_PI);

        // problem_y.SetParameterUpperBound(&params_y[1],0,20);
        // problem_y.SetParameterLowerBound(&params_y[1],0,-20);
        // problem_y.SetParameterUpperBound(&params_y[1],1,0.5);
        // problem_y.SetParameterLowerBound(&params_y[1],1,-0.5);
        // problem_y.SetParameterUpperBound(&params_y[1],2,CV_PI);
        // problem_y.SetParameterLowerBound(&params_y[1],2,-CV_PI);

        // problem_x.SetParameterUpperBound(&params_x[1],0,5);
        // problem_x.SetParameterLowerBound(&params_x[1],0,-5);
        // problem_x.SetParameterUpperBound(&params_x[1],1,5);
        // problem_x.SetParameterLowerBound(&params_x[1],1,-5);

        // problem_y.SetParameterUpperBound(&params_y[1],0,5);
        // problem_y.SetParameterLowerBound(&params_y[1],0,-5);
        // problem_y.SetParameterUpperBound(&params_y[1],1,5);
        // problem_y.SetParameterLowerBound(&params_y[1],1,-5);
        
        //异步计算
        auto status_solve_x = std::async(std::launch::deferred, [&](){ceres::Solve(options_x, &problem_x, &summary_x);});
        auto status_solve_y = std::async(std::launch::deferred, [&](){ceres::Solve(options_y, &problem_y, &summary_y);});

        status_solve_x.wait();
        status_solve_y.wait();

        auto x_cost = summary_x.final_cost;
        auto y_cost = summary_y.final_cost;
        // cout<<x_cost<<endl;

        PredictStatus is_available;

        is_available.xyz_status[0] = (x_cost <= predict_param_.max_cost);
        is_available.xyz_status[1] = (y_cost <= predict_param_.max_cost);
        // cout<<z_cost<<endl;
        
        // std::cout << "X:" << params_x[0] << " " << params_x[1] << " " << params_x[2] << " " << params_x[3] << std::endl; 
        // std::cout << "Y:" << params_y[0] << " " << params_y[1] << " " << params_y[2] << " " << params_y[3] << std::endl;
        // cout<<summary_y.BriefReport()<<endl;
        // cout<<time_estimated<<endl;
        // cout<<bullet_speed<<endl;

        auto x_pred = params_x[0] + params_x[1] * (time_estimated / 1e3) + params_x[2] * pow((time_estimated / 1e3), 2);
        auto y_pred = params_y[0] + params_y[1] * (time_estimated / 1e3) + params_y[2] * pow((time_estimated / 1e3), 2);  
        // auto x_pred = params_x[0] + params_x[1] * cos(params_x[2] * (time_estimated / 1e3) + params_x[3]);
        // auto y_pred = params_y[0] + params_y[1] * cos(params_y[2] * (time_estimated / 1e3) + params_y[3]);
        // auto x_pred = params_x[0] + params_x[1] * cos(params_x[3] * (time_estimated / 1e3)) + params_x[2] * sin(params_x[3] * (time_estimated / 1e3));
        // auto y_pred = params_y[0] + params_y[1] * cos(params_y[3] * (time_estimated / 1e3)) + params_y[2] * sin(params_y[3] * (time_estimated / 1e3));

        // std::cout << x_pred << " : " << y_pred << std::endl;
        // std::cout << "..........." << std::endl;
        
        result = {x_pred, y_pred, dc[2]};
        return is_available;
    }

    PredictStatus ArmorPredictor::couple_fitting_predict(bool is_still_spinning, TargetInfo target, Eigen::Vector3d& result, int time_estimated)
    {   
        /**
         * @brief 车辆小陀螺运动轨迹拟合(已知量：角速度&陀螺半径）
         * 若目标仅处于原地小陀螺状态，则剔除掉模型中的横移项，直接给平动项乘以系数0。
         */
        
        auto time_start = std::chrono::steady_clock::now();
        double params[8] = {fitting_params_[0], fitting_params_[1], fitting_params_[2], fitting_params_[3],
            fitting_params_[4], fitting_params_[5], fitting_params_[6], fitting_params_[7]};
        // double params[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

        double params_y[8] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

        // x-axis
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        
        options.max_num_iterations = 20;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        
        double x0 = history_info_.front().xyz[0];
        int st = history_info_.begin()->timestamp;
        last_start_timestamp_ = st;

        // y-axis
        ceres::Problem problem_y;
        ceres::Solver::Options options_y;
        ceres::Solver::Summary summary_y;
        options_y.max_num_iterations = 20;
        options_y.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout =false;

        //TODO: camera frame's y coord matches to gyro frame's z.
        double y0 = history_info_.front().xyz[2];

        double x_sum = 0;
        double y_sum = 0;
        // if(!is_still_spinning)
        // {
        //     for(auto& target_info : history_info_)
        //     {   
        //         x_sum += target_info.xyz[0];
        //         y_sum += target_info.xyz[2];
        //         // std::cout << std::endl;
        //         // std::cout << "t:" << (target_info.timestamp - st) / 1e3 << std::endl;
        //         // std::cout << std::endl;

        //         problem.AddResidualBlock(
        //             new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1, 1, 1, 1>
        //             (
        //                 new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp) / 1e3, 1)
        //             ),
        //             new ceres::CauchyLoss(0.5),
        //             &params[0],
        //             &params[1],
        //             &params[2],
        //             &params[3],
        //             &params[4],
        //             &params[5],
        //             &params[6],
        //             &params[7]
        //         );
        //         problem.AddResidualBlock(
        //             new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1, 1, 1, 1>
        //             (
        //                 new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp) / 1e3, 1)
        //             ),
        //             new ceres::CauchyLoss(0.5),
        //             &params[0],
        //             &params[1],
        //             &params[2],
        //             &params[3],
        //             &params[4],
        //             &params[5],
        //             &params[6],
        //             &params[7]
        //         );
                // problem.AddResidualBlock(
                //     new ceres::AutoDiffCostFunction<XAxisFitting, 1, 1, 1, 1, 1, 1>
                //     (
                //         new XAxisFitting(target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e3)
                //     ),
                //     new ceres::CauchyLoss(0.5),
                //     &params[0],
                //     &params[1],
                //     &params[2],
                //     &params[3],
                //     &params[4]
                // );

                // problem.AddResidualBlock(
                //     new ceres::AutoDiffCostFunction<YAxisFitting, 1, 1, 1, 1, 1, 1>
                //     (
                //         new YAxisFitting(target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e3)
                //     ),
                //     new ceres::CauchyLoss(0.5),
                //     &params[0],
                //     &params[1],
                //     &params[2],
                //     &params[3],
                //     &params[4]
                // );
        //     }
        // }
        // else
        // {
            for(auto& target_info : history_info_)
            {   
                // Y-axis matches to camera.
                y_sum += target_info.xyz[2];
                
                // std::cout << std::endl;
                // std::cout << "t:" << (target_info.timestamp- st) / 1e3 << std::endl;
                // std::cout << std::endl;

                // problem.AddResidualBlock(
                //     new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1, 1, 1, 1>
                //     (
                //         new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp - st) / 1e3, 0)
                //     ),
                //     new ceres::CauchyLoss(0.5),
                //     &params[0],
                //     &params[1],
                //     &params[2],
                //     &params[3],
                //     &params[4],
                //     &params[5],
                //     &params[6],   
                //     &params[7]    
                // );
                // problem.AddResidualBlock(
                //     new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1, 1, 1, 1>
                //     (
                //         new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp - st) / 1e3, 0)
                //     ),
                //     new ceres::CauchyLoss(0.5),
                //     &params[0],
                //     &params[1],
                //     &params[2],
                //     &params[3],
                //     &params[4],
                //     &params[5],
                //     &params[6],
                //     &params[7]
                // );

                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1>
                    (
                        // new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp - st) / 1e3, 0)
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp - st) / 1e3, x0)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &params[0],
                    &params[1],
                    &params[2],
                    &params[3],
                    &params[4]
                );

                problem_y.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1>
                    (
                        // new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp - st) / 1e3, 0)
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp), y0)
                    ),
                    new ceres::CauchyLoss(0.25),
                    &params_y[0],
                    &params_y[1],
                    &params_y[2],
                    &params_y[3],
                    &params_y[4]
                );

                // problem.AddResidualBlock(
                //     new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1, 1, 1>
                //     (
                //         // new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp - st) / 1e3, 0)
                //         new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[2], (target_info.timestamp) / 1e3, 0)
                //     ),
                //     new ceres::CauchyLoss(0.5),
                //     &params[0],
                //     &params[1],
                //     &params[2],
                //     &params[3],
                //     &params[4]
                // );
            }
        // }
        // problem.SetParameterUpperBound(&params[0], 0, 4 * M_PI);
        // problem.SetParameterLowerBound(&params[0], 0, 0.25 * M_PI);

        // problem.SetParameterUpperBound(&params[1], 0, 0.5 * M_PI);
        // problem.SetParameterLowerBound(&params[1], 0, -0.5 * M_PI);
        
        // problem.SetParameterUpperBound(&params[2], 0, 2);
        // problem.SetParameterLowerBound(&params[2], 0, -2);
        
        // problem.SetParameterUpperBound(&params[3], 0, 20);
        // problem.SetParameterLowerBound(&params[3], 0, 20);
        
        // problem.SetParameterUpperBound(&params[4], 0, 20);
        // problem.SetParameterLowerBound(&params[4], 0, 20);
        
        // problem.SetParameterUpperBound(&params[5], 0, 0.50);
        // problem.SetParameterLowerBound(&params[5], 0, 0.12);
        // problem.SetParameterUpperBound(&params[6], 0, 0.50);
        // problem.SetParameterLowerBound(&params[6], 0, 0.12);
        
        // problem.SetParameterUpperBound(&params[7], 0, 0.5 * M_PI);
        // problem.SetParameterLowerBound(&params[7], 0, -0.5 * M_PI);

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

        // ceres::Solve(options, &problem, &summary);
        auto x_future = std::async(std::launch::deferred, [&](){ceres::Solve(options, &problem, &summary);});
        auto y_future = std::async(std::launch::deferred, [&](){ceres::Solve(options_y, &problem_y, &summary_y);});

        x_future.wait();
        y_future.wait();

        auto x_cost = summary.final_cost;
        auto y_cost = summary_y.final_cost;
        
        auto time_now = std::chrono::steady_clock::now();
        auto t = std::chrono::duration<double, std::milli>(time_now - time_start).count();
        std::cout << "fitting_time: " << t << "ms" << std::endl;
        std::cout << "x_cost:" << x_cost << " y_cost:" << y_cost << std::endl;

        PredictStatus is_available;
        is_available.xyz_status[0] = (x_cost <= predict_param_.max_cost);
        is_available.xyz_status[2] = (y_cost <= predict_param_.max_cost);        

        if(is_available.xyz_status[0])
            is_predicted = true;
        else
            is_predicted = false;
        
        // std::cout << "is_predicted: " << is_predicted << std::endl;
        // is_available.xyz_status[1] = (summary.final_cost <= predict_param_.max_cost);
        // std::cout << "xyz_status[0]:" << is_available.xyz_status[0] << std::endl;
        // std::cout << "xyz_status[1]:" << is_available.xyz_status[1] << std::endl;

        // std::cout << std::endl;
        // std::cout << "estimated_time: " << time_estimated / 1e3 << std::endl;
        // std::cout << std::endl;
        double x_pred, y_pred;
        // if(!is_still_spinning)
        // {
        //     x_pred = params[3] - params[5] * ceres::cos(params[0] * ((time_estimated - st) / 1e3) + params[7]) + params[2] * ((time_estimated - st) / 1e3) * ceres::cos(params[1]);
        //     y_pred = params[4] - params[6] * ceres::sin(params[0] * ((time_estimated - st) / 1e3) + params[7]) + params[2] * ((time_estimated - st) / 1e3) * ceres::sin(params[1]);
        // }
        // else
        // {
            // x_pred = params[3] - params[5] * ceres::cos(params[0] * ((time_estimated - st) / 1e3) + params[7]);
            // y_pred = params[4] - params[6] * ceres::sin(params[0] * ((time_estimated - st) / 1e3) + params[7]);

            double start_point = history_info_.front().timestamp;            
            if((time_estimated - start_point) / 1e3 < target.period)
            {
                // x_pred = params[0] * ((time_estimated - st) / 1e3) + x0; // x(t)=k(t+dt)+x0
                x_pred = params[0] * ((time_estimated - st) / 1e3) + params[1]; // x(t)=k(t+dt)+d

                //y(t)=a*(k^2)*(t^2)+(2kad+kb)*t+a(d^2)+bd+c
                // y_pred = params_y[2] * pow(params_y[0], 2) * pow(((time_estimated - st) / 1e3), 2) + ((2.0 * (params_y[0] * params_y[2] * params_y[1]))
                // + (params_y[0] * params_y[3])) * ((time_estimated - st) / 1e3) + params_y[2] * pow(params_y[1], 2) + params_y[3] * params_y[1] + params_y[4];

                //f(t)=a*(t^2)+b*t+c
                // y_pred = params_y[2] * pow(((time_estimated - st) / 1e3), 2) + params_y[3] * ((time_estimated - st) / 1e3) + params_y[4];
            
                //f(t)=(b/t)+c
                // y_pred = params_y[3] / (time_estimated) + params_y[4];
            
                //f(t)=(1/t)+c
                y_pred = (1.0 / (time_estimated )) + params_y[4];
            }
            else
            {
                if(history_origin_info_.size() == 2)
                {
                    // double last_delta_x0 = history_origin_info_.at(history_origin_info_.size() - 2) - history_origin_info_.at(history_origin_info_.size() - 3);
                    double cur_delta_x0 = history_origin_info_.at(history_origin_info_.size() - 1).x - history_origin_info_.at(history_origin_info_.size() - 2).x;
                    double cur_delta_y0 = history_origin_info_.at(history_origin_info_.size() - 1).y - history_origin_info_.at(history_origin_info_.size() - 2).y;
                    // double delta_ave = (last_delta_x0 + cur_delta_x0) / 2.0;
                    
                    // std::cout << "cur_delta:" << cur_delta_x0 << std::endl;
                    
                    // double x_pred_sum = 0;
                    // double ave_x_pred = 0;
                    // if(history_delta_x_pred_.size() != 0)
                    // {
                    //     for(auto& delta_x_pred : history_delta_x_pred_)
                    //         x_pred_sum += delta_x_pred;
                    //     ave_x_pred = x_pred_sum / history_delta_x_pred_.size();
                    // }
                    
                    if(!is_still_spinning)
                    {
                        x_pred = params[0] * ((time_estimated - st) / 1e3 - target.period) + params[1] - cur_delta_x0; // x(t)=k(t+dt-T)+d
                        // y_pred = params_y[2] * pow(params_y[0], 2) * pow(((time_estimated - st) / 1e3 - target.period), 2)
                        //         + ((2.0 * (params_y[0] * params_y[2] * params_y[1]))
                        //         + (params_y[0] * params_y[3])) * ((time_estimated - st) / 1e3 - target.period)
                        //         + params_y[2] * pow(params_y[1], 2) + params_y[3] * params_y[1] + params_y[4] - cur_delta_y0;

                        //f(t)=a*(t^2)+b*t+c
                        // y_pred = params_y[2] * pow(((time_estimated - st) / 1e3 - target.period), 2)
                        //         + params_y[3] * ((time_estimated - st) / 1e3 - target.period) + params_y[4] - cur_delta_y0;

                        //f(t)=(b/t)+c
                        // y_pred = params_y[3] / ((time_estimated) - target.period * 1e3) + params_y[4] - cur_delta_y0;

                        //f(t)=(1/t)+c
                        y_pred = (1.0 / (time_estimated - target.period * 1e3)) + params_y[4];
                    }
                    else
                    {
                        x_pred = params[0] * ((time_estimated - st) / 1e3 - target.period) + params[1]; // x(t)=k(t+dt-T)+d
                        // y_pred = params_y[2] * pow(params_y[0], 2) * pow(((time_estimated - st) / 1e3 - target.period), 2)
                        //         + ((2.0 * (params_y[0] * params_y[2] * params_y[1]))
                        //         + (params_y[0] * params_y[3])) * ((time_estimated - st) / 1e3 - target.period)
                        //         + params_y[2] * pow(params_y[1], 2) + params_y[3] * params_y[1] + params_y[4];
                       
                        //f(t)=a*(t^2)+b*t+c
                        // y_pred = params_y[2] * pow(((time_estimated - st) / 1e3 - target.period), 2)
                        //         + params_y[3] * ((time_estimated - st) / 1e3 - target.period) + params_y[4];

                        //f(t)=(b/t)+c
                        // y_pred = params_y[3] / ((time_estimated) - target.period * 1e3) + params_y[4];

                        //f(t)=(1/t)+c
                        y_pred = (1.0 / ((time_estimated - target.period * 1e3))) + params_y[4];
                    }

                    // if(ave_x_pred)
                    // {
                    //     double xx = x_pred - target.xyz[0];
                    //     if(abs(xx) > abs(ave_x_pred))
                    //         x_pred = target.xyz[0] + ave_x_pred;
                    // }
                    // int flag = -1;
                    // for(auto& aim : history_info_)
                    // {
                        // if(x_origin > 0)
                        // {
                        
                        // std::cout << "Target_direction: " <<  target.is_clockwise << std::endl;
                        double x_origin = history_origin_info_.at(history_origin_info_.size() - 1).x;
                        double y_origin = history_origin_info_.at(history_origin_info_.size() - 1).y;
                        if(target.is_clockwise)
                        {
                            if(x_pred > x_origin * 0.85)
                                x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[0];
                            if(y_pred > y_origin * 0.85)
                                y_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[2];

                        }
                        else
                        {
                            if(x_pred < x_origin * 0.85)
                                x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[0];
                            if(y_pred < y_origin * 0.85)
                                y_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[2];
                        }

                        // }
                        // if(x_origin < 0)
                        // {
                            // if()
                        // }
                    // }
                    // std::cout << "x_pred: " << x_pred << std::endl;
                    // std::cout << std::endl;
                }
                else
                {
                    if(is_still_spinning)
                    {
                        x_pred = params[0] * ((time_estimated - st) / 1e3 - target.period) + params[1]; // x(t)=k(t+dt-T)+d
                        // y_pred = params_y[2] * pow(params_y[0], 2) * pow(((time_estimated - st) / 1e3 - target.period), 2)
                        //         + ((2.0 * (params_y[0] * params_y[2] * params_y[1]))
                        //         + (params_y[0] * params_y[3])) * ((time_estimated - st) / 1e3 - target.period)
                        //         + params_y[2] * pow(params_y[1], 2) + params_y[3] * params_y[1] + params_y[4];

                        //f(t)=a*(t^2)+b*t+c
                        // y_pred = params_y[2] * pow(((time_estimated - st) / 1e3 - target.period), 2)
                        //         + params_y[3] * ((time_estimated - st) / 1e3 - target.period) + params_y[4];

                        //f(t)=(b/t)+c
                        // y_pred = params_y[3] / ((time_estimated) - target.period * 1e3) + params_y[4];

                        y_pred = (1.0 / ((time_estimated - target.period * 1e3))) + params_y[4];
                            
                        if(history_origin_info_.size() != 0)
                        {
                            double x_origin = history_origin_info_.at(history_origin_info_.size() - 1).x;
                            double y_origin = history_origin_info_.at(history_origin_info_.size() - 1).y;
                            if(target.is_clockwise)
                            {
                                if(x_pred > x_origin * 0.80)
                                    x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[0];
                                if(y_pred > y_origin * 0.80)
                                    y_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[2];
                            }
                            else
                            {
                                if(x_pred < x_origin * 0.80)
                                    x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[0];
                                if(y_pred < y_origin * 0.80)
                                    y_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[2];
                            }
                        }
                    }
                    else
                    {
                        x_pred = history_info_.at(history_info_.size() - 1).xyz[0];
                        y_pred = history_info_.at(history_info_.size() - 1).xyz[2];
                    }
                }

                y_pred = y_sum / history_info_.size();
                // double tt = (time_estimated - start_point) / 1e3 - target.period;
                // if(history_info_.size() > 6)
                // {
                    // int flag = -1;
                    // std::cout << "delta_t:" << tt << std::endl;
                    // for(int ii = 0; ii < history_info_.size(); ii++)
                    // {
                    //     if(history_info_[ii].timestamp > (start_point + tt))
                    //     {   
                    //         flag = ii;
                    //         break;
                    //     }
                    //     else
                    //         continue;
                    // }
                    // std::cout << "id:" << flag << std::endl;
                    // x_pred = history_info_.at(flag).xyz[0];

                    // x_pred = history_info_.at((int)(history_info_.size() / 2)).xyz[0];
                // }
                // else
                // {
                    // x_pred = params[0] * ((target.timestamp - st) / 1e3 - target.period) + x0; // x(t)=k(t+dt-T)+x0
                    // x_pred = params[0] * ((target.timestamp - st) / 1e3 - target.period) + params[1]; // x(t)=k(t+dt-T)+d
                    // x_pred = target.xyz[0];
                //     x_pred = history_info_.at(3).xyz[0];
                // }
                    // x_pred = history_info_.at(history_info_.size() - 1).xyz[0];
            }
            // x(t)=kt+d
            // y(t)=a*(k^2)*(t^2)+(2kad+kb)*t+a(d^2)+bd+c
            // y_pred = params[2] * pow(params[0], 2) * pow(((time_estimated) / 1e3), 2) + 
            //     (2 * params[0] * params[2] * params[1] + params[0] * params[3]) *
            //     ((time_estimated) / 1e3) + params[2] * pow(params[1], 2) + 
            //     params[3] * params[1] + params[4];
        // }

        std::memcpy(fitting_params_, params, sizeof(fitting_params_));
        std::memcpy(fitting_y_params_, params_y, sizeof(fitting_y_params_));

        // std::cout << "w:" << params[0] << " theta:" << params[1] << " V:" << params[2] << " x0:" 
        //           << params[3] << " y0:" << params[4] << " a:" << params[5] << " b:" << params[6] << " phi:" << params[7] << std::endl;

        // std::cout << "k:" << params[0] << " d:" << params[1] << " a:" << params[2] << " b:" 
        //           << params[3] << " c:" << params[4] << std::endl;

        std::cout << "k:" << params_y[0] << " d:" << params_y[1] << " a:" << params_y[2] << " b:" 
                  << params_y[3] << " c:" << params_y[4] << std::endl;

        // std::cout << "x_: " << target.xyz[0] << " x_pred: " << x_pred << std::endl; 
        // std::cout << std::endl;
        // std::cout << "x:" << x_pred << " y:" << target.xyz[1] << " z:" << target.xyz[2] << " y_pred:" << y_pred << std::endl;
        // std::cout << std::endl;

        double x_front = history_info_.front().xyz[0];
        double x_back = history_info_.back().xyz[0];
        last_end_x_ = x_back;
        // if(x_back < 0)
        // {
        //     if(abs(x_pred) + 0.15 < abs(x_back))
        //     {
        //         x_pred = x_back + 0.15;
        //     }
        // }
        // else
        // {
        //     if(x_pred > (x_back + 0.15))
        //     {
        //         x_pred = (x_back + 0.15);
        //     }
        // }

        //     if(abs(x_pred) > abs(history_info_.front().xyz[0])) // Back in time.
        //         x_pred = history_info_.at(4).xyz[0];
        // // }

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
        
        // result = {x_pred, target.xyz[1], target.xyz[2]};
        // result = {x_pred, target.xyz[1], y_pred}; //camera frame
        result = {target.xyz[0], target.xyz[1], y_pred}; //camera frame
        
        // std::cout << "x:" << target.xyz[0] << " x_pred:" << x_pred << std::endl;
        std::cout << "y:" << target.xyz[2] << " y_pred:" << y_pred << std::endl;

        return is_available;
    }

    PredictStatus ArmorPredictor::spinningPredict(bool is_controlled, TargetInfo& target, Eigen::Vector3d& result, int time_estimated)
    {  
        /**
         * @brief 前哨站旋转装甲运动预测（已知量：旋转半径&转速），考虑我方占领控制区旋转装甲板转速减半，应加入条件判断。
         * 
         */
        //轨迹拟合
        auto time_start = std::chrono::steady_clock::now();
        double x0, y0, theta;

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;

        options.max_num_iterations = 200;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        if(!is_controlled)
        {
            for(auto& target_info : history_info_)
            {   
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e3, 1)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e3, 1)
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
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(0, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e3, 0.5)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 1, 1, 1>
                    (
                        new CurveFittingCost(1, target_info.xyz[0], target_info.xyz[1], target_info.timestamp / 1e3, 0.5)
                    ),
                    new ceres::CauchyLoss(0.5),
                    &x0,
                    &y0,
                    &theta
                );
            }
        }

        ceres::Solve(options, &problem, &summary);

        auto time_now = std::chrono::steady_clock::now();
        auto t = std::chrono::duration<double, std::milli>(time_now - time_start).count();
        // std::cout << "fitting_fime:" << t / 1e3 << "ms" << std::endl;

        PredictStatus is_available;
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

    void ArmorPredictor::kfInit()
    {
        double alpha = singer_param_.alpha;
        double dt = singer_param_.dt;

        kalman_filter_.F_ << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  
                            0, 1, (1 - exp(-alpha * dt)) / alpha,
		                    0, 0, exp(-alpha * dt);

        kalman_filter_.H_ << 1, 0, 0;

        kalman_filter_.C_ << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                            dt - (1 - exp(-alpha * dt) / alpha),
                            1 - exp(-alpha * dt);
        
        double p = singer_param_.p;
        kalman_filter_.P_ << p, 0, 0,
                             0, p, 0,
                             0, 0, p;

        double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
        double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
        double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
        double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
        double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
        double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));

        double sigma = singer_param_.sigma;
        kalman_filter_.Q_ << 2 * pow(sigma, 2) * alpha * q11, 2 * pow(sigma, 2) * alpha * q12, 2 * pow(sigma, 2) * alpha* q13,
                            2 * pow(sigma, 2) * alpha* q12, 2 * pow(sigma, 2) * alpha* q22, 2 * pow(sigma, 2) * alpha* q23,
		                    2 * pow(sigma, 2) * alpha* q13, 2 * pow(sigma, 2) * alpha* q23, 2 * pow(sigma, 2) * alpha* q33;
        
        double meaCov = singer_param_.r;
        kalman_filter_.R_ << meaCov;
    }

    PredictStatus ArmorPredictor::predict_ekf_run(PredictDirection predictDirection, TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d target_vel, Eigen::Vector2d target_acc, int timestamp)
    {
        PredictStatus is_available;
        // 计算目标速度、加速度 
        //取目标t-2、t-1、t时刻的坐标信息
        // auto delta_x_last = (history_info_.at(history_info_.size() - 2).xyz[0] - history_info_.at(history_info_.size() - 3).xyz[0]);
        // auto delta_t_last = (history_info_.at(history_info_.size() - 2).timestamp - history_info_.at(history_info_.size() - 3).timestamp);
        // auto v_last = delta_x_last / delta_t_last;

        // auto delta_x_now = (target.xyz[0] - history_info_.at(history_info_.size() - 2).xyz[0]);
        // auto delta_t_now = (target.timestamp - history_info_.at(history_info_.size() - 2).timestamp);
        // auto v_now = delta_x_now / delta_t_now;

        // auto ax = (v_now - v_last) / ((delta_t_now + delta_t_last) / 2);

        // SingerState x;

        if(!is_ekf_init)
        {   
            if(predictDirection == CAMERA_X_DIRECTION)
                kalman_filter_.x_ << target.xyz[0], target_vel[0], target_acc[0];
            else if(predictDirection == CAMERA_Y_DIRECTION)
                kalman_filter_.x_ << target.xyz[1], 0, 0;
            else if(predictDirection == CAMERA_Z_DIRECTION)
                kalman_filter_.x_ << target.xyz[2], target_vel[1], target_acc[1]; 
            else if(predictDirection == GYRO_X_DIRECTION)
                kalman_filter_.x_ << target.xyz[0], target_vel[0], target_acc[0];
            else if(predictDirection == GYRO_Y_DIRECTION)
                kalman_filter_.x_ << target.xyz[1], target_vel[1], target_acc[1];
            else if(predictDirection == GYRO_Z_DIRECTION)
                kalman_filter_.x_ << target.xyz[2], 0, 0;
            
            is_available.xyz_status[0] = false;
            is_ekf_init = true;
        }
        else
        {

            // if(!is_ekf_init)
            // {
            //     ekf.init(x);
            //     result << target.xyz[0], target.xyz[1], target.xyz[2];
            //     is_available.xyz_status[0] = false;
            //     is_ekf_init = true;
            // }
            // else
            // {
            //     // auto x_model = singer.f(x, u, timestamp);
            //     ekf.init(x);

            //     //预测
            //     auto x_ekf_pred = ekf.predict(singer, u, timestamp);
                
            //     //更新
            //     SingerPosMeasure pos = pos_model.h(x);
            //     auto x_ekf_update = ekf.update(pos_model, pos);
            //     is_available.xyz_status[0] = true;

            //     // std::cout << "target_x:" << target.xyz[0] << " pred_x:" << x_model[0] << std::endl;

            //     // result << x_ekf_update[0], target.xyz[1], target.xyz[2];
            //     // result << x_model[0], target.xyz[1], target.xyz[2];
            //     result << x_ekf_pred[0], target.xyz[1], target.xyz[2];
            // }
            
            Eigen::VectorXd measurement = Eigen::VectorXd(1);
            
            if(predictDirection == CAMERA_X_DIRECTION)
                measurement << target.xyz[0];
            else if(predictDirection == CAMERA_Y_DIRECTION)
                measurement << target.xyz[1];
            else if(predictDirection == CAMERA_Z_DIRECTION)
                measurement << target.xyz[2];
            else if(predictDirection == GYRO_X_DIRECTION)
                measurement << target.xyz[0];
            else if(predictDirection == GYRO_Y_DIRECTION)
                measurement << target.xyz[1];
            else if(predictDirection == GYRO_Z_DIRECTION)
                measurement << target.xyz[2];
            
            kalman_filter_.Predict();
            kalman_filter_.Update(measurement);

            // Eigen::MatrixXd predictState(3, 1);
			Eigen::VectorXd State(3, 1);
            State << kalman_filter_.x_[0], kalman_filter_.x_[1], kalman_filter_.x_[2];
            
            double alpha = singer_param_.alpha;
            double dt = 5 * singer_param_.dt;

            Eigen::MatrixXd F(3, 3);
            F << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / pow(alpha, 2),
                0, 1, (1 - exp(-alpha * dt)) / alpha,
                0, 0, exp(-alpha * dt); 

            Eigen::MatrixXd control(3, 1);
            control << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                    dt - (1 - exp(-alpha * dt) / alpha), 
                    1 - exp(-alpha * dt);
            
            VectorXd x_pred = F * State + control * State[2];
            
            // double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
            // double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
            // double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
            // double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
            // double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
            // double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));

            // double sigma = singer_param_.sigma;
            // if(ax > 0)
            // {
            //     sigma = ((4 - CV_PI) / CV_PI) * pow(singer_param_.a_max - ax, 2);
            // }
            // else
            // {
            //     sigma = ((4 - CV_PI) / CV_PI) * pow(singer_param_.a_max + ax, 2);
            // }

            // kalman_filter_.Q_ << 2 * sigma * alpha * q11, 2 * sigma * alpha * q12, 2 * sigma * alpha * q13,  
		    //                     2 * sigma * alpha * q12, 2 * sigma * alpha * q22, 2 * sigma * alpha * q23,
		    //                     2 * sigma * alpha * q13, 2 * sigma * alpha* q23, 2 * sigma * alpha * q33;
            
            if(predictDirection == CAMERA_X_DIRECTION)
            {
                result[0] = x_pred[0];
                result[1] = target.xyz[1];
                result[2] = target.xyz[2];
                is_available.xyz_status[0] = true;
            }
            else if(predictDirection == CAMERA_Y_DIRECTION)
            {
                result[0] = target.xyz[0];
                result[1] = x_pred[0];
                result[2] = target.xyz[2];
                is_available.xyz_status[1] = true;
            }
            else if(predictDirection == CAMERA_Z_DIRECTION)
            {
                result[0] = target.xyz[0];
                result[1] = target.xyz[1];
                result[2] = x_pred[0];
                is_available.xyz_status[2] = true;
            }
            else if(predictDirection == GYRO_X_DIRECTION)
            {
                result[0] = x_pred[0];
                result[1] = target.xyz[1];
                result[2] = target.xyz[2];
                is_available.xyz_status[0] = true;
            }
            else if(predictDirection == GYRO_Y_DIRECTION)
            {
                result[0] = target.xyz[0];
                result[1] = x_pred[0];
                result[2] = target.xyz[2];
                is_available.xyz_status[1] = true;

            }
            else if(predictDirection == GYRO_Z_DIRECTION)
            {
                result[0] = target.xyz[0];
                result[1] = target.xyz[1];
                result[2] = x_pred[0];
                is_available.xyz_status[2] = true;
            }
            
            // std::cout << "..." << std::endl;
        }

        return is_available;
    }

    PredictStatus ArmorPredictor::predict_based_imm(TargetInfo target, Eigen::Vector3d& result, Eigen::Vector2d& target_v, double& ax, int timestamp)
    {
        PredictStatus is_available;
        double dt = singer_param_.dt;   
        if(!is_imm_init)
        {
            Eigen::VectorXd x(6);
            x << target.xyz[0], target.xyz[1], 0, 0, 0, 0;
            imm_ = model_generator_.generateIMMModel(x, dt);   
            is_imm_init = true;
        }
        else
        {
            // std::cout << 4 << std::endl;
            Eigen::VectorXd measurement(4);
            measurement << target.xyz[0], target.xyz[1], 0, 0;
            imm_->updateOnce(measurement, dt);

            Eigen::VectorXd State(6);
            State = imm_->x();

            result[0] = State[0];
            result[1] = State[1];
            result[2] = target.xyz[2];
            is_available.xyz_status[0] = true;
            is_available.xyz_status[1] = true;

            // std::cout << 5 << std::endl;
        }
        return is_available;
    }

    void ArmorPredictor::setSingerParam(double& alpha, double& a_max, double& p_max, double& p0)
    {
        // singer.set_alpha(alpha);
        // singer.set_a_max(a_max);
        // singer.set_p_max(p_max);
        // singer.set_p0(p0);       
        // singer.set_sigma(); 
    }

    void ArmorPredictor::set_singer_alpha(double& alpha)
    {
        // singer.set_alpha(alpha);
        this->singer_param_.alpha = alpha;
    }
    void ArmorPredictor::set_singer_a_max(double& a_max)
    {
        // singer.set_a_max(a_max);
        this->singer_param_.a_max = a_max;
    }
    
    void ArmorPredictor::set_singer_p_max(double& p_max)
    {
        // singer.set_p_max(p_max);
        this->singer_param_.p_max = p_max;
    }
    
    void ArmorPredictor::set_singer_p0(double& p0)
    {
        // singer.set_p0(p0);    
        this->singer_param_.p0 = p0;    
    }

    void ArmorPredictor::set_singer_sigma(double& sigma)
    {
        // singer.set_sigma();
        this->singer_param_.sigma = sigma;
    }

    void ArmorPredictor::set_singer_dt(double& dt)
    {
        // singer.set_sigma();
        this->singer_param_.dt = dt;
    }

    void ArmorPredictor::set_singer_p(double& p)
    {
        singer_param_.p = p;
    }

    void ArmorPredictor::set_singer_r(double& r)
    {
        singer_param_.r = r;
    }
} // ArmorPredictor