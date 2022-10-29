/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 12:46:41
 * @LastEditTime: 2022-10-24 14:10:37
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/prediction/prediction.cpp
 */
#include "../../include/prediction/prediction.h"

namespace armor_processor
{
    ArmorPredictor::ArmorPredictor()
    {
        int cnt = 0;
        pic_x = cv::Mat::zeros(500, 2000, CV_8UC3);
        pic_y = cv::Mat::zeros(500, 2000, CV_8UC3);
        pic_z = cv::Mat::zeros(500, 2000, CV_8UC3);
    }

    ArmorPredictor::~ArmorPredictor(){}

    ArmorPredictor::ArmorPredictor(const PredictParam& predict_param, DebugParam& debug_param, std::string filter_param_path)
    : predict_param_(predict_param), debug_param_(debug_param), filter_param_path_(filter_param_path)
    {
        int cnt = 0;
        pic_x = cv::Mat::zeros(500, 2000, CV_8UC3);
        pic_y = cv::Mat::zeros(500, 2000, CV_8UC3);
        pic_z = cv::Mat::zeros(500, 2000, CV_8UC3);

        // config_ = YAML::LoadFile(coord_file);
        // pf_pos.initParam(config_, "pos");
        // pf_v.initParam(config_, "v");
        
        fitting_disabled_ = false;
        is_init = false;
    }

    Eigen::Vector3d ArmorPredictor::predict(Eigen::Vector3d xyz, int timestamp)
    {
        if(!is_init)
        {
            config_ = YAML::LoadFile(filter_param_path_);
            pf_pos.initParam(config_, "pos");
            pf_v.initParam(config_, "v");

            is_init = true;
        }

        auto t1 = std::chrono::steady_clock::now();
        TargetInfo target = {xyz, (int)xyz.norm(), timestamp};
        
        //-----------------对位置进行粒子滤波,以降低测距噪声影响-------------------------------------
        Eigen::VectorXd measure (2);
        measure << xyz[0], xyz[1];
        bool is_pos_filter_ready = pf_pos.update(measure);
        Eigen::VectorXd predict_pos_xy = pf_pos.predict();
        Eigen::Vector3d predict_pos = {predict_pos_xy[0], predict_pos_xy[1], xyz[2]};
        
        //若位置粒子滤波器未完成初始化或滤波结果与目前位置相距过远,则本次不对目标位置做滤波,直接向队列压入原值
        if (!is_pos_filter_ready || (predict_pos - xyz).norm() > 0.1)
        {
            history_info_.push_back(target);
        }
        else
        {  //若位置粒子滤波器已完成初始化且预测值大小恰当,则对目标位置做滤波
            // cout<<"FIL:"<<predict_pos[0] - xyz[0]<<endl;
            target.xyz[0] = predict_pos[0];
            target.xyz[1] = predict_pos[1];
            history_info_.push_back(target);
        }
        
        //-----------------进行滑窗滤波,备选方案,暂未使用-------------------------------------
        auto d_xyz = target.xyz - final_target_.xyz;
        auto delta_t = timestamp - final_target_.timestamp;
        auto last_dist = history_info_.back().dist;
        auto delta_time_estimate = (last_dist / predict_param_.bullet_speed) * 1e3 + predict_param_.shoot_delay;
        auto time_estimate = delta_time_estimate + history_info_.back().timestamp;

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
        if (history_info_.size() < 4)
        {
            final_target_ = target;
            return xyz;
        }
        else if (history_info_.size() < predict_param_.min_fitting_lens)
        {  //当队列长度不足时不使用拟合
            fitting_disabled_ = true;
        }
        else if (target.timestamp - history_info_.front().timestamp >= predict_param_.max_time_delta)
        {  //当队列时间跨度过长时不使用拟合
            history_info_.pop_front();
            fitting_disabled_ = true;
        }
        else
        {  //其余状况下皆可以进行拟合
            fitting_disabled_ = false;
            //若队列过长，移除首元素
            if(history_info_.size() > history_deque_lens_)
            {
                history_info_.pop_front();
            }
        }

        if(debug_param_.disable_fitting)
        {
            fitting_disabled_ = true;
        }

        Eigen::Vector3d result = {0, 0, 0};
        Eigen::Vector3d result_pf = {0, 0, 0};
        Eigen::Vector3d result_fitting = {0, 0, 0};
        PredictStatus is_pf_available;
        PredictStatus is_fitting_available;
        
        //需注意粒子滤波使用相对时间（自上一次检测时所经过ms数），拟合使用自首帧所经过时间
        if(fitting_disabled_)
        {
            auto is_pf_available = predict_pf_run(target, result_pf, delta_time_estimate);
        }
        else
        {
            auto get_pf_available = std::async(std::launch::async, [=, &result_pf](){return predict_pf_run(target, result_pf, delta_time_estimate);});
            auto get_fitting_available = std::async(std::launch::async, [=, &result_fitting](){return predict_fitting_run(result_fitting, time_estimate);});

            is_pf_available = get_pf_available.get();
            is_fitting_available = get_fitting_available.get();
        }

        // if (fitting_disabled)
        // {
        //     return xyz;
        //     // auto is_pf_available = predict_pf_run(target, result_pf, delta_time_estimate);
        // }
        // else
        // {
        //     auto get_fitting_available = std::async(std::launch::async, [=, &result_fitting](){return predict_fitting_run(result_fitting, time_estimate);});

        //     is_fitting_available = get_fitting_available.get();
        // }
        
        // 进行融合
        if(is_fitting_available.xyz_status[0] && !fitting_disabled_)
        {
            result[0] = result_fitting[0];
        }
        else if(is_pf_available.xyz_status[0])
        {
            result[0] = result_pf[0];
        }
        else
        {
            result[0] = xyz[0];
        }

        if(is_fitting_available.xyz_status[1] && !fitting_disabled_)
        {
            result[1] = result_fitting[1];
        }
        else if(is_pf_available.xyz_status[1])
        {
            result[1] = result_pf[1];
        }
        else
        {
            result[1] = xyz[1];
        }

        if(is_fitting_available.xyz_status[2] && !fitting_disabled_)
        {
            result[2] = result_fitting[2];
        }
        else
        {
            result[2] = xyz[2];
        }

        result = result_pf;
        auto t2 = std::chrono::steady_clock::now();
        double dr_ms = std::chrono::duration<double,std::milli>(t2-t1).count();
        // if(timestamp % 10 == 0)
        delta_time_estimate = 0;
        // cout<<dr_ms<<endl;
        // cout<<xyz<<endl;
        // result_pf = target.xyz;
        if(debug_param_.draw_predict)
        {
            double x_offset = 400;
            double y_offset = 400;
            double z_offset = 200;
            if (cnt < 2000)
            {
                auto x = cnt * 5;
                cv::circle(pic_x,cv::Point2f((timestamp) / 10,xyz[0] * 100 + x_offset),1,cv::Scalar(0,0,255),1);
                cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[0] * 100 + x_offset),1,cv::Scalar(0,255,0),1);
                cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[0] * 100 + x_offset),1,cv::Scalar(255,255,0),1);
                // cv::circle(pic_x,cv::Point2f((timestamp + delta_time_estimate) / 10,result[0]+ 200),1,cv::Scalar(255,255,255),1);


                cv::circle(pic_y,cv::Point2f((timestamp) / 10,xyz[1] * 100 + y_offset),1,cv::Scalar(0,0,255),1);
                cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[1] * 100 + y_offset),1,cv::Scalar(0,255,0),1);
                cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[1] * 100 + y_offset),1,cv::Scalar(255,255,0),1);
                // cv::circle(pic_y,cv::Point2f((timestamp + delta_time_estimate) / 10,result[1]+ 200),1,cv::Scalar(255,255,255),1);

                cv::circle(pic_z,cv::Point2f((timestamp) / 10,xyz[2] * 100 + z_offset),1,cv::Scalar(0,0,255),1);
                cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result_pf[2] * 100  + z_offset),1,cv::Scalar(0,255,0),1);
                cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result_fitting[2] * 100 + z_offset),1,cv::Scalar(255,255,0),1);
                // cv::circle(pic_z,cv::Point2f((timestamp + delta_time_estimate) / 10,result[2]),1,cv::Scalar(255,255,255),1);
                cnt++;
            }
            cv::imshow("result_x",pic_x);
            cv::imshow("result_y",pic_y);
            cv::imshow("result_z",pic_z);
            cv::waitKey(1);
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
        // cout<<history_info.size()<<endl;
        // cout<<max_iter<<endl;
        // cout<<start_idx<<endl;
        if (max_iter == 0 || start_idx < 0)
            return history_info_.back().xyz;
        
        for (int i = 0; i < max_iter; i++)
        {
            Eigen::Vector3d sum = {0,0,0};
            for (int j = 0; j < predict_param_.window_size; j++)
                sum += history_info_.at(start_idx + i + j).xyz;
            total_sum += sum / predict_param_.window_size;
        }
        // cout<<history_info.back().xyz<<endl;
        // cout<<total_sum / max_iter<<endl;
        // cout<<endl;
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

        // cout<<result<<endl;

        return is_available;
    }

    PredictStatus ArmorPredictor::predict_fitting_run(Eigen::Vector3d& result, int time_estimated)
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
            std::cout << "T : " << target_info.timestamp / 1e3 << " X:" << target_info.xyz[0] << " Y:" << target_info.xyz[1] << std::endl;
            problem_x.AddResidualBlock (     // 向问题中添加误差项
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 2> ( 
                    new CurveFittingCost (target_info.timestamp / 1e3, (target_info.xyz[0] - params_x[0]))
                ),
                new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
                &params_x[1]                 // 待估计参数
            );
            problem_y.AddResidualBlock(     // 向问题中添加误差项 
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 2> ( 
                    new CurveFittingCost (target_info.timestamp / 1e3, (target_info.xyz[1] - params_y[0]))
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
        
        std::cout << "X:" << params_x[0] << " " << params_x[1] << " " << params_x[2] << " " << params_x[3] << std::endl; 
        std::cout << "Y:" << params_y[0] << " " << params_y[1] << " " << params_y[2] << " " << params_y[3] << std::endl;
        // cout<<summary_y.BriefReport()<<endl;
        // cout<<time_estimated<<endl;
        // cout<<bullet_speed<<endl;

        auto x_pred = params_x[0] + params_x[1] * (time_estimated / 1e3) + params_x[2] * pow((time_estimated / 1e3), 2);
        auto y_pred = params_y[0] + params_y[1] * (time_estimated / 1e3) + params_y[2] * pow((time_estimated / 1e3), 2);  
        // auto x_pred = params_x[0] + params_x[1] * cos(params_x[2] * (time_estimated / 1e3) + params_x[3]);
        // auto y_pred = params_y[0] + params_y[1] * cos(params_y[2] * (time_estimated / 1e3) + params_y[3]);
        // auto x_pred = params_x[0] + params_x[1] * cos(params_x[3] * (time_estimated / 1e3)) + params_x[2] * sin(params_x[3] * (time_estimated / 1e3));
        // auto y_pred = params_y[0] + params_y[1] * cos(params_y[3] * (time_estimated / 1e3)) + params_y[2] * sin(params_y[3] * (time_estimated / 1e3));

        std::cout << x_pred << " : " << y_pred << std::endl;
        std::cout << "..........." << std::endl;
        
        result = {x_pred, y_pred, dc[2]};
        return is_available;
    }


} // armor_processor