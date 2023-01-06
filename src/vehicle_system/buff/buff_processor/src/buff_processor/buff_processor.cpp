/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2023-01-07 02:17:56
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/buff_processor/buff_processor.cpp
 */
#include "../../include/buff_processor/buff_processor.hpp"

namespace buff_processor
{
    Processor::Processor()
    : logger_(rclcpp::get_logger("buff_processor"))
    {
        is_initialized = false;
    }

    Processor::Processor(const PredictorParam& predict_param, const PathParam& path_param, const DebugParam& debug_param)
    : predictor_param_(predict_param), path_param_(path_param), debug_param_(debug_param),
    logger_(rclcpp::get_logger("buff_processor"))
    {
        is_initialized = false;
        buff_predictor_.predictor_param_ = predict_param;
    }

    Processor::~Processor()
    {

    }

    bool Processor::predictor(BuffMsg buff_msg, TargetInfo& target_info)
    {
        // if(!is_initialized)
        // {
        //     coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
        //     is_initialized = true;
        // }
        // std::cout << 5 << std::endl;
        if(!debug_param_.using_imu)
            buff_predictor_.mode = 1;

        buff_predictor_.mode = target_info.buff_mode;
        buff_predictor_.last_mode = buff_predictor_.mode;

        double theta_offset = 0.0;
        if(buff_predictor_.mode != -1)
        {   // 进入能量机关预测模式
            // std::cout << 6 << std::endl;
            if(buff_predictor_.mode == 3)
                buff_predictor_.last_mode = buff_predictor_.mode = 0;
            if(buff_predictor_.mode == 4)
                buff_predictor_.last_mode = buff_predictor_.mode = 1;

            Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
            if(!buff_predictor_.predict(buff_msg.rotate_speed, r_center.norm(), buff_msg.header.stamp.nanosec, theta_offset))
                return false;
            else
            {
                // 计算击打点世界坐标
                Eigen::Vector3d hit_point_world = {sin(theta_offset) * this->predictor_param_.fan_length, (cos(theta_offset) - 1) * this->predictor_param_.fan_length, 0};
                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                Eigen::Matrix3d rmat;
                if(debug_param_.using_imu)
                {
                    rmat << buff_msg.rmat.x, 
                            buff_msg.rmat.y, 
                            buff_msg.rmat.z;
                }
                else
                    rmat = Eigen::Matrix3d::Identity();
                std::cout << 6 << std::endl;

                hit_point_world = rmat * hit_point_world + armor3d_world;

                // 转换到相机系
                Eigen::Vector3d hit_point_cam = coordsolver_.worldToCam(hit_point_world, rmat);
                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat);
                RCLCPP_INFO(logger_, "Yaw: %lf Pitch: %lf", angle[0], angle[1]);

                target_info.angle = angle;
                target_info.armor3d_world = armor3d_world;
                target_info.hit_point_world = hit_point_world;
                target_info.hit_point_cam = hit_point_cam;
                target_info.armor3d_cam = coordsolver_.worldToCam(armor3d_world, target_info.rmat_imu);
                target_info.target_switched = buff_msg.target_switched;
                return true;
            }
        }

        return false;
    }

    void Processor::setPredictorParam(double param, int idx)
    {
        switch (idx)
        {
        case 1:
            predictor_param_.bullet_speed = param;
            break;
        case 2:
            predictor_param_.delay_big = param;
            break;
        case 3:
            predictor_param_.delay_small = param;
            break;
        case 4:
            predictor_param_.history_deque_len_cos = param;
            break;
        case 5:
            predictor_param_.history_deque_len_phase = param;
            break;
        case 6:
            predictor_param_.history_deque_len_uniform = param;
            break;
        case 7:
            predictor_param_.max_a = param;
            break;
        case 8:
            predictor_param_.max_rmse = param;
            break;
        case 9:
            predictor_param_.max_timespan = param;
            break;
        case 10:
            predictor_param_.max_v = param;
            break;
        case 11:
            predictor_param_.window_size = param;
            break;
        default:
            break;
        }
    }

    void Processor::setDebugParam(double param, int idx)
    {
        switch (idx)
        {
        case 1:
            debug_param_.show_predict = param;
            break;
        case 2:
            debug_param_.using_imu = param;
            break;
        default:
            break;
        }
    }

} //namespace buff_processor