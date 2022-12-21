/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2022-12-21 17:26:25
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/buff_processor/buff_processor.cpp
 */
#include "../../include/buff_processor/buff_processor.hpp"

namespace buff_processor
{
    Processor::Processor()
    {
        buff_predictor_->is_params_confirmed = false;
        buff_predictor_->last_mode = buff_predictor_->mode = -1;
        buff_predictor_->history_info.clear();
    }

    Processor::Processor(const PredictorParam& predict_param, const PathParam& path_param, const DebugParam& debug_param)
    : predictor_param_(predict_param), path_param_(path_param), debug_param_(debug_param)
    {
        buff_predictor_->is_params_confirmed = false;
        buff_predictor_->last_mode = buff_predictor_->mode = -1;
        buff_predictor_->history_info.clear();
    }

    Processor::~Processor()
    {

    }

    bool Processor::predictor(BuffMsg& buff_msg, TargetInfo& target_info)
    {
        buff_predictor_->mode = target_info.buff_mode;
        buff_predictor_->last_mode = buff_predictor_->mode;

        double theta_offset = 0.0;
        if(buff_predictor_->mode != -1)
        {   // 进入能量机关预测模式
            Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
            if(!buff_predictor_->predict(buff_msg.rotate_speed, r_center.norm(), buff_msg.header.stamp.nanosec, theta_offset))
                return false;
            else
            {
                // 计算击打点世界坐标
                Eigen::Vector3d hit_point_world = {sin(theta_offset) * this->predictor_param_.fan_length, (cos(theta_offset) - 1) * this->predictor_param_.fan_length, 0};
                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                Eigen::Matrix3d rmat << buff_msg.rmat.x, buff_msg.rmat.y, buff_msg.rmat.z;
                hit_point_world = rmat * hit_point_world + armor3d_world;

                // 转换到相机系
                Eigen::Vector3d hit_point_cam = coordsolver_->worldToCam(hit_point_world, target_info.rmat_imu);
                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_->getAngle(hit_point_cam, target_info.rmat_imu);

                target_info.angle = angle;
                target_info.armor3d_world = armor3d_world;
                target_info.hit_point_world = hit_point_world;
                target_info.hit_point_cam = hit_point_cam;
                target_info.armor3d_cam = coordsolver_->worldToCam(armor3d_world, target_info.rmat_imu);
                target_info.target_switched = buff_msg.target_switched;
                return true;
            }
        }
    }




} //namespace buff_processor