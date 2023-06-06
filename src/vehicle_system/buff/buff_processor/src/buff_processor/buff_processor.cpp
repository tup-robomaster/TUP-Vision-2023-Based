/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2023-06-04 02:06:58
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/test/src/buff_processor/buff_processor.cpp
 */
#include "../../include/buff_processor/buff_processor.hpp"

namespace buff_processor
{
    Processor::Processor()
    : logger_(rclcpp::get_logger("buff_processor"))
    {
        is_initialized_ = false;
    }

    Processor::Processor(const PredictorParam& predict_param, const PathParam& path_param, const DebugParam& debug_param)
    : predictor_param_(predict_param), path_param_(path_param), debug_param_(debug_param),
    logger_(rclcpp::get_logger("buff_processor"))
    {
        is_initialized_ = false;
        buff_predictor_.predictor_param_ = predict_param;
    }

    Processor::~Processor()
    {

    }

    bool Processor::predictor(BuffMsg buff_msg, BuffInfo& target_info)
    {
        int buff_mode = buff_msg.mode;
        buff_predictor_.last_mode = buff_predictor_.mode;
        buff_predictor_.mode = buff_mode;

        double theta_offset = 0.0;
        if (buff_mode == SMALL_BUFF || buff_mode == BIG_BUFF)
        {   // 进入能量机关预测模式
            Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
            Eigen::Vector3d armor_center = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
            if (!buff_predictor_.predict(buff_msg.rotate_speed, armor_center.norm(), buff_msg.timestamp, theta_offset))
            {
                Eigen::Quaterniond imu_quat = {buff_msg.quat_imu.w, buff_msg.quat_imu.x, buff_msg.quat_imu.y, buff_msg.quat_imu.z};
                rmat_imu_ = imu_quat.toRotationMatrix();
                Eigen::Vector3d armor3d_cam = {buff_msg.armor3d_cam.x, buff_msg.armor3d_cam.y, buff_msg.armor3d_cam.z};
               
                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d tracking_angle = coordsolver_.getAngle(armor3d_cam, rmat_imu_);
                target_info.angle = tracking_angle;
                target_info.target_switched = buff_msg.target_switched;
                return false;
            }
            else
            {
                // 计算击打点世界坐标
                Eigen::Vector3d hit_point_world = {sin(theta_offset) * this->predictor_param_.fan_length, (cos(theta_offset) - 1) * this->predictor_param_.fan_length, 0};
                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                Eigen::Quaterniond quat = {buff_msg.quat_world.w, buff_msg.quat_world.x, buff_msg.quat_world.y, buff_msg.quat_world.z};
                Eigen::Matrix3d rmat = quat.toRotationMatrix();
                Eigen::Quaterniond imu_quat = {buff_msg.quat_imu.w, buff_msg.quat_imu.x, buff_msg.quat_imu.y, buff_msg.quat_imu.z};
                rmat_imu_ = imu_quat.toRotationMatrix();

                hit_point_world = rmat * hit_point_world + armor3d_world;

                // 转换到相机系
                Eigen::Vector3d hit_point_cam = coordsolver_.worldToCam(hit_point_world, rmat_imu_);
                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat_imu_);
                
                target_info.angle = angle;
                target_info.armor3d_world = armor3d_world;
                target_info.hit_point_world = hit_point_world;
                target_info.hit_point_cam = hit_point_cam;
                target_info.armor3d_cam = coordsolver_.worldToCam(armor3d_world, rmat_imu_);
                target_info.target_switched = buff_msg.target_switched;
                return true;
            }
        }
        return false;
    }
} //namespace buff_processor