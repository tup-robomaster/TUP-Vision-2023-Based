/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2023-05-18 19:08:25
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/buff_processor/buff_processor.cpp
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

    bool Processor::predict(BuffMsg buff_msg, Eigen::Matrix3d rmat_gimbal, Eigen::Vector3d translation, TargetInfo& pred_info)
    {
        int mode = buff_msg.mode;
        buff_predictor_.mode_ = mode;
        
        if (mode == 3 || mode == 4)
        {   // 进入能量机关预测模式
            double theta_offset = 0.0;
            Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
            Eigen::Vector3d armor_center = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
            if (!buff_predictor_.predict(buff_msg, armor_center.norm(), theta_offset))
            {
                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                
                // 转换到相机系
                Eigen::Vector3d hit_point_cam = rmat_gimbal.transpose() * (armor3d_world - translation);

                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat_gimbal, translation);
                RCLCPP_INFO(
                    logger_, 
                    "Yaw: %.3f Pitch: %.3f", 
                    angle[0], angle[1]
                );

                pred_info.angle = angle;
                pred_info.armor3d_world = armor3d_world;
                pred_info.hit_point_world = armor3d_world;
                pred_info.hit_point_cam = hit_point_cam;
                pred_info.armor3d_cam = hit_point_cam;
                pred_info.target_switched = buff_msg.target_switched;

                RCLCPP_WARN(logger_, "Predictor failed...");
                return true;
            }
            else
            {
                RCLCPP_INFO(logger_, "Angle offset: %lf", theta_offset);
                
                // 计算击打点世界坐标
                Eigen::Vector3d hit_point_world = {-sin(theta_offset) * this->predictor_param_.fan_length, (cos(theta_offset) - 1) * this->predictor_param_.fan_length, 0};
                // RCLCPP_INFO(logger_, "hit_point_world: %lf %lf %lf", hit_point_world[0], hit_point_world[1], hit_point_world[2]);

                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                Eigen::Quaterniond quat = {buff_msg.quat_cam.w, buff_msg.quat_cam.x, buff_msg.quat_cam.y, buff_msg.quat_cam.z};
                Eigen::Matrix3d rmat = quat.toRotationMatrix();
                hit_point_world = rmat * hit_point_world + armor3d_world;

                // RCLCPP_INFO(logger_, "hit_point_world: %lf %lf %lf", (rmat * hit_point_world)[0], (rmat * hit_point_world)[1], (rmat * hit_point_world)[2]);
                // RCLCPP_INFO(logger_, "armor3d_world: %lf %lf %lf", armor3d_world[0], armor3d_world[1], armor3d_world[2]);
                // RCLCPP_INFO(logger_, "hit_point: %lf %lf %lf", hit_point_world[0], hit_point_world[1], hit_point_world[2]);

                // 转换到相机系
                Eigen::Vector3d hit_point_cam = rmat_gimbal.transpose() * (hit_point_world - translation);
                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat_gimbal, translation);
                RCLCPP_INFO(logger_, "Yaw: %lf Pitch: %lf", angle[0], angle[1]);

                pred_info.angle = angle;
                pred_info.armor3d_world = armor3d_world;
                pred_info.hit_point_world = hit_point_world;
                pred_info.hit_point_cam = hit_point_cam;
                pred_info.armor3d_cam = rmat_gimbal * armor3d_world + translation; 
                pred_info.target_switched = buff_msg.target_switched;
                return true;
            }
        }
        else
        {
            RCLCPP_INFO_THROTTLE(logger_, buff_predictor_.steady_clock_, 2000, "Not buff mode... Current mode: %d", buff_msg.mode);
            return false;
        }
    }
} //namespace buff_processor