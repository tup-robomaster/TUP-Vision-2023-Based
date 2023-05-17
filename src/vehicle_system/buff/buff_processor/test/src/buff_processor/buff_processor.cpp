/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2023-03-20 16:21:59
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

    bool Processor::fittingThread(BuffMsg buff_msg)
    {
        // mutex_.lock();
        buff_msg.mode = 4;
        int local_mode = buff_predictor_.mode;
        buff_predictor_.last_mode = local_mode;
        if(buff_msg.mode == 3)
            buff_predictor_.mode = 0;
        else if(buff_msg.mode == 4)
            buff_predictor_.mode = 1;
        // mutex_.unlock();

        if(buff_msg.mode == 3 || buff_msg.mode == 4)
        {
            if(buff_predictor_.curveFitting(buff_msg))
            {
                RCLCPP_INFO_THROTTLE(logger_, buff_predictor_.steady_clock_, 2000, "Curve fitting succeed...");
                return true;
            }
            else
            {
                RCLCPP_WARN(logger_, "Curve fitting failed...");
                return false;
            }
        }
        else
        {
            RCLCPP_INFO_THROTTLE(logger_, buff_predictor_.steady_clock_, 2000, "Not buff mode... Current mode: %d", buff_msg.mode);
            return false;
        }
    }
        
    bool Processor::predictorThread(BuffMsg buff_msg, Eigen::Matrix3d rmat_gimbal, Eigen::Vector3d translation, TargetInfo& pred_info)
    {
        buff_msg.mode = 4;
        // mutex_.lock();
        int custom_mode = buff_predictor_.mode;
        buff_predictor_.last_mode = custom_mode;
        if (buff_msg.mode == 3)
            buff_predictor_.mode = 0;
        else if (buff_msg.mode == 4)
            buff_predictor_.mode = 1;
        // mutex_.unlock();
        
        if (buff_msg.mode == 3 || buff_msg.mode == 4)
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

    // bool Processor::predictor(BuffMsg buff_msg, TargetInfo& pred_info)
    // {
    //     // if(!is_initialized)
    //     // {
    //     //     coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
    //     //     is_initialized = true;
    //     // }
    //     buff_predictor_.mode = buff_msg.mode = 1;
    //     int local_mode = buff_predictor_.mode;
    //     buff_predictor_.last_mode = local_mode;

    //     double theta_offset = 0.0;
    //     if(buff_predictor_.mode != -1)
    //     {   // 进入能量机关预测模式
    //         // std::cout << 6 << std::endl;
    //         if(buff_predictor_.mode == 3)
    //             buff_predictor_.mode = 0;
    //         else if(buff_predictor_.mode == 4)
    //             buff_predictor_.mode = 1;

    //         if(buff_predictor_.last_mode == 3)
    //             buff_predictor_.mode = 0;
    //         else if(buff_predictor_.last_mode == 4)
    //             buff_predictor_.mode = 1;

    //         Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
    //         Eigen::Vector3d armor_center = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
    //         if(!buff_predictor_.predict(buff_msg, armor_center.norm(), theta_offset))
    //             return false;
    //         else
    //         {
    //             // 计算击打点世界坐标
    //             Eigen::Vector3d hit_point_world = {sin(theta_offset) * this->predictor_param_.fan_length, (cos(theta_offset) - 1) * this->predictor_param_.fan_length, 0};
    //             Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
    //             Eigen::Quaterniond quat = {buff_msg.quat_cam.w, buff_msg.quat_cam.x, buff_msg.quat_cam.y, buff_msg.quat_cam.z};
    //             Eigen::Matrix3d rmat = quat.toRotationMatrix();
    //             if(debug_param_.using_imu)
    //             {
    //                 Eigen::Quaterniond imu_quat = {buff_msg.quat_imu.w, buff_msg.quat_imu.x, buff_msg.quat_imu.y, buff_msg.quat_imu.z};
    //                 rmat_imu_ = imu_quat.toRotationMatrix();
    //             }
    //             else
    //                 rmat_imu_ = Eigen::Matrix3d::Identity();

    //             hit_point_world = rmat * hit_point_world + armor3d_world;

    //             // 转换到相机系
    //             Eigen::Vector3d hit_point_cam = coordsolver_.worldToCam(hit_point_world, rmat_imu_);
    //             // 计算云台偏转角度（pitch、yaw）
    //             Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat_imu_);
    //             RCLCPP_INFO(logger_, "Yaw: %lf Pitch: %lf", angle[0], angle[1]);

    //             pred_info.angle = angle;
    //             pred_info.armor3d_world = armor3d_world;
    //             pred_info.hit_point_world = hit_point_world;
    //             pred_info.hit_point_cam = hit_point_cam;
    //             pred_info.armor3d_cam = coordsolver_.worldToCam(armor3d_world, rmat_imu_);
    //             pred_info.target_switched = buff_msg.target_switched;
    //             return true;
    //         }
    //     }
    //     return false;
    // }
} //namespace buff_processor