/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2023-06-04 00:03:36
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
        
    bool Processor::predict(BuffMsg buff_msg, BuffInfo& buff_info)
    {
        buff_predictor_.last_mode_ = buff_predictor_.mode_;
        int mode = buff_msg.mode;
        buff_predictor_.mode_ = mode;
        
        if (mode == SMALL_BUFF || mode == BIG_BUFF)
        {   // 进入能量机关预测模式
            double theta_offset = 0.0;
            Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
            Eigen::Vector3d armor_center = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
            if (!buff_predictor_.predict(buff_msg, buff_info, theta_offset))
            {
                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                Eigen::Quaterniond imu_quat = {buff_msg.quat_imu.w, buff_msg.quat_imu.x, buff_msg.quat_imu.y, buff_msg.quat_imu.z};
                rmat_imu_ = imu_quat.toRotationMatrix();
                
                // 转换到相机系
                Eigen::Vector3d hit_point_cam = coordsolver_.worldToCam(armor3d_world, rmat_imu_);

                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat_imu_);
                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "Yaw: %lf Pitch: %lf", 
                    angle(0), angle(1)
                );

                buff_info.angle = angle;
                buff_info.armor3d_world = armor3d_world;
                buff_info.hit_point_world = armor3d_world;
                buff_info.hit_point_cam = hit_point_cam;
                buff_info.armor3d_cam = hit_point_cam;
                buff_info.target_switched = buff_msg.target_switched;

                RCLCPP_WARN(
                    logger_, 
                    "Predictor failed..."
                );
                return true;
            }
            else
            {
                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "Angle offset: %.4f", 
                    theta_offset
                );
                
                // 计算击打点世界坐标
                Eigen::Vector3d hit_point_world = {sin(theta_offset) * this->predictor_param_.fan_length, (cos(theta_offset) - 1) * this->predictor_param_.fan_length, 0};
                // RCLCPP_INFO(logger_, "hit_point_world: %lf %lf %lf", hit_point_world[0], hit_point_world[1], hit_point_world[2]);

                Eigen::Vector3d armor3d_world = {buff_msg.armor3d_world.x, buff_msg.armor3d_world.y, buff_msg.armor3d_world.z};
                Eigen::Vector3d center3d_world = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
                Eigen::Quaterniond quat_world = Eigen::Quaterniond::Identity();
                quat_world.w() = buff_msg.quat_world.w; 
                quat_world.x() = buff_msg.quat_world.x; 
                quat_world.y() = buff_msg.quat_world.y;
                quat_world.z() = buff_msg.quat_world.z;
                Eigen::Matrix3d rmat = quat_world.toRotationMatrix();
                Eigen::Quaterniond imu_quat = {buff_msg.quat_imu.w, buff_msg.quat_imu.x, buff_msg.quat_imu.y, buff_msg.quat_imu.z};
                rmat_imu_ = imu_quat.toRotationMatrix();

                hit_point_world = rmat * hit_point_world + armor3d_world;
                
                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "armor3d_world: (%lf, %lf, %lf) hit_point_world: (%lf, %lf, %lf)", 
                    armor3d_world(0), armor3d_world(1), armor3d_world(2),
                    hit_point_world(0), hit_point_world(1), hit_point_world(2)
                );

                // 转换到相机系
                Eigen::Vector3d hit_point_cam = coordsolver_.worldToCam(hit_point_world, rmat_imu_);
                // 计算云台偏转角度（pitch、yaw）
                Eigen::Vector2d angle = coordsolver_.getAngle(hit_point_cam, rmat_imu_);
                
                RCLCPP_INFO_THROTTLE(
                    logger_, 
                    steady_clock_,
                    50,
                    "Yaw: %lf Pitch: %lf", 
                    angle(0), angle(1)
                );

                buff_info.angle = angle;
                buff_info.armor3d_world = armor3d_world;
                buff_info.hit_point_world = hit_point_world;
                buff_info.hit_point_cam = hit_point_cam;
                buff_info.armor3d_cam = coordsolver_.worldToCam(armor3d_world, rmat_imu_);
                
                // buff_info.abs_meas_angle = abs_meas_angle * (180 / CV_PI);
                // buff_info.abs_pred_angle = abs_pred_angle * (180 / CV_PI);
                // cout << "buff_info.abs_angle:" << abs_pred_angle << endl;

                buff_info.target_switched = buff_msg.target_switched;
                return true;
            }
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                logger_, 
                steady_clock_, 
                50, 
                "Not buff mode... Current mode: %d", 
                buff_msg.mode
            );
            return false;
        }
    }
} //namespace buff_processor