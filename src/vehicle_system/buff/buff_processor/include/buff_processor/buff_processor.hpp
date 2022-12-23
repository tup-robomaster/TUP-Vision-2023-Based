/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:45:06
 * @LastEditTime: 2022-12-23 20:03:19
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/include/buff_processor/buff_processor.hpp
 */
#ifndef BUFF_PROCESSOR_HPP_
#define BUFF_PROCESSOR_HPP_

#include "../predictor/predictor.hpp"
#include "../../../../global_user/include/coordsolver.hpp"
#include "../../../../global_user/include/global_user/global_user.hpp"
#include "global_interface/msg/buff.hpp"

using namespace global_user;
using namespace coordsolver;

namespace buff_processor
{
    struct TargetInfo
    {   
        Eigen::Vector3d armor3d_world;
        Eigen::Vector3d hit_point_world;

        Eigen::Vector3d armor3d_cam;
        Eigen::Vector3d hit_point_cam;

        Eigen::Vector2d angle;
        Eigen::Matrix3d rmat_imu;

        bool target_switched;
        int buff_mode;
    };

    class Processor
    {
        typedef global_interface::msg::Buff BuffMsg;

    public:
        Processor();
        Processor(const PredictorParam& predict_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Processor();

    private:
        PathParam path_param_;
        DebugParam debug_param_;
        PredictorParam predictor_param_;

    public:
        bool is_initialized;
        Eigen::Matrix3d rmat_imu_;
        std::unique_ptr<CoordSolver> coordsolver_;
        std::unique_ptr<BuffPredictor> buff_predictor_;
        
        bool predictor(BuffMsg buff_msg, TargetInfo& target_info);

        void setPredictorParam(double param, int idx);
        void setDebugParam(double param, int idx);
    };
} //namespace buff_processor

#endif