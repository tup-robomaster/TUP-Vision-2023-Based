/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:45:06
 * @LastEditTime: 2023-03-20 19:53:21
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/test/include/buff_processor/buff_processor.hpp
 */
#ifndef BUFF_PROCESSOR_HPP_
#define BUFF_PROCESSOR_HPP_

#include "../predictor/predictor.hpp"
#include "../../../../global_user/include/coordsolver.hpp"
#include "../../../../global_user/include/global_user/global_user.hpp"
#include "global_interface/msg/buff.hpp"

//c++
#include <mutex>
#include <thread>
#include <atomic>

using namespace global_user;
using namespace coordsolver;
namespace buff_processor
{
    class Processor
    {
        typedef global_interface::msg::Buff BuffMsg;

    public:
        Processor();
        Processor(const PredictorParam& predict_param, const PathParam& path_param, const DebugParam& debug_param);
        ~Processor();

        PathParam path_param_;
        DebugParam debug_param_;
        PredictorParam predictor_param_;
    
    private:
        rclcpp::Logger logger_;

    public:
        // Mutex mutex_;
        bool is_initialized_;
        Eigen::Matrix3d rmat_imu_;
        CoordSolver coordsolver_;
        BuffPredictor buff_predictor_;
        
        bool fittingThread(BuffMsg buff_msg);
        bool predictorThread(BuffMsg buff_msg, Eigen::Matrix3d rmat_gimbal, Eigen::Vector3d translation, TargetInfo& target_info);
        bool predictor(BuffMsg buff_msg, TargetInfo& target_info);
    };
} //namespace buff_processor

#endif