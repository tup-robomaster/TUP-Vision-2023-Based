/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 18:47:32
 * @LastEditTime: 2022-12-20 21:53:01
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/buff_processor/buff_processor.cpp
 */
#include "../../include/buff_processor/buff_processor.hpp"

namespace buff_processor
{
    Processor::Processor()
    {

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

    bool Processor::predictor(BuffMsg& buff_msg, int buff_mode)
    {
        buff_predictor_->mode = buff_mode;
        buff_predictor_->last_mode = buff_predictor_->mode;

        double theta_offset = 0.0;
        if(buff_predictor_->mode != -1)
        {   // 进入能量机关预测模式
            Eigen::Vector3d r_center = {buff_msg.r_center.x, buff_msg.r_center.y, buff_msg.r_center.z};
            if(!buff_predictor_->predict(buff_msg.rotate_speed, r_center.norm(), buff_msg.header.stamp.nanosec, theta_offset))
                return false;
            else
            {
                
            }
        }
    }




} //namespace buff_processor