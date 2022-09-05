/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:19:33
 * @LastEditTime: 2022-09-05 17:05:19
 * @FilePath: /tup_2023/src/vehicle_system/autoaim/include/autoaim/autoaim.hpp
 */
#include "params_common/include/params_common/params_common.hpp"

namespace autoaim
{
    class autoaim : robot_base::params_common
    {
    public:
        autoaim();
        virtual ~autoaim() = 0;
        virtual void armor_detector() = 0;
        virtual void local_decision() = 0;
        virtual void anti_gyro() = 0;
        virtual void prediction() = 0;
    };
}