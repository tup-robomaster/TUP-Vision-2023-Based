/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:19:33
 * @LastEditTime: 2022-09-13 22:28:44
 * @FilePath: /tup_2023/src/vehicle_system/autoaim/include/autoaim/autoaim.hpp
 */

#include "global_user/include/global_user/global_user.hpp"

namespace autoaim
{
    class autoaim : public global_user::global_user
    {
    public:
        autoaim();
        virtual ~autoaim() = 0;
        virtual void armor_detector() = 0;
        virtual void local_decision() = 0;
        virtual void anti_gyro() = 0;
        virtual void armor_prediction() = 0;
    };
}