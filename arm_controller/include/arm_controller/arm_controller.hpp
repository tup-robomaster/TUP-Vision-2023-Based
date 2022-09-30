/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:18:42
 * @LastEditTime: 2022-09-06 13:51:01
 * @FilePath: /tup_2023/src/arm_controller/include/arm_controller/arm_controller.hpp
 */
#include "global_user/include/global_user/global_user.hpp"

namespace arm_controller
{
    class arm_controller : public global_user::global_user
    {
    public:
        arm_controller();
        virtual ~arm_controller() = 0;

        virtual bool detect_ore() = 0;
        virtual bool trajectory_planning() = 0;
        virtual bool pose_adjust() = 0;
    };
};
