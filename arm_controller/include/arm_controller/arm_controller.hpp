/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:18:42
 * @LastEditTime: 2022-09-06 01:33:47
 * @FilePath: /tup_2023/src/arm_controller/include/arm_controller/arm_controller.hpp
 */
#include "params_common/include/params_common/params_common.hpp"

namespace arm_controller
{
    class arm_controller : public robot_base::params_common
    {
    public:
        arm_controller();
        virtual ~arm_controller() = 0;

        virtual bool detect_ore() = 0;
        virtual bool trajectory_planning() = 0;
        virtual bool pose_adjust() = 0;
    };
};
