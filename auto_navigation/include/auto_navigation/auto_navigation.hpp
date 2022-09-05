/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:35:06
 * @LastEditTime: 2022-09-06 01:40:16
 * @FilePath: /tup_2023/src/auto_navigation/include/auto_navigation/auto_navigation.hpp
 */
#include "params_common/include/params_common/params_common.hpp"

namespace auto_navigation
{
    class auto_navigation : public robot_base::params_common
    {
    public:
        auto_navigation();
        virtual ~auto_navigation() = 0;

        virtual bool build_map() = 0;
        virtual bool build_costmap() = 0;
        virtual bool path_planning() = 0;
        virtual bool motion_controller() = 0;
        virtual bool motion_recovery() = 0;
    };
};

