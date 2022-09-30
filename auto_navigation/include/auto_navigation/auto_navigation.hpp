/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:35:06
 * @LastEditTime: 2022-09-06 13:50:24
 * @FilePath: /tup_2023/src/auto_navigation/include/auto_navigation/auto_navigation.hpp
 */
#include "global_user/include/global_user/global_user.hpp"

namespace auto_navigation
{
    class auto_navigation : public global_user::global_user
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

