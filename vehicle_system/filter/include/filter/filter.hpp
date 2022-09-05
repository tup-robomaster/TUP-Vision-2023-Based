/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 02:27:06
 * @LastEditTime: 2022-09-06 02:56:47
 * @FilePath: /tup_2023/src/vehicle_system/filter/include/filter/filter.hpp
 */
#include "global_user/include/global_user/global_user.hpp"

namespace filter
{
    class filter : public global_user::global_user
    {
    public:
        filter();
        virtual ~filter() = 0;

        virtual bool init_param() = 0;
        virtual Eigen::VectorXd predict() = 0;
        virtual bool update() = 0;
    };
} // namespace filter


