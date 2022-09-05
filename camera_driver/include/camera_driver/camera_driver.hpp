/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 00:29:49
 * @LastEditTime: 2022-09-06 00:33:41
 * @FilePath: /tup_2023/src/camera_driver/include/camera_driver/camera_driver.hpp
 */
#include "params_common/include/params_common/params_common.hpp"

namespace camera_driver
{
    class camera_driver : public robot_base::params_common
    {
    public:
        camera_driver();
        virtual ~camera_driver() = 0;
    };
};