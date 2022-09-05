/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:59:06
 * @LastEditTime: 2022-09-06 02:01:20
 * @FilePath: /tup_2023/src/serialport/include/serialport/serialport.hpp
 */
#include "global_user/include/global_user/global_user.hpp"

namespace serialport 
{
    class serialport : public global_user::global_user
    {
    public:
        serialport();
        virtual ~serialport() = 0;
    };
};