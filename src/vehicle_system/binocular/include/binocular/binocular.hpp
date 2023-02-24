/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-12 01:26:01
 * @LastEditTime: 2022-09-12 01:36:53
 * @FilePath: /tup_2023/src/vehicle_system/binocular/include/binocular/binocular.hpp
 */
#include "global_user/include/global_user/global_user.hpp"

namespace binocular
{
    class binocular
    {
        binocular(){}
        ~binocular(){}
        /**
         * @brief 目前来看双目测距没必要
         *  1.单目标定
         *  2.双目标定
         *  3.双目校正（Bouguet法）
         *  4.双目匹配
         *  5.双目测距
        */
        cv::Point3f get_position(cv::Point2f point_left, cv::Point2f point_right, bool direction){}
        std::pair<cv::Point3f, cv::Point3d> get_position_angle(std::vector<cv::Point2f> left_points, std::vector<cv::Point2f> right_points){}
    };
}; //binocular