/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-09 22:41:12
 * @LastEditTime: 2023-04-07 15:34:26
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/param_struct/curve_fitting.hpp
 */
#ifndef CURVE_FITTING_HPP_
#define CURVE_FITTING_HPP_

//c++
#include <iostream>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//ceres
#include <ceres/ceres.h>

//opencv 
#include <opencv2/opencv.hpp>

namespace armor_detector
{
    struct CurveFittingCost
    {
        const Eigen::Vector3d _p1, _p2;
        const double _theta;

        CurveFittingCost(Eigen::Vector3d p1, Eigen::Vector3d p2, double theta)
        : _p1(p1), _p2(p2), _theta(theta) {}

        //计算残差
        template<class T>
        bool operator()(
            const T* const circle, // 圆心坐标，2维
            T* residual) const     // 残差
        {
            residual[0] = T(_p1[1]) + T(_p2[1]) / 2.0 - ((T(_p2[0]) - T(_p1[0])) / (2 * ceres::tan(0.5 * T(_theta)))) - circle[0];
            residual[1] = T(_p1[0]) + T(_p2[0]) / 2.0 + ((T(_p2[1]) - T(_p1[1])) / (2 * ceres::tan(0.5 * T(_theta)))) - circle[1];
            return true;
        }
    };
} //namespace armor_detector
#endif