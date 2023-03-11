/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-09 22:41:12
 * @LastEditTime: 2023-03-09 22:43:57
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/prediction/curve_fitting.hpp
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

namespace armor_processor
{
    struct CurveFittingCost
    {
        const bool _axis;
        const double _x, _y, _t, _coeff;

        CurveFittingCost(bool axis, double x, double y, double t, double coeff)
        : _axis(axis), _x (x), _y(y), _t(t), _coeff(coeff) {}

        //计算残差
        template<class T>
        bool operator()(
            const T* const params, // 模型参数，有3维
            T* residual) const     // 残差
        {
            // residual[0] = T (_x) - params[0] * T(_t); // f(x) = a0 + a1 * t + a2 * t^2 
            residual[0] = T (_x) - params[0] * T(_t) - params[1] * T(_t) * T(_t); // f(x) = a0 + a1 * t + a2 * t^2 
            // residual[0] = T (_x) - params[0] * ceres::cos(params[1] * T (_t) + params[2]); // f(x) = a0 + a1 * cos(wt + THETA)
            // residual[0] = T (_x) - params[0] * ceres::cos(params[2] * T (_t)) - params[1] * ceres::sin(params[2] * T (_t)); // f(x) = a0 + a1 * cos(wt) + b1 * sin(wt) 
            return true;
        }

        // 前哨站旋转装甲板轨迹拟合
        template<class T>
        bool operator()
        (
            const T* const x0,
            const T* const y0,
            const T* const theta,
            T* residual
        ) const
        {
            if(!_axis)
            {   // x轴
                residual[0] = x0[0] + 0.2765 * ceres::cos(0.8 * M_PI * _coeff * _t + theta[0]) - _x;
            }
            else
            {   // y轴
                residual[0] = y0[0] + 0.2765 * ceres::sin(0.8 * M_PI * _coeff * _t + theta[0]) - _y;
            }
            return true;
        }

        //小陀螺+左右横移运动轨迹拟合（反陀螺）
        template <class T>
        bool operator()
        (
            const T* const w,
            const T* const theta,
            const T* const V,
            const T* const x0,
            const T* const y0,
            const T* const a,
            const T* const b,
            const T* const phi,
            T* residual
        ) const
        {
            if(!_axis)
            {   //x轴
                // residual[0] = x0[0] + a[0] * ceres::cos(w[0] * T(_t)) + V[0] * T(_t) * ceres::cos(theta[0]) * _coeff - _x;
                residual[0] = x0[0] + a[0] * ceres::cos(w[0] * T(_t) + phi[0]) + V[0] * T(_t) * ceres::cos(theta[0]) * _coeff - _x;
            }
            else
            {   //y轴
                // residual[0] = y0[0] + b[0] * ceres::sin(w[0] * T(_t)) + V[0] * T(_t) * ceres::sin(theta[0]) * _coeff - _y;
                residual[0] = y0[0] + b[0] * ceres::sin(w[0] * T(_t) + phi[0]) + V[0] * T(_t) * ceres::sin(theta[0]) * _coeff - _y;
            }
            return true;
        }

        template <class T>
        bool operator()
        (
            const T* const k,
            const T* const d,
            const T* const a,
            const T* const b,
            const T* const c,
            T* residual
        ) const
        {
            if(!_axis)
            {   //x轴 
                // residual[0] = k[0] * T(_t) + _coeff - _x; //f(t)=kt+x0
                residual[0] = k[0] * T(_t) + d[0] - _x; //f(t)=kt+d
            }  
            else
            {   //f(t)=a*(k^2)*(t^2)+(2kad+kb)*t+a(d^2)+bd+c
                // residual[0] = a[0] * pow(k[0], 2) * pow(T(_t), 2) 
                //             + ((2.0 * (k[0] * a[0] * d[0])) + (k[0] * b[0])) * T(_t) 
                //             + a[0] * pow(d[0], 2) + b[0] * d[0] + c[0] - _y; 

                //f(t)=a*(t^2)+b*t+c
                // residual[0] = a[0] * pow(T(_t), 2) + b[0] * T(_t) + c[0] - _y;

                //f(t)=(a/t) + b
                // residual[0] = b[0] / T(_t) + c[0] - _y;

                //f(t)=(1/t) + b
                residual[0] = (1.0 / T(_t)) + c[0] - _y;
            }

            return true;
        }
    };
} //namespace armor_processor
#endif