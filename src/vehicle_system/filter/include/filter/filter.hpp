/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 02:27:06
 * @LastEditTime: 2022-11-03 00:07:00
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/filter/filter.hpp
 */
#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <Eigen/Dense>

namespace filter
{
    template<typename T, int rows, int cols>
    using Matrix = Eigen::Matrix<T, rows, cols>;

    //向量
    template<typename T, int N>
    using Vector = Matrix<T, N, 1>;

    //方形矩阵
    template<typename T, int N>
    using SquareMatrix = Matrix<T, N, N>;

    //协方差矩阵
    template<class Type>
    using Covariance = SquareMatrix<typename Type::Scaler, Type::RowsAtCompileTime>;

    //卡尔曼增益
    template<class State, class Measurement>
    using KalmanGain = Matrix<typename State::Scaler,
                                State::RowsAtCompileTime,
                                Measurement::RowsAtCompileTime>;
    
    //雅可比矩阵
    template<class A, class B>
    using Jacobian = Matrix<typename A::Scalar,
                            A::RowsAtCompileTime,
                            B::RowsAtCompileTime>;

    template<class StateType>
    class Base
    {
        //协方差
        Covariance<StateType> P;
    
    public:
        const Covariance<StateType>& getCovariance() const
        {
            return P;
        }

        bool setCovariance(const Covariance<StateType>& covariance)
        {
            P = covariance;
            return true;
        }

    protected:
        Base()
        {
            P.setIdentity();
        }
    };

} //namespace filter

#endif