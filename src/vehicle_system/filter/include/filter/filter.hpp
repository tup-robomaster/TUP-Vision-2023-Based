/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 02:27:06
 * @LastEditTime: 2022-11-06 11:52:15
 * @FilePath: /filter/include/filter/filter.hpp
 */
#ifndef FILTER_HPP_
#define FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace filter
{
    template<typename T, int rows, int cols>
    using Matrix = Eigen::Matrix<T, rows, cols>;

    //向量
    template<typename T, int N>
    class Vector : public Matrix<T, N, 1>
    {
    public:
        typedef Matrix<T, N, 1> Base;

        using typename Base::Scalar;
        using Base::RowsAtCompileTime;
        using Base::ColsAtCompileTime;
        using Base::SizeAtCompileTime;

        Vector(void) : Matrix<T, N, 1>() {}

        //拷贝构造
        template<typename OtherDerived>
        Vector(const Eigen::MatrixBase<OtherDerived>& other)
        : Matrix<T, N, 1>(other)
        {}
    };

    //方形矩阵
    template<typename T, int N>
    using SquareMatrix = Matrix<T, N, N>;

    //协方差矩阵
    template<class Type>
    using Covariance = SquareMatrix<typename Type::Scalar, Type::RowsAtCompileTime>;

    //卡尔曼增益
    template<class State, class Measurement>
    using KalmanGain = Matrix<typename State::Scalar,
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
    public:
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

    //抽象基类适配标准滤波
    template<class StateType>
    class FilterBase : public Base<StateType>
    {
    public:
        using Base<StateType>::P;
    };
} //namespace filter

#endif