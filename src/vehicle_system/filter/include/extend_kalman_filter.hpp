/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 15:51:26
 * @LastEditTime: 2022-11-14 21:51:40
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/extend_kalman_filter.hpp
 */
#ifndef EXTEND_KALMAN_FILTER_HPP_
#define EXTEND_KALMAN_FILTER_HPP_

#include "./motion_model/kalman_filter_base.hpp"
#include "./motion_model/linear_system_model.hpp"
#include "./motion_model/linear_measurement_model.hpp"

namespace filter
{
    template<class StateType>
    class ExtendKalmanFilter : public KalmanFilterBase<StateType>, public FilterBase<StateType>
    {
    public:
        typedef KalmanFilterBase<StateType> KalmanBase;
        typedef FilterBase<StateType> Filter_Base;
        
        //从基类继承的元素类型
        using typename KalmanBase::T;

        //从基类继承的状态矩阵类型
        using typename KalmanBase::State;

        template<class Measurement, template<class> class CovarianceBase>
        using MeasurementModelType = LinearMeasurementModel<State, Measurement, CovarianceBase>;

        template<class Control, template<class> class CovarianceBase>
        using SystemModelType = LinearSystemModel<State, Control, CovarianceBase>;

    protected:
        //卡尔曼增益矩阵类型
        template<class Measurement>
        using KalmanGain = KalmanGain<State, Measurement>;

    public:
        //状态估计量
        using KalmanBase::x;

        //状态协方差矩阵
        using Filter_Base::P;
    
    public:
        Covariance<StateType> S_;
        double likelihood_;
    
    public:
        ExtendKalmanFilter()
        {
            //初始化协方差矩阵
            P.setIdentity();
        }

        ~ExtendKalmanFilter(){}

        //无控制输入的线性模型预测
        template<class Control, template<class> class CovarianceBase>
        const State& predict(SystemModelType<Control, CovarianceBase>& s, const double& dt)
        {
            //预测
            Control u;
            u.setZero();
            return predict(s, u, dt);
        }

        //含控制输入的线性模型预测
        template<class Control, template<class> class CovarianceBase>
        const State& predict(SystemModelType<Control, CovarianceBase>& s, const Control& u, const float& t)
        {
            s.updateJacobians(x, u, t);

            //状态预测
            x = s.f(x, u, t);

            //协方差预测
            // P = (s.F * P * s.F.transpose()) +  s.W(); 
            P = (s.F * P * s.F.transpose()) +  (s.W * s.getCovariance() * s.W.transpose());

            return this->getState();
        }

        //更新
        template<class Measurement, template<class> class CovarianceBase>
        const State& update(MeasurementModelType<Measurement, CovarianceBase>& m, const Measurement& z)
        {
            m.updateJacobians(x);

            //测量值与预测值之间的残差
            auto v = z - m.H * this->getState();
            // std::cout << v.size() << std::endl;
            
            //计算卡尔曼增益
            //残差的协方差矩阵
            // Covariance<Measurement> S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
            Covariance<Measurement> S = (m.H * P * m.H.transpose()) + (m.V * m.getCovariance() * m.V.transpose());
            KalmanGain<Measurement> K = P * (m.H.transpose() * S.inverse());

            //假定模型残差符合高斯分布，计算似然值
            // double det = S.determinant();
            // this->S_ = S;
            // S = S.inverse();
            // this->likelihood_ = (1.0 / sqrt(2 * M_PI * fabs(det))) * exp(-0.5 * v.transpose() * S * v);

            //更新状态矩阵和协方差矩阵
            x += (K * (z - m.h(x)));
            P -= (K * m.H * P);

            return this->getState();
        }
    };

} //namespace filter

#endif