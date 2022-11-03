/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 12:27:21
 * @LastEditTime: 2022-11-03 16:37:09
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/motion_model/linear_measurement_model.hpp
 */
#ifndef LINEAR_SYSTEM_MODEL_HPP_
#define LINEAR_SYSTEM_MODEL_HPP_

#include "./measurement_model.hpp"

namespace filter
{
    //抽象观测模型
    template<calss StateType, class MeasurementType, template<class> class CovarianceBase = Base>
    class LinearMeasurementModel : public MeasurementModel<StateType, MeasurementType, CovarianceBase>
    {
    public:
        typedef MeasurementModel<StateType, MeasurementType, CovarianceBase> MeasurementModelBase;
        using typename MeasurementModelBase::State;
        using typename MeasurementModelBase::Measurement; 

    public:
        //观测模型雅可比矩阵
        Jacobian<Measurement, State> H;
        //观测噪声雅可比矩阵
        Jacobian<Measurement, Measurement> V;

        //更新雅可比矩阵
        virtual void updateJcobians(const State& x)
        {
            //默认不更新
            (void)x;
        }
        
    protected:
        LinearMeasurementModel()
        {
            H.setIdentity();
            V.setIdentity();
        }
        ~LinearMeasurementModel(){}
    };

} //namespace filter

#endif