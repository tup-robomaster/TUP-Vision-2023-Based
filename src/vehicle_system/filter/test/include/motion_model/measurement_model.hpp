/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 12:28:37
 * @LastEditTime: 2022-11-04 21:19:08
 * @FilePath: /filter/include/motion_model/measurement_model.hpp
 */
#ifndef MEASUREMENT_MODEL_HPP_
#define MEASUREMENT_MODEL_HPP_

#include <type_traits>
#include "../filter/filter.hpp"

namespace filter
{
    template<class StateType, class MeasurementType, template<class> class CovarianceBase = Base>
    class MeasurementModel : public CovarianceBase<MeasurementType>
    {
        static_assert(StateType::RowsAtCompileTime > 0, "状态向量至少包含一个元素");
        static_assert(MeasurementType::RowsAtCompileTime > 0, "观测向量至少包含一个元素");
        static_assert(std::is_same<typename StateType::Scalar, typename MeasurementType::Scalar>::value,
                        "状态向量元素与控制向量元素的类型必须相同");
    
    public:
        typedef StateType State;
        typedef MeasurementType Measurement;

    public:
        //观测模型函数
        virtual Measurement h(const State& x) const = 0;
    
    protected:
        MeasurementModel(){}
        virtual ~MeasurementModel(){}
    };

} //namespace filter

#endif