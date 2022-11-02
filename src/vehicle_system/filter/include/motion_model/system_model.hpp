/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-02 23:48:25
 * @LastEditTime: 2022-11-03 00:22:44
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/motion_model/system_model.hpp
 */
#ifndef SYSTEM_MODEL_HPP_
#define SYSTEM_MODEL_HPP_

#include <type_traits>
#include "../filter/filter.hpp"

namespace filter
{
    template<class StateType, class ControlType = Vector<typename StateType::Scaler, 0>, template<class> class CovarianceBase = Base>
    class SystemModel : public CovarianceBase<StateType>
    {
        static_assert(StateType::RowsAtCompileTime > 0, "状态向量至少包含一个元素");
        static_assert(ControlType::RowsAtCompileTime > 0, "控制向量至少包含一个元素");
        static_assert(std::is_same(typename StateType::Scaler, typename ControlType::Scaler>::value),
                        "状态向量元素与控制向量元素的类型必须相同");
    public:
        typedef StateType State;
        typedef ControlType Control;
    
    public:
        //状态转移函数
        virtual State f(const State& x, const Control& u) const = 0;
    
    protected:
        SystemModel(){}
        virtual ~SystemModel(){}
    };
} //namespace filter

#endif