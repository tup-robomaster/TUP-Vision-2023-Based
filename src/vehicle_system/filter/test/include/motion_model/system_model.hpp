/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-02 23:48:25
 * @LastEditTime: 2022-11-06 13:11:43
 * @FilePath: /filter/include/motion_model/system_model.hpp
 */
#ifndef SYSTEM_MODEL_HPP
#define SYSTEM_MODEL_HPP

#include <type_traits>
#include "../filter/filter.hpp"

namespace filter
{
    template<class StateType, class ControlType = Vector<typename StateType::Scaler, 0>, template<class> class CovarianceBase = Base>
    class SystemModel : public CovarianceBase<StateType>
    {
        static_assert(StateType::RowsAtCompileTime > 0, "状态向量至少包含一个元素");
        static_assert(ControlType::RowsAtCompileTime > 0, "控制向量至少包含一个元素");
        static_assert(std::is_same<typename StateType::Scalar, typename ControlType::Scalar>::value,
                        "状态向量元素与控制向量元素的类型必须相同");
    public:
        typedef StateType State;
        typedef ControlType Control;
    
    public:
        //状态转移函数
        virtual State f(const State& x, const Control& u, const float& t) const = 0;
    
    protected:
        SystemModel(){}
        virtual ~SystemModel(){}
    };
} //namespace filter

#endif