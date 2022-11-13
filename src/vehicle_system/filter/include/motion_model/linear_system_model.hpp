/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 00:23:18
 * @LastEditTime: 2022-11-14 08:25:14
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/motion_model/linear_system_model.hpp
 */
#ifndef LINEAR_SYSTEM_MODEL_HPP_
#define LINEAR_SYSTEM_MODEL_HPP_

#include "./system_model.hpp"

namespace filter
{
    //抽象运动模型
    template<class StateType, class ControlType = Vector<typename StateType::Scaler, 0>, template<class> class CovarianceBase = Base>
    class LinearSystemModel : public SystemModel<StateType, ControlType, CovarianceBase>
    {
    public:
        typedef SystemModel<StateType, ControlType, CovarianceBase> SystemModelBase;
        
        using typename SystemModelBase::State;
        using typename SystemModelBase::Control;
    
    public:
        //运动模型的雅可比矩阵
        Jacobian<State, State> F;
        //运动噪声的雅可比矩阵
        Jacobian<State, State> W;

        virtual void updateJacobians(const State& x, const Control& u, const float& dt)
        {
            //默认不更新
            (void)x;
            (void)u;
            (void)dt;
        }

    protected:
        LinearSystemModel()
        {
            F.setIdentity();
            W.setIdentity();
        }
        ~LinearSystemModel(){}
    };
} //namespace filter

#endif