/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 13:10:34
 * @LastEditTime: 2022-11-12 11:36:26
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/motion_model/kalman_filter_base.hpp
 */
#ifndef KALMAN_FILTER_BASE_HPP_
#define KALMAN_FILTER_BASE_HPP_

#include "../filter/filter.hpp"
namespace filter
{
    //抽象基类适配各种卡尔曼滤波
    template<class StateType>
    class KalmanFilterBase
    {
    public:
        static_assert(StateType::RowsAtCompileTime > 0, "状态向量至少包含一个元素！");
        static_assert(StateType::ColsAtCompileTime == 1, "状态向量必须是列向量!");

        //元素类型
        typedef typename StateType::Scalar T;
        
        typedef StateType State;
    
    public:
        State x;
    
    public:
        //初始化状态向量
        void init(const State& initialState)
        {
            x = initialState;
        }

        //返回状态估计量
        const State& getState() const
        {
            return x;
        }

        bool setState(const State& state)
        {
            x = state;
            return true;
        }

    public:
        KalmanFilterBase(){}
    };

} //namespace filter

#endif
