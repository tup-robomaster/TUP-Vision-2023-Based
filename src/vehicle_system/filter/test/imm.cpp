/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-06 21:32:09
 * @LastEditTime: 2022-11-08 00:25:44
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/imm.cpp
 */
#include "./imm.hpp"

template<class T>
IMM<T>::IMM()
{
    
}

template<class T>
IMM<T>::~IMM()
{

}

// template<class T>
// void IMM<T>::addModel(const std::shared_ptr<filter::ExtendKalmanFilter>& model)
// {
//     this->models_.push_back(model);
//     this->model_num_++;
// }

template<class T>
void IMM<T>::init(
    const Vector& x,
    const Matrix& P,
    const Matrix& model_prob,
    const Matrix& transfer_prob)
{
    this->state_num_ = x.size();
    this->X_.resize(this->state_num_, this->model_num_);
    for(int ii = 0; ii < this->model_num_; ii++)
    {
        this->X_.col(ii) = this->models_[ii]->getState();
    }

    this->x_ = x;
    this->P_ = P;
    this->model_prob_ = model_prob;
    this->transfer_prob_ = transfer_prob;
}

template<class T>
void IMM<T>::stateInteraction()
{

}

template<class T>
void IMM<T>::updateState()
{

}

template<class T>
void IMM<T>::updateModelProb()
{

}

template<class T>
void IMM<T>::estimateFusion()
{

}