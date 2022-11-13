/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-06 21:32:09
 * @LastEditTime: 2022-11-12 20:30:14
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/imm.cpp
 */
#include "./imm.hpp"

// template<class T, template<class> class CovarianceBase = filter::Base>
// IMM<T>::IMM()
// {
    
// }

// template<class T, template<class> class CovarianceBase = filter::Base>
// IMM<T>::~IMM()
// {

// }

// template<class T, template<class> class CovarianceBase = filter::Base>
// void IMM<T>::addModel(const std::shared_ptr<filter::ExtendKalmanFilter>& model)
// {
//     this->models_.push_back(model);
//     this->model_num_++;
// }

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::addCVModel(const std::shared_ptr<filter::ExtendKalmanFilter<CVState>> model)
{
    this->models_.push_back(model);
    this->model_num_++;
}

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::addCAModel(const std::shared_ptr<filter::ExtendKalmanFilter<CAState>> model)
{
    this->models_.push_back(model);
    this->model_num_++;
}

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::addCTRVModel(const std::shared_ptr<filter::ExtendKalmanFilter<CTRVState>> model)
{   
    this->models_.push_back(model);
    this->model_num_++;
}

template<class T, template<class> class CovarianceBase = filter::Base>
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

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::stateInteraction()
{
    //计算交互后各模型概率
    //step1
    this->c_ = Eigen::VectorXd::Zero(this->model_num_);
    for(size_t j = 0; j < this->model_num_; j++)
    {
        for(size_t i = 0; i < this->model_num_; i++)
        {
            this->c_(j) += this->transfer_prob_(i, j) * this->model_prob_(i);
        }
    }

    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);
    for(int ii = 0; ii < this->model_num_; ii++)
    {
        this->X_.col(ii) = this->models_[ii]->getState();
    }

    Eigen::MatrixXd X = this->X_;
    this->X_.fill(0);
    //step2
    for(int j = 0; j < this->model_num_; j++)
    {
        for(int i = 0; i < this->model_num_; i++)
        {
            U(i, j) = (1 / this->c_(j)) * this->transfer_prob_(i, j) * this->model_prob_(i);
            this->X_.col(j) += X.col(i) * U(i, j);
        }
    }

    //step3
    for(int i = 0; i < this->model_num_; i++)
    {
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);
        for(int j = 0; j < this->model_num_; j++)
        {
            Eigen::VectorXd s = this->X_.col(i) - this->X_.col(j);
            P += U(i, j) * (this->models_[i]->getCovariance() + s * s.transpose());
        }
        this->models_[i]->setCovariance(P);
        this->models_[i]->setState(this->X_.col(i));
    }
}

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::updateState()
{

}

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::updateState(std::vector<SystemModel>& system_model, std::vector<PositionModel>& measurement_model, const Eigen::VectorXd& z, const double& dt)
{
    for(size_t i = 0; i < this->model_num_; i++)
    {
        this->models_[i]->predict(system_model[i], dt);
        this->models_[i]->update(measurement_model[i], z);
    }
}

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::updateModelProb()
{
    //模型概率更新
    double c_sum = 0;
    for(size_t i = 0; i < this->model_num_; i++)
    {
        c_sum += this->models_[i]->likelihood() * this->c_(i);
    }

    for(size_t i = 0; i < this->model_num_; i++)
    {
        this->model_prob_(i) = (1 / c_sum) * this->models_[i]->likelihood() * this->c_(i);
    }
}

template<class T, template<class> class CovarianceBase = filter::Base>
void IMM<T>::estimateFusion()
{
    this->x_ = this->X_ * this->model_prob_;

    for(size_t i = 0; i < this->model_num_; i++)
    {
        //TODO:
        Eigen::MatrixXd v = this->X_.col(i) - this->x_;
        this->P_ += this->model_prob_[i] * (this->models_[i]->getCovariance() + v * v.transpose());
    } 
}