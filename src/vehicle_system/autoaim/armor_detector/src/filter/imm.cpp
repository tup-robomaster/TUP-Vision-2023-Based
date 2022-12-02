/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 16:30:16
 * @LastEditTime: 2022-12-02 14:58:41
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/filter/imm.cpp
 */
#include "../../include/filter/imm.hpp"

namespace armor_detector
{
    IMM::IMM():model_num_(0){}
    IMM::~IMM(){}

    IMM::IMM(const IMM& imm)
    {}

    void IMM::addModel(const std::shared_ptr<KalmanFilter>& model)
    {
        this->models_.push_back(model);
        ++this->model_num_;
    }

    void IMM::init(
        const Eigen::VectorXd& x,
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& model_prob,
        const Eigen::MatrixXd& transfer_prob)
    {
        assert(this->model_num_ > 0);
        assert(transfer_prob.cols() == this->model_num_ && transfer_prob.rows() == this->model_num_);
        assert(model_prob.size() == this->model_num_);

        this->state_num_ = x.size();
        this->X_.resize(this->state_num_, this->model_num_);
        for(int ii = 0; ii < this->model_num_; ii++)
        {
            this->X_.col(ii) = this->models_[ii]->x();
        }

        this->c_.resize(this->model_num_);
        this->x_ = x;
        this->P_ = P;
        this->model_prob_ = model_prob;
        this->transfer_prob_ = transfer_prob;
    }

    void IMM::init(
        const Eigen::VectorXd& x,
        const Eigen::MatrixXd& model_prob,
        const Eigen::MatrixXd& transfer_prob)
    {
        assert(this->model_num_ > 0);
        assert(transfer_prob.cols() == this->model_num_ && transfer_prob.rows() == this->model_num_);
        assert(model_prob.size() == this->model_num_);

        // std::cout << 1 << std::endl;

        this->state_num_ = x.size();
        this->X_.resize(this->state_num_, this->model_num_);
        for(int ii = 0; ii < this->model_num_; ii++)
        {
            this->X_.col(ii) = this->models_[ii]->x();
        }

        this->c_.resize(this->model_num_);
        this->x_ = x;
        this->model_prob_ = model_prob;
        this->transfer_prob_ = transfer_prob;

        // std::cout << 2 << std::endl;
    }

    void IMM::stateInteraction()
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
        // std::cout << 3 << std::endl;

        Eigen::MatrixXd U = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);
        for(int ii = 0; ii < this->model_num_; ii++)
        {
            this->X_.col(ii) = this->models_[ii]->x();
        }

        // std::cout << 4 << std::endl;

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
        // std::cout << 5 << std::endl;

        //step3
        for(int i = 0; i < this->model_num_; i++)
        {
            // std::cout << "model_num: rows " << i << std::endl;
            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(this->model_num_ + 2, this->model_num_ + 2);
            for(int j = 0; j < this->model_num_; j++)
            {
                // std::cout << 7 << std::endl;
                // std::cout << "model_num: cols " << j << std::endl;
                // std::cout << "X_ : rows:" << this->X_.rows() << " cols:" << this->X_.cols() << std::endl;
                Eigen::Matrix<double, 6, 1> s = X.col(i) - X.col(j);

                // std::cout << "X_.cols:" << this->X_.col(i).size() << std::endl;
                // std::cout << "s size:" << s.size() << " cols: " << s.cols() << " rows: "  << s.rows() << std::endl;
                // std::cout << "P cols: " << this->models_[i]->P().cols() << " rows: " << this->models_[i]->P().rows() << std::endl;
                P += U(i, j) * (this->models_[i]->P() + s * s.transpose());
            }
            this->models_[i]->setStateCoveriance(P);
            this->models_[i]->setState(this->X_.col(i));
        }
        // std::cout << 6 << std::endl;
    }

    void IMM::updateState()
    {

    }

    void IMM::updateState(const Eigen::VectorXd& z, const double& dt)
    {
        // std::cout << "measure: " << z[0] << " dt: " << dt << std::endl;
        for(size_t i = 0; i < this->model_num_; i++)
        {
            // std::cout << 8 << std::endl;

            // std::cout << "predict: " << i << std::endl;
            this->models_[i]->Predict(dt);
            // if(z != nullptr)
            // std::cout << "update: " << i << std::endl;
            this->models_[i]->Update(z, 1);
        }

        // std::cout << 7 << std::endl;
    }

    void IMM::updateModelProb()
    {
        //模型概率更新
        double c_sum = 0;
        for(size_t i = 0; i < this->model_num_; i++)
        {
            c_sum += this->models_[i]->getLikelihoodValue() * this->c_(i);
        }
        // std::cout << 8 << std::endl;

        for(size_t i = 0; i < this->model_num_; i++)
        {
            this->model_prob_(i) = (1 / c_sum) * this->models_[i]->getLikelihoodValue() * this->c_(i);
        }
        // std::cout << 9 << std::endl;
    }

    void IMM::estimateFusion()
    {
        // this->x_ = this->X_ * this->model_prob_;
        // this->x_ = this->models_[3]->x();
        // std::cout << 30 << std::endl;
        this->x_.setZero();
        for(size_t i = 0; i < this->model_num_; i++)
        {
            // std::cout << "Model" << i << " prob:" << this->model_prob_[i] << std::endl; 
            // std::cout << 31 << std::endl;
            // std::cout << "Model" << i << " prob:" << this->model_prob_[i] << std::endl; 
            // std::cout << "x: " << this->models_[i]->x()[0] << std::endl;
            // std::cout << 6 << std::endl;
            this->models_[i]->setCoeff(5.0);
            this->x_ += this->models_[i]->F_ * this->models_[i]->x() * this->model_prob_[i];
            
            //TODO:
            // Eigen::MatrixXd v = this->X_.col(i) - this->x_;
            // this->P_ += this->model_prob_[i] * (this->models_[i]->P() + v * v.transpose());
        } 
        // std::cout << 10 << std::endl;
    }

    void IMM::updateOnce(const Eigen::VectorXd& z, const double& dt)
    {
        if(z.isZero())
        {
            stateInteraction();
            updateState(z, dt);
            estimateFusion();
        }
        else
        {
            stateInteraction();
            updateState(z, dt);
            updateModelProb();
            estimateFusion();
        }
    }
} //armor_processor