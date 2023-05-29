/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:20:59
 * @LastEditTime: 2023-05-29 22:23:22
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/src/kalman_filter.cpp
 */
#include "../include/kalman_filter.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace filter
{
    KalmanFilter::KalmanFilter(){}

    KalmanFilter::KalmanFilter(KFParam kf_param)
    {
        kf_param_ = kf_param;
    }

    KalmanFilter::~KalmanFilter(){}

    void KalmanFilter::Init(int SP, int MP, int CP)
    {
        assert(SP > 0 && MP > 0);
        this->x_ = Eigen::VectorXd::Zero(SP);
        this->z_ = Eigen::VectorXd::Zero(MP);
        this->C_ = Eigen::MatrixXd::Zero(SP, CP);
        this->F_ = Eigen::MatrixXd::Identity(SP, SP);
        this->Jf_ = Eigen::MatrixXd::Identity(SP, SP);
        this->P_ = Eigen::MatrixXd::Identity(SP, SP);
        this->Q_ = Eigen::MatrixXd::Identity(SP, SP);
        this->H_ = Eigen::MatrixXd::Identity(MP, SP);
        this->Jh_ = Eigen::MatrixXd::Identity(MP, SP);
        this->R_ = Eigen::MatrixXd::Identity(MP, MP);
        this->cp_ = CP;
    }

    void KalmanFilter::Predict()
    {
        if(cp_ == 1)
        {
            x_ = F_ * x_ + C_ * x_[2];
        }
        else if (cp_ == 3)
        {
            Eigen::Vector3d acc = {x_(6), x_(7), x_(8)};
            x_ = F_ * x_ + C_ * acc;
        }
        else
        {
            x_ = F_ * x_;
        }

        P_ = Jf_ * P_ * Jf_.transpose() + Q_;
    }

    void KalmanFilter::Predict(const double& dt)
    {
        this->dt_ = dt;
        Predict();
    }
 
    void KalmanFilter::Update(const VectorXd& z)
    {
        // if (x_.size() > 5)
        // {
        //     cout << "x_pre:" << x_(0) << " " << x_(1) << " " << x_(2) << " " << x_(3) << endl;
        //     cout << "z:" << z(0) << " " << z(1) << " " << z(2) << endl;
        // }

        MatrixXd z_pred = H_ * x_;
        MatrixXd y = z - z_pred;

        // if (x_.size() > 5)
        // {
        //     cout << "z_meas:" << z(0) << " " << z(1) << " " << z(2) << " " << z(3) << endl;
        //     cout << "z_pred:" << z_pred(0, 0) << " " << z_pred(1, 0) << " " << z_pred(2, 0) << z_pred(3, 0) << endl;
        //     cout << "y:" << y(0, 0) << " " << y(1, 0) << " " << y(2, 0) << " " << y(3, 0) << endl;
        // }

        //卡尔曼增益
        MatrixXd Ht = Jh_.transpose();
        MatrixXd PHt = P_ * Ht;
        MatrixXd S = Jh_ * PHt + R_;
        MatrixXd Si = S.inverse();
        MatrixXd K = PHt * Si;

        //update
        x_ = x_ + (K * y);
        
        // if (x_.size() > 5)
        // {
        //     cout << "x_post:" << x_(0) << " " << x_(1) << " " << x_(2) << " " << x_(3) << endl;
        // }

        int x_size = x_.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P_ = (I - K * Jh_) * P_;
    }

    /**
     * @brief 卡尔曼滤波更新步骤
     * 此处应用于IMM模型，需要计算模型似然值
     * @param z 
     * @param mp 
     */
    void KalmanFilter::Update(const Eigen::VectorXd& z, int mp)
    {
        // 测量值与预测值之间的残差
        Eigen::VectorXd v = z - this->x_;
        
        // 残差的协方差矩阵
        Eigen::MatrixXd S = this->H_ * this->P_ * this->H_.transpose() + this->R_;
        this->S_ = S;

        // 计算模型似然值
        double det = S.determinant();
        this->likelihood_ = exp(-0.5 * v.transpose() * S.inverse() * v) / sqrt(pow(2 * M_PI, mp) * det);

        // 计算卡尔曼增益
        Eigen::MatrixXd K = this->P_ * this->H_.transpose() * S.inverse();

        //更新
        this->x_ = this->x_ + K * v;
        Eigen::MatrixXd I;
        I.setIdentity();
        Eigen::MatrixXd C = (I - K * this->H_);
        this->P_ = C * this->P_ * C.transpose() + K * this->R_ * K.transpose();
    }

    void KalmanFilter::updateOnce(const double& dt, const Eigen::VectorXd* z)
    {
        if(z->isZero())
        {   //若没有观测值传入，则只进行预测
            Predict(dt);
        }
        else
        {   
            Predict(dt);
            Update(*z);
        }
    }

    void KalmanFilter::UpdateEKF(const MatrixXd& z)
    {
        //TODO:
    }

    double KalmanFilter::getLikelihoodValue() const
    {
        return this->likelihood_;
    }

    // KalmanFilter::Clone()
    // {
    //     return new KalmanFilter(*this);
    // }

} // filter
