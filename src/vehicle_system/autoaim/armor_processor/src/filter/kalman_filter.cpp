/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:20:59
 * @LastEditTime: 2022-11-20 21:55:08
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/filter/kalman_filter.cpp
 */
#include "../../include/filter/kalman_filter.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace armor_processor
{
    KalmanFilter::KalmanFilter(){}

    KalmanFilter::~KalmanFilter(){}

    void KalmanFilter::Init(int SP, int MP, int CP)
    {
        assert(SP > 0 && MP > 0 && CP > 0);

        this->x_ = Eigen::MatrixXd::Zero(SP, 1);
        this->F_ = Eigen::MatrixXd::Identity(SP, SP);
        this->Q_ = Eigen::MatrixXd::Identity(SP, SP);
        this->H_ = Eigen::MatrixXd::Zero(MP, SP);
        this->R_ = Eigen::MatrixXd::Zero(MP, MP);
        this->P_ = Eigen::MatrixXd::Zero(SP, SP);
        this->C_ = Eigen::MatrixXd::Zero(SP, CP);

        this->cp_ = CP;
    }

    void KalmanFilter::Init(MatrixXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
            MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in)
    {
        x_ = x_in;
        P_ = P_in;
        F_ = F_in;
        H_ = H_in;
        R_ = R_in;
        Q_ = Q_in;
    }

    void KalmanFilter::Init(MatrixXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
            MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in, MatrixXd& J_in)
    {
        x_ = x_in;
        P_ = P_in;
        F_ = F_in;
        H_ = H_in;
        R_ = R_in;
        Q_ = Q_in;
        J_ = J_in;
    }

    void KalmanFilter::Predict()
    {
        if(this->cp_ > 0)
        {
            x_ = F_ * x_ + C_ * x_(2, 0);
        }
        else
        {
            x_ = F_ * x_;
        }

        MatrixXd Ft = F_.transpose();
        
        P_ = F_ * P_ * Ft + Q_;
    }

    void KalmanFilter::Update(const MatrixXd& z)
    {
        MatrixXd z_pred = H_ * x_;

        MatrixXd y = z - z_pred;
        
        //卡尔曼增益
        MatrixXd Ht = H_.transpose();
        MatrixXd PHt = P_ * Ht;
        MatrixXd S = H_ * PHt + R_;
        MatrixXd Si = S.inverse();
        MatrixXd K = PHt * Si;

        //update
        x_ = x_ + (K * y);
        int x_size = x_.size();
        MatrixXd I = MatrixXd::Identity(x_size, x_size);
        P_ = (I - K * H_) * P_;
    }

    void KalmanFilter::UpdateJacobians()
    {
        
    }

    void KalmanFilter::UpdateEKF(const MatrixXd& z)
    {
        //TODO:
    }


} // armor_processor
