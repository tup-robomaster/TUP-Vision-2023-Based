/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:20:59
 * @LastEditTime: 2022-11-06 15:33:12
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/src/kalman_filter.cpp
 */
#include "../include/kalman_filter.hpp"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace kalman_filter
{
    KalmanFilter::KalmanFilter(){}

    KalmanFilter::~KalmanFilter(){}

    void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
            MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in)
    {
        x_ = x_in;
        P_ = P_in;
        F_ = F_in;
        H_ = H_in;
        R_ = R_in;
        Q_ = Q_in;
    }

    void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
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
        x_ = F_ * x_;

        MatrixXd Ft = F_.transpose();

        P_ = F_ * P_ * Ft + Q_;
    }

    void KalmanFilter::Update(const VectorXd& z)
    {
        VectorXd z_pred = H_ * x_;

        VectorXd y = z - z_pred;
        
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

    void KalmanFilter::UpdateEKF(const VectorXd& z)
    {
        //TODO:
    }


} // kalman_filter
