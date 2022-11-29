/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:36:22
 * @LastEditTime: 2022-11-29 19:32:38
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/filter/motion_model.cpp
 */
#include "../../include/filter/motion_model.hpp"

namespace armor_processor
{
    CV::CV() {}
    CV::~CV() {}
    
    void CV::init(const Eigen::VectorXd& x, const double& dt)
    {   
        // 状态向量与模型维度不匹配
        if(x.size() != 6)
        {
            std::cerr << "[Error]Dismatched between State and CV model!" << std::endl;
            exit(1);
        }

        this->P_.setIdentity(6, 6);

        this->R_.resize(4, 4);
        this->R_ << 60,    0,   0,   0,
                    0,    60,   0,   0,
                    0,    0,   30,   0,
                    0,    0,   0,   30;

        this->x_ = x;  //x(x, y, theta, v)
        this->F_.resize(6, 6);
        this->H_.resize(4, 6);
        this->H_ << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, 0, 0;
        this->z_.resize(4);
    }

    void CV::updatePrediction()
    {
        // double vx = this->x_(2);
        // double vy = this->x_(3);

        this->F_ << 1, 0, dt, 0,  0, 0,
                    0, 1, 0,  dt, 0, 0,
                    0, 0, 1,  0,  0, 0,
                    0, 0, 0,  1,  0, 0,
                    0, 0, 0,  0,  1, 0,
                    0, 0, 0,  0,  0, 1;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        Eigen::Matrix<double, 6, 2> G;
        G << 0.5 * pow(dt, 2),                0,
                            0, 0.5 * pow(dt, 2),
                            dt,               0,
                            0,               dt,
                            0,                0,
                            0,                0;
        Eigen::Matrix2d E;
        E << 400, 0,
               0, 400;
        
        this->Q_.setIdentity();
        // this->Q_ = G * E * G.transpose();

        this->Q_ << 0.4, 0, 0, 0,  0, 0,
                    0, 0.4, 0,  0, 0, 0,
                    0, 0, 0.3,  0,  0, 0,
                    0, 0, 0,  0.3,  0, 0,
                    0, 0, 0,  0,  0.2, 0,
                    0, 0, 0,  0,  0, 0.2;
    }

    void CV::setCoeff(const double& coeff)
    {
        this->F_ << 1, 0, coeff * dt, 0,  0, 0,
                    0, 1, 0,  coeff * dt, 0, 0,
                    0, 0, 1,  0,  0, 0,
                    0, 0, 0,  1,  0, 0,
                    0, 0, 0,  0,  1, 0,
                    0, 0, 0,  0,  0, 1;
    }

    void CV::updateMeasurement()
    {}

    CA::CA(){}
    CA::~CA(){}

    void CA::init(const Eigen::VectorXd& x, const double& dt)
    {
        if(x.size() != 6)
        {
            std::cerr << "[Error] Dismatch between State and CA model!" << std::endl;
            exit(1);
        }
        this->x_ = x;

        this->P_.setIdentity(6, 6);
        this->R_.resize(4, 4);
        this->R_ << 60,    0,   0,   0,
                    0,    60,   0,   0,
                    0,    0,   30,   0,
                    0,    0,   0,   30;

        this->F_.resize(6, 6);
        this->H_.resize(4, 6);
        this->H_ << 1, 0,  0, 0, 0, 0,
                    0, 1,  0, 0, 0, 0,
                    0, 0,  1, 0, 0, 0,
                    0, 0,  0, 1, 0, 0;

        this->z_.resize(4);
    }

    void CA::updatePrediction()
    {
        this->F_ << 1, 0, dt,   0, 0.5 * pow(dt, 2),              0,
                    0, 1,  0,  dt,                 0, 0.5 * (dt, 2),
                    0, 0,  1,   0,                dt,             0,
                    0, 0,  0,   1,                 0,            dt,
                    0, 0,  0,   0,                 1,             0,
                    0, 0,  0,   0,                 0,             1;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        Eigen::Matrix<double, 6, 2> G;
        G << 1 / 6.0 * pow(dt, 3),                    0,
                                0, 1 / 6.0 * pow(dt, 3), 
             1 / 2.0 * pow(dt, 2),                    0,
                                0, 1 / 2.0 * pow(dt, 2),
                               dt,                    0, 
                                0,                   dt;
        Eigen::Matrix2d E;
        E << 400,   0,
               0, 400;
               
        this->Q_.setIdentity(6, 6);
        // this->Q_ = G * E * G.transpose();    

        this->Q_ << 0.4, 0, 0, 0,  0, 0,
                    0, 0.4, 0,  0, 0, 0,
                    0, 0, 0.3,  0,  0, 0,
                    0, 0, 0,  0.3,  0, 0,
                    0, 0, 0,  0,  0.2, 0,
                    0, 0, 0,  0,  0, 0.2;
    }   

    void CA::setCoeff(const double& coeff)
    {
        this->F_ << 1, 0, coeff * dt,   0, 0.5 * pow(coeff * dt, 2),              0,
                    0, 1,  0,  coeff * dt,                 0, 0.5 * pow(coeff * dt, 2),
                    0, 0,  1,   0,                coeff * dt,             0,
                    0, 0,  0,   1,                 0,            coeff * dt,
                    0, 0,  0,   0,                 1,             0,
                    0, 0,  0,   0,                 0,             1;
    }

    void CA::updateMeasurement()
    {}

    CT::CT(const double& w):w_(w) {}
    CT::~CT() {}

    void CT::init(const Eigen::VectorXd& x, const double& dt)
    {
        assert(x.size() == 6);
        this->x_ = x;

        this->P_.setIdentity(6, 6);
        this->R_.resize(4, 4);
        this->R_ << 60,    0,   0,   0,
                    0,    60,   0,   0,
                    0,    0,   30,   0,
                    0,    0,   0,   30;

        this->F_.resize(6, 6);
        this->H_.resize(4, 6);
        this->H_ << 1, 0,  0, 0, 0, 0,
                    0, 1,  0, 0, 0, 0,
                    0, 0,  1, 0, 0, 0,
                    0, 0,  0, 1, 0, 0;
        this->z_.resize(4);
    }

    void CT::updatePrediction()
    {
        this->F_ << 1, 0,       sin(w_ * dt) / w_, (cos(w_ * dt) - 1) / w_, 0, 0,
                    0, 1, (1 - cos(w_ * dt)) / w_,       sin(w_ * dt) / w_, 0, 0,
                    0, 0,            cos(w_ * dt),           -sin(w_ * dt), 0, 0,
                    0, 0,            sin(w_ * dt),            cos(w_ * dt), 0, 0,
                    0, 0,                       0,                       0, 0, 0,
                    0, 0,                       0,                       0, 0, 0;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        {
            Eigen::Matrix<double, 6, 2> G;
            G << 0.5 * pow(dt, 2),                0,
                                0, 0.5 * pow(dt, 2),
                               dt,                0,
                                0,               dt,
                                0,                0,
                                0,                0;
            
            Eigen::Matrix2d E;
            E << 400,   0,
                   0, 400;

            this->Q_.setIdentity(6, 6); 
            // this->Q_ = G * E * G.transpose();
            this->Q_ << 0.4, 0, 0, 0,  0, 0,
                        0, 0.4, 0,  0, 0, 0,
                        0, 0, 0.3,  0,  0, 0,
                        0, 0, 0,  0.3,  0, 0,
                        0, 0, 0,  0,  0.2, 0,
                        0, 0, 0,  0,  0, 0.2;
        }
    }

    void CT::setCoeff(const double& coeff)
    {
        this->F_ << 1, 0,       sin(w_ * coeff * dt) / w_, (cos(w_ * coeff * dt) - 1) / w_, 0, 0,
                            0, 1, (1 - cos(w_ * coeff * dt)) / w_,       sin(w_ * coeff * dt) / w_, 0, 0,
                            0, 0,            cos(w_ * coeff * dt),           -sin(w_ * coeff * dt), 0, 0,
                            0, 0,            sin(w_ * coeff * dt),            cos(w_ * coeff * dt), 0, 0,
                            0, 0,                       0,                       0, 0, 0,
                            0, 0,                       0,                       0, 0, 0;
    }

    void CT::updateMeasurement()
    {}

    
} // armor_processor
