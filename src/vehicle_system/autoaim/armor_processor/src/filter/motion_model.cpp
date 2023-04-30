/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:36:22
 * @LastEditTime: 2023-04-30 17:56:26
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/filter/motion_model.cpp
 */
#include "../../include/filter/motion_model.hpp"

namespace armor_processor
{
    CV::CV()
    {
        // r1_ = 60, r2_ = 60, r3_ = 30, r4_ = 30;
        // q1_ = 0.4, q2_ = 0.4, q3_ = 0.3, q4_ = 0.3, q5_ = 0.2, q6_ = 0.2;
        kf_param_.measure_noise_params = {60, 60, 30, 30};
        kf_param_.process_noise_params = {0.4, 0.4, 0.3, 0.3, 0.2, 0.2};
    }
    CV::~CV() {}
    
    /**
     * @brief 初始化CV模型对应的卡尔曼滤波
     * 
     * @param x 状态向量
     * @param dt_ 时间量
     */
    void CV::init(const Eigen::VectorXd& x, const double& dt)
    {   
        // 状态向量与模型维度不匹配
        if(x.size() != 6)
        {
            std::cerr << "[Error]Dismatched between State and CV model!" << std::endl;
            exit(1);
        }
        this->dt_ = dt;

        this->P_.setIdentity();
        this->R_.resize(4, 4);
        double r[] = {kf_param_.measure_noise_params[0], kf_param_.measure_noise_params[1], kf_param_.measure_noise_params[2], kf_param_.measure_noise_params[3]};
        this->R_ << r[0], 0,   0,   0,
                    0,   r[1], 0,   0,
                    0,   0,   r[2], 0,
                    0,   0,   0,   r[3];
                    
        this->x_ = x;  //x(x, y, theta, v)
        this->F_.resize(6, 6);
        this->H_.resize(4, 6);
        this->H_ << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, 0, 0;
        this->z_.resize(4);
    }

    /**
     * @brief 更新预测信息
     * 
     */
    void CV::updatePrediction()
    {
        // double vx = this->x_(2);
        // double vy = this->x_(3);

        this->F_ << 1, 0, dt_, 0,  0, 0,
                    0, 1, 0,  dt_, 0, 0,
                    0, 0, 1,  0,  0, 0,
                    0, 0, 0,  1,  0, 0,
                    0, 0, 0,  0,  1, 0,
                    0, 0, 0,  0,  0, 1;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        // Eigen::Matrix<double, 6, 2> G;
        // G << 0.5 * pow(dt_, 2),                0,
        //                     0, 0.5 * pow(dt_, 2),
        //                     dt_,               0,
        //                     0,               dt_,
        //                     0,                0,
        //                     0,                0;
        // Eigen::Matrix2d E;
        // E << 400, 0,
        //        0, 400;
        
        // this->Q_ = G * E * G.transpose();
        this->Q_.setIdentity(6, 6);
        double q[] = {kf_param_.process_noise_params[0], kf_param_.process_noise_params[1], kf_param_.process_noise_params[2],
            kf_param_.process_noise_params[3], kf_param_.process_noise_params[4], kf_param_.process_noise_params[5]};
        this->Q_ << q[0], 0,   0,   0,   0,   0,
                    0,   q[1], 0,   0,   0,   0,
                    0,   0,   q[2], 0,   0,   0,
                    0,   0,   0,   q[3], 0,   0,
                    0,   0,   0,   0,   q[4], 0,
                    0,   0,   0,   0,   0,   q[5];
    }

    /**
     * @brief 更新测量信息
     * 
     */
    void CV::updateMeasurement()
    {}

    CA::CA()
    {
        // r1_ = 60, r2_ = 60, r3_ = 30, r4_ = 30;
        // q1_ = 0.4, q2_ = 0.4, q3_ = 0.3, q4_ = 0.3, q5_ = 0.2, q5_ = 0.2;
        kf_param_.measure_noise_params = {60, 60, 30, 30};
        kf_param_.process_noise_params = {0.4, 0.4, 0.3, 0.3, 0.2, 0.2};
    }
    CA::~CA(){}

    void CA::init(const Eigen::VectorXd& x, const double& dt)
    {
        if(x.size() != 6)
        {
            std::cerr << "[Error] Dismatch between State and CA model!" << std::endl;
            exit(1);
        }
        this->x_ = x;
        this->dt_ = dt;

        this->P_.setIdentity();
        this->R_.resize(4, 4);
        double r[] = {kf_param_.measure_noise_params[0], kf_param_.measure_noise_params[1], kf_param_.measure_noise_params[2], kf_param_.measure_noise_params[3]};
        this->R_ << r[0], 0,   0,   0,
                    0,   r[1], 0,   0,
                    0,   0,   r[2], 0,
                    0,   0,   0,   r[3];

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
        this->F_ << 1, 0, dt_,   0, 0.5 * pow(dt_, 2),              0,
                    0, 1,  0,  dt_,                 0, 0.5 * (dt_, 2),
                    0, 0,  1,   0,                dt_,             0,
                    0, 0,  0,   1,                 0,            dt_,
                    0, 0,  0,   0,                 1,             0,
                    0, 0,  0,   0,                 0,             1;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        // Eigen::Matrix<double, 6, 2> G;
        // G << 1 / 6.0 * pow(dt_, 3),                    0,
        //                         0, 1 / 6.0 * pow(dt_, 3), 
        //      1 / 2.0 * pow(dt_, 2),                    0,
        //                         0, 1 / 2.0 * pow(dt_, 2),
        //                        dt_,                    0, 
        //                         0,                   dt_;
        // Eigen::Matrix2d E;
        // E << 400,   0,
        //        0, 400;
        // this->Q_ = G * E * G.transpose();    
        this->Q_.setIdentity(6, 6);
        double q[] = {kf_param_.process_noise_params[0], kf_param_.process_noise_params[1], kf_param_.process_noise_params[2],
            kf_param_.process_noise_params[3], kf_param_.process_noise_params[4], kf_param_.process_noise_params[5]};
        this->Q_ << q[0], 0,   0,   0,   0,   0,
                    0,   q[1], 0,   0,   0,   0,
                    0,   0,   q[2], 0,   0,   0,
                    0,   0,   0,   q[3], 0,   0,
                    0,   0,   0,   0,   q[4], 0,
                    0,   0,   0,   0,   0,   q[5];
    }   

    void CA::updateMeasurement()
    {}

    CT::CT(const double& w):w_(w) 
    {
        // r1_ = 60, r2_ = 60, r3_ = 30, r4_ = 30;
        // q1_ = 0.4, q2_ = 0.4, q3_ = 0.3, q4_ = 0.3, q5_ = 0.2, q5_ = 0.2;
        kf_param_.measure_noise_params = {60, 60, 30, 30};
        kf_param_.process_noise_params = {0.4, 0.4, 0.3, 0.3, 0.2, 0.2};
    }
    CT::~CT() {}

    void CT::init(const Eigen::VectorXd& x, const double& dt)
    {
        assert(x.size() == 6);
        this->x_ = x;

        this->P_.setIdentity(6, 6);
        this->R_.resize(4, 4);
        double r[4] = {
            kf_param_.measure_noise_params[0], 
            kf_param_.measure_noise_params[1], 
            kf_param_.measure_noise_params[2], 
            kf_param_.measure_noise_params[3]
        };
        this->R_ << r[0], 0,   0,   0,
                    0,   r[1], 0,   0,
                    0,   0,   r[2], 0,
                    0,   0,   0,   r[3];

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
        this->F_ << 1, 0,       sin(w_ * dt_) / w_, (cos(w_ * dt_) - 1) / w_, 0, 0,
                    0, 1, (1 - cos(w_ * dt_)) / w_,       sin(w_ * dt_) / w_, 0, 0,
                    0, 0,            cos(w_ * dt_),           -sin(w_ * dt_), 0, 0,
                    0, 0,            sin(w_ * dt_),            cos(w_ * dt_), 0, 0,
                    0, 0,                       0,                       0, 0, 0,
                    0, 0,                       0,                       0, 0, 0;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        {
            // Eigen::Matrix<double, 6, 2> G;
            // G << 0.5 * pow(dt_, 2),                0,
            //                     0, 0.5 * pow(dt_, 2),
            //                    dt_,                0,
            //                     0,               dt_,
            //                     0,                0,
            //                     0,                0;
            
            // Eigen::Matrix2d E;
            // E << 400,   0,
            //        0, 400;

            // this->Q_ = G * E * G.transpose();
            this->Q_.setIdentity(6, 6);
            double q[] = {kf_param_.process_noise_params[0], kf_param_.process_noise_params[1], kf_param_.process_noise_params[2],
            kf_param_.process_noise_params[3], kf_param_.process_noise_params[4], kf_param_.process_noise_params[5]};
            this->Q_ << q[0], 0,   0,   0,   0,   0,
                        0,   q[1], 0,   0,   0,   0,
                        0,   0,   q[2], 0,   0,   0,
                        0,   0,   0,   q[3], 0,   0,
                        0,   0,   0,   0,   q[4], 0,
                        0,   0,   0,   0,   0,   q[5];
        }           
    }

    void CT::updateMeasurement()
    {
        
    }

    UniformModel::UniformModel(const KFParam kf_param)
    {
        kf_param_ = kf_param;
    }

    UniformModel::UniformModel()
    {
        radius_ = 0.25;
        kf_param_.process_noise_params = {1.0, 1.0};
        kf_param_.measure_noise_params = {1.0, 1.0, 1.0, 1.0};
        kf_param_.singer_params = {8.00, 10.0, 0.1, 0.8, 5.00, 0.0030, 1.0, 1.0, 7.0};
    }

    UniformModel::~UniformModel()
    {

    }

    void UniformModel::init()
    {
        double alpha = kf_param_.singer_params[0];
        double sigma = kf_param_.singer_params[5];

        this->x_.resize(11);
        // this->dt_ = dt;
        this->P_.setIdentity(11, 11);
        this->F_.resize(11, 11);
        this->C_.resize(11, 3);
        /*
                    // Xc--Yc--Zc--r--theta-omega-vx--vy--vz----------------ax---------------ay-----------------az
                    Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
        */
        // this->F_ << 1,  0,  0,  0,  0,    dt,  0,  0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,                                                   0,
        //             0,  1,  0,  0,  0,     0, dt,  0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,
        //             0,  0,  1,  0,  0,     0,  0, dt,                                                   0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
        //             0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
        //             0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
        //             0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,                                                   0,
        //             0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,
        //             0,  0,  0,  0,  0,     0,  0,  1,                                                   0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,
        //             0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt),                                                   0,                                                   0,
        //             0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt),                                                   0,
        //             0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                    exp(-alpha * dt);
        
        this->z_.resize(4);
        this->H_.resize(4, 11);
        this->R_.resize(4, 4);
        double r[4] = {
            this->kf_param_.measure_noise_params[0],
            this->kf_param_.measure_noise_params[1],
            this->kf_param_.measure_noise_params[2],
            this->kf_param_.measure_noise_params[3],
        };
        this->R_ << r[0],    0,    0,    0,   
                       0, r[1],    0,    0,    
                       0,    0, r[2],    0,    
                       0,    0,    0, r[3];

        this->Q_.setIdentity(11, 11);

        setKF(this->dt_);
    }

    void UniformModel::setKF(double dt)
    {
        this->dt_ = dt;
        double alpha = kf_param_.singer_params[0];
        double sigma = kf_param_.singer_params[5];
        /*
                    Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
        */
        this->F_ << 1,  0,  0,  0,  0,    dt,  0,  0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,                                                   0,
                    0,  1,  0,  0,  0,     0, dt,  0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,
                    0,  0,  1,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt),                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt),                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0;

        double q[2] = {this->kf_param_.process_noise_params[0], this->kf_param_.process_noise_params[1]};
        double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
        double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
        double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
        double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
        double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
        double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
        this->Q_ << 2 * pow(sigma, 2) * alpha * q11,                               0,                               0,    0,    0,    2 * pow(sigma, 2) * alpha * q12,                                  0,                                  0,    2 * pow(sigma, 2) * alpha* q13,                                 0,                                  0,    
                                                  0, 2 * pow(sigma, 2) * alpha * q11,                               0,    0,    0,                                  0,    2 * pow(sigma, 2) * alpha * q12,                                  0,                                 0,    2 * pow(sigma, 2) * alpha* q13,                                  0,    
                                                  0,                               0,                               1,    0,    0,                                  0,                                  0,                                  1,                                 0,                                 0,                                  1,   
                                                  0,                               0,                               0, q[0],    0,                                  0,                                  0,                                  0,                                 0,                                 0,                                  0,   
                                                  0,                               0,                               0,    0, q[1],                                  0,                                  0,                                  0,                                 0,                                 0,                                  0,   
                    2 * pow(sigma, 2) * alpha * q12,                               0,                               0,    0,    0,    2 * pow(sigma, 2) * alpha * q22,                                  0,                                  0,    2 * pow(sigma, 2) * alpha* q23,                                 0,                                  0,   
                                                  0, 2 * pow(sigma, 2) * alpha * q12,                               0,    0,    0,                                  0,    2 * pow(sigma, 2) * alpha * q22,                                  0,                                 0,    2 * pow(sigma, 2) * alpha* q23,                                  0,   
                                                  0,                               0,                               1,    0,    0,                                  0,                                  0,                                  1,                                 0,                                 0,                                  1,  
                    2 * pow(sigma, 2) * alpha * q13,                               0,                               0,    0,    0,    2 * pow(sigma, 2) * alpha * q23,                                  0,                                  0,   2 * pow(sigma, 2) * alpha * q33,                                 0,                                  0,  
                                                  0, 2 * pow(sigma, 2) * alpha * q13,                               0,    0,    0,                                  0,    2 * pow(sigma, 2) * alpha * q23,                                  0,                                 0,   2 * pow(sigma, 2) * alpha * q33,                                  0,  
                                                  0,                               0,                               1,    0,    0,                                  0,                                  0,                                  1,                                 0,                                 0,                                  1;
        
        // dt /= 2;
        this->C_ << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * (dt)) / alpha)),                                                                      0,                                                                        0,
                                                                                           0, 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),                                                                        0,
                                                                                           0,                                                                        0,                                                                        1,
                                                                                           0,                                                                        0,                                                                        0,
                                                                                           0,                                                                        0,                                                                        0,
                                                         dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,                                                                        0,
                                                                                           0,                                      dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,
                                                                                           0,                                                                        0,                                                                        1,
                                                                        1 - exp(-alpha * dt),                                                                        0,                                                                        0,
                                                                                           0,                                                     1 - exp(-alpha * dt),                                                                        0,
                                                                                           0,                                                                        0,                                                                        1;
        
        // this->H_ << 1, 0, 0, -sin(rangle_), 0, 0, 0, 0, 0, 0, 0,
        //             0, 1, 0,  cos(rangle_), 0, 0, 0, 0, 0, 0, 0,
        //             0, 0, 1,             0, 0, 0, 0, 0, 0, 0, 0,
        //             0, 0, 0,             0, 1, 0, 0, 0, 0, 0, 0;
    }

    void UniformModel::setF(Eigen::MatrixXd& Ft, const double& dt)
    {
        double alpha = kf_param_.singer_params[0];
        double sigma = kf_param_.singer_params[5];
        /*
              Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
        */
        Ft << 1,  0,  0,  0,  0,    dt,  0,  0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,                                                   0,
                    0,  1,  0,  0,  0,     0, dt,  0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,
                    0,  0,  1,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt),                                                   0,                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt),                                                   0,
                    0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0;
    }

    void UniformModel::setC(Eigen::MatrixXd& Ct, const double& dt)
    {
        double alpha = kf_param_.singer_params[0];
        Ct << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * (dt)) / alpha)),                                                                      0,                                                                        0,
                                                                                           0, 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),                                                                        0,
                                                                                           0,                                                                        0,                                                                        1,
                                                                                           0,                                                                        0,                                                                        0,
                                                                                           0,                                                                        0,                                                                        0,
                                                         dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,                                                                        0,
                                                                                           0,                                      dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,
                                                                                           0,                                                                        0,                                                                        1,
                                                                        1 - exp(-alpha * dt),                                                                        0,                                                                        0,
                                                                                           0,                                                     1 - exp(-alpha * dt),                                                                        0,
                                                                                           0,                                                                        0,                                                                        1;
    }

    void UniformModel::updatePrediction()
    {
        // double alpha = kf_param_.singer_params[0];
        // double sigma = kf_param_.singer_params[5];
        // /*
        //       Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
        // */
        // F_ << 1,  0,  0,  0,  0,   dt_,  0,  0, (alpha * dt_ - 1 + exp(-alpha * dt_)) / alpha / alpha,                                                   0,                                                   0,
        //       0,  1,  0,  0,  0,     0,dt_,  0,                                                   0, (alpha *dt_ - 1 + exp(-alpha * dt_)) / alpha / alpha,                                                   0,
        //       0,  0,  1,  0,  0,     0,  0,dt_,                                                   0,                                                   0, (alpha * dt_ - 1 + exp(-alpha * dt_)) / alpha / alpha,
        //       0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
        //       0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
        //       0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt_)) / alpha,                                                   0,                                                   0,
        //       0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt_)) / alpha,                                                   0,
        //       0,  0,  0,  0,  0,     0,  0,  1,                                                   0,                                                   0,                      (1 - exp(-alpha * dt_)) / alpha,
        //       0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt_),                                                   0,                                                   0,
        //       0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt_),                                                   0,
        //       0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                    exp(-alpha * dt_);
        
        // Eigen::MatrixXd acc;
        // acc.resize(3, 1);
        // acc << x_(8), x_(9), x_(10);
        // if (this->cp_ > 0)
        // {
        //     x_ = F_ * x_ + C_ * acc;
        // }
        // else
        // {
        //     x_ = F_ * x_;
        // }
        this->x_ = this->F_ * this->x_;
        // cout << "F:" << F_(0, 0) << " " << F_(0, 1) << " " << F_(0, 2)<< " "  << F_(0, 3) << " "  << F_(0, 4) << " "  << F_(0, 5) << " "  << F_(0, 6) << " "  << F_(0, 7) << " "  << F_(0, 8) << " "  << F_(0, 9) << " "  << F_(0, 10) << endl <<
        //         "  " << F_(1, 0) << " " << F_(1, 1) << " " << F_(1, 2)<< " "  << F_(1, 3) << " "  << F_(1, 4) << " "  << F_(1, 5) << " "  << F_(1, 6) << " "  << F_(1, 7) << " "  << F_(1, 8) << " "  << F_(1, 9) << " "  << F_(1, 10) << endl <<
        //         "  " << F_(2, 0) << " " << F_(2, 1) << " " << F_(2, 2)<< " "  << F_(2, 3) << " "  << F_(2, 4) << " "  << F_(2, 5) << " "  << F_(2, 6) << " "  << F_(2, 7) << " "  << F_(2, 8) << " "  << F_(2, 9) << " "  << F_(2, 10) << endl <<
        //         "  " << F_(3, 0) << " " << F_(3, 1) << " " << F_(3, 2)<< " "  << F_(3, 3) << " "  << F_(3, 4) << " "  << F_(3, 5) << " "  << F_(3, 6) << " "  << F_(3, 7) << " "  << F_(3, 8) << " "  << F_(3, 9) << " "  << F_(3, 10) << endl <<
        //         "  " << F_(4, 0) << " " << F_(4, 1) << " " << F_(4, 2)<< " "  << F_(4, 3) << " "  << F_(4, 4) << " "  << F_(4, 5) << " "  << F_(4, 6) << " "  << F_(4, 7) << " "  << F_(4, 8) << " "  << F_(4, 9) << " "  << F_(4, 10) << endl <<
        //         "  " << F_(5, 0) << " " << F_(5, 1) << " " << F_(5, 2)<< " "  << F_(5, 3) << " "  << F_(5, 4) << " "  << F_(5, 5) << " "  << F_(5, 6) << " "  << F_(5, 7) << " "  << F_(5, 8) << " "  << F_(5, 9) << " "  << F_(5, 10) << endl <<
        //         "  " << F_(6, 0) << " " << F_(6, 1) << " " << F_(6, 2)<< " "  << F_(6, 3) << " "  << F_(6, 4) << " "  << F_(6, 5) << " "  << F_(6, 6) << " "  << F_(6, 7) << " "  << F_(6, 8) << " "  << F_(6, 9) << " "  << F_(6, 10) << endl <<
        //         "  " << F_(7, 0) << " " << F_(7, 1) << " " << F_(7, 2)<< " "  << F_(7, 3) << " "  << F_(7, 4) << " "  << F_(7, 5) << " "  << F_(7, 6) << " "  << F_(7, 7) << " "  << F_(7, 8) << " "  << F_(7, 9) << " "  << F_(7, 10) << endl <<
        //         "  " << F_(8, 0) << " " << F_(8, 1) << " " << F_(8, 2)<< " "  << F_(8, 3) << " "  << F_(8, 4) << " "  << F_(8, 5) << " "  << F_(8, 6) << " "  << F_(8, 7) << " "  << F_(8, 8) << " "  << F_(8, 9) << " "  << F_(8, 10) << endl <<
        //         "  " << F_(9, 0) << " " << F_(9, 1) << " " << F_(9, 2)<< " "  << F_(9, 3) << " "  << F_(9, 4) << " "  << F_(9, 5) << " "  << F_(9, 6) << " "  << F_(9, 7) << " "  << F_(9, 8) << " "  << F_(9, 9) << " "  << F_(9, 10) << endl <<
        //         "  " << F_(10, 0) << " " << F_(10, 1) << " " << F_(10, 2)<< " "  << F_(10, 3) << " "  << F_(10, 4) << " "  << F_(10, 5) << " "  << F_(10, 6) << " "  << F_(10, 7) << " "  << F_(10, 8) << " "  << F_(10, 9) << " "  << F_(10, 10) << endl;
    }

    void UniformModel::updateMeasurement()
    {
        
    }
} // armor_processor
