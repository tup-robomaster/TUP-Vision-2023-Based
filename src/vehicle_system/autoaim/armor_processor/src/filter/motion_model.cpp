/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:36:22
 * @LastEditTime: 2023-02-05 01:00:12
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
     * @param dt 时间量
     */
    void CV::init(const Eigen::VectorXd& x, const double& dt)
    {   
        // 状态向量与模型维度不匹配
        if(x.size() != 6)
        {
            std::cerr << "[Error]Dismatched between State and CV model!" << std::endl;
            exit(1);
        }

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

        this->F_ << 1, 0, dt, 0,  0, 0,
                    0, 1, 0,  dt, 0, 0,
                    0, 0, 1,  0,  0, 0,
                    0, 0, 0,  1,  0, 0,
                    0, 0, 0,  0,  1, 0,
                    0, 0, 0,  0,  0, 1;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        // Eigen::Matrix<double, 6, 2> G;
        // G << 0.5 * pow(dt, 2),                0,
        //                     0, 0.5 * pow(dt, 2),
        //                     dt,               0,
        //                     0,               dt,
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
        this->F_ << 1, 0, dt,   0, 0.5 * pow(dt, 2),              0,
                    0, 1,  0,  dt,                 0, 0.5 * (dt, 2),
                    0, 0,  1,   0,                dt,             0,
                    0, 0,  0,   1,                 0,            dt,
                    0, 0,  0,   0,                 1,             0,
                    0, 0,  0,   0,                 0,             1;
        this->x_ = this->F_ * this->x_;

        //计算状态协方差矩阵
        // Eigen::Matrix<double, 6, 2> G;
        // G << 1 / 6.0 * pow(dt, 3),                    0,
        //                         0, 1 / 6.0 * pow(dt, 3), 
        //      1 / 2.0 * pow(dt, 2),                    0,
        //                         0, 1 / 2.0 * pow(dt, 2),
        //                        dt,                    0, 
        //                         0,                   dt;
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
            // Eigen::Matrix<double, 6, 2> G;
            // G << 0.5 * pow(dt, 2),                0,
            //                     0, 0.5 * pow(dt, 2),
            //                    dt,                0,
            //                     0,               dt,
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
    {}

    
} // armor_processor
