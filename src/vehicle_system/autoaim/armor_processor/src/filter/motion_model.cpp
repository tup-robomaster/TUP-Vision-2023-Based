/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:36:22
 * @LastEditTime: 2023-04-25 18:26:53
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
        kf_param_.measure_noise_params = {1, 1, 1, 1};
        kf_param_.process_noise_params = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    }

    UniformModel::~UniformModel()
    {

    }

    void UniformModel::init(const Eigen::VectorXd& x, const double& dt)
    {
        assert(x.size() == 11);
        this->x_ = x;
        this->dt_ = dt;
        this->P_.setIdentity(11, 11);
        this->F_.resize(11, 11);
        /*
                    // Xc--Yc--Zc--r--theta-omega-vx--vy--vz----------------ax---------------ay-----------------az
                    Xc--Yc--Zc--r--theta--vx--vy--vz----------------ax---------------ay-----------------az
        */
        this->F_ << 1,  0,  0,  0,  0,    dt,  0,  0, 0.5 * pow(dt, 2),                0,                0,
                    0,  1,  0,  0,  0,     0, dt,  0,                0, 0.5 * pow(dt, 2),                0,
                    0,  0,  1,  0,  0,     0,  0, dt,                0,                0, 0.5 * pow(dt, 2),
                    0,  0,  0,  1,  0,     0,  0,  0,                0,                0,                0,
                    0,  0,  0,  0,  1,     0,  0,  0,                0,                0,                0,
                    0,  0,  0,  0,  0,     1,  0,  0,               dt,                0,                0,
                    0,  0,  0,  0,  0,     0,  1,  0,                0,                dt,               0,
                    0,  0,  0,  0,  0,     0,  0,  1,                0,                0,               dt,
                    0,  0,  0,  0,  0,     0,  0,  0,                1,                0,                0,
                    0,  0,  0,  0,  0,     0,  0,  0,                0,                1,                0,
                    0,  0,  0,  0,  0,     0,  0,  0,                0,                0,                1;
        this->z_.resize(4);
        this->H_.resize(4, 11);
        this->H_ << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;

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
        double q[11] = {
            this->kf_param_.process_noise_params[0],  this->kf_param_.process_noise_params[1], this->kf_param_.process_noise_params[2],
            this->kf_param_.process_noise_params[3],  this->kf_param_.process_noise_params[4], this->kf_param_.process_noise_params[5],
            this->kf_param_.process_noise_params[6],  this->kf_param_.process_noise_params[7], this->kf_param_.process_noise_params[8],
            this->kf_param_.process_noise_params[9], this->kf_param_.process_noise_params[10]};
        this->Q_ << q[0],    0,    0,    0,    0,    0,    0,    0,    0,    0,     0,    
                       0, q[1],    0,    0,    0,    0,    0,    0,    0,    0,     0,    
                       0,    0, q[2],    0,    0,    0,    0,    0,    0,    0,     0,   
                       0,    0,    0, q[3],    0,    0,    0,    0,    0,    0,     0,   
                       0,    0,    0,    0, q[4],    0,    0,    0,    0,    0,     0,   
                       0,    0,    0,    0,    0, q[5],    0,    0,    0,    0,     0,   
                       0,    0,    0,    0,    0,    0, q[6],    0,    0,    0,     0,   
                       0,    0,    0,    0,    0,    0,    0, q[7],    0,    0,     0,  
                       0,    0,    0,    0,    0,    0,    0,    0, q[8],    0,     0,  
                       0,    0,    0,    0,    0,    0,    0,    0,    0, q[9],     0,  
                       0,    0,    0,    0,    0,    0,    0,    0,    0,    0, q[10]; 
    }

    void UniformModel::setF(Eigen::MatrixXd& Ft, const double& dt)
    {
        //--------- Xc--Yc--Zc--r--theta--vx--vy--vz----------------ax---------------ay-----------------az
        Ft << 1,  0,  0,  0,  0,    dt,  0,  0, 0.5 * pow(dt, 2),                0,                0,
             0,  1,  0,  0,  0,     0, dt,  0,                0, 0.5 * pow(dt, 2),                0,
             0,  0,  1,  0,  0,     0,  0, dt,                0,                0, 0.5 * pow(dt, 2),
             0,  0,  0,  1,  0,     0,  0,  0,                0,                0,                0,
             0,  0,  0,  0,  1,     0,  0,  0,                0,                0,                0,
             0,  0,  0,  0,  0,     1,  0,  0,               dt,                0,                0,
             0,  0,  0,  0,  0,     0,  1,  0,                0,                dt,               0,
             0,  0,  0,  0,  0,     0,  0,  1,                0,                0,               dt,
             0,  0,  0,  0,  0,     0,  0,  0,                1,                0,                0,
             0,  0,  0,  0,  0,     0,  0,  0,                0,                1,                0,
             0,  0,  0,  0,  0,     0,  0,  0,                0,                0,                1;
    }

    void UniformModel::updatePrediction()
    {
        this->x_ = this->F_ * this->x_;
    }

    void UniformModel::updateMeasurement()
    {
        
    }
} // armor_processor
