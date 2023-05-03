/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:36:22
 * @LastEditTime: 2023-05-04 02:27:32
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
        radius_ = 0.20;
        dt_ = 0.015;
        kf_param_ = kf_param;
    }

    UniformModel::UniformModel()
    {
        radius_ = 0.20;
        dt_ = 0.015;
        kf_param_.process_noise_params = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        kf_param_.measure_noise_params = {1.0, 1.0, 1.0, 1.0};
        kf_param_.singer_params = {8.00, 10.0, 0.1, 0.8, 5.00, 0.0030, 1.0, 1.0, 7.0};
    }

    UniformModel::~UniformModel()
    {

    }

    // void UniformModel::init()
    // {
    //     double alpha = kf_param_.singer_params[0];
    //     double sigma = kf_param_.singer_params[5];

    //     this->x_.resize(9);
    //     this->P_.setIdentity(9, 9);
    //     this->F_.resize(9, 9);
    //     this->C_.resize(9, 3);
    //     this->z_.resize(4);
    //     this->H_.resize(4, 9);
    //     this->R_.resize(4, 4);
    //     double r[4] = {
    //         this->kf_param_.measure_noise_params[0],
    //         this->kf_param_.measure_noise_params[1],
    //         this->kf_param_.measure_noise_params[2],
    //         this->kf_param_.measure_noise_params[3],
    //     };
    //     this->R_ << r[0],    0,    0,    0,   
    //                 0, r[1],    0,    0,    
    //                 0,    0, r[2],    0,    
    //                 0,    0,    0, r[3];
    //     this->Q_.setIdentity(9, 9);
    //     double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
    //     double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
    //     double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
    //     double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
    //     double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
    //     double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
    //     this->Q_ << q[0], 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   ,  
    //                 0   , q[1], 0   , 0   , 0   , 0   , 0   , 0   , 0   ,    
    //                 0   , 0   , q[2], 0   , 0   , 0   , 0   , 0   , 0   ,  
    //                 0   , 0   , 0   , q[3], 0   , 0   , 0   , 0   , 0   , 
    //                 0   , 0   , 0   , 0   , q[4], 0   , 0   , 0   , 0   , 
    //                 0   , 0   , 0   , 0   , 0   , q[5], 0   , 0   , 0   , 
    //                 0   , 0   , 0   , 0   , 0   , 0   , q[6], 0   , 0   ,  
    //                 0   , 0   , 0   , 0   , 0   , 0   , 0   , q[7], 0   ,
    //                 0   , 0   , 0   , 0   , 0   , 0   , 0   , 0   , q[8];

    //     this->x_.resize(11);
    //     // this->dt_ = dt;
    //     this->P_.setIdentity(11, 11);
    //     this->F_.resize(11, 11);
    //     this->C_.resize(11, 3);
    //     /*
    //                 // Xc--Yc--Zc--r--theta-omega-vx--vy--vz----------------ax---------------ay-----------------az
    //                 Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
    //     */
    //     // this->F_ << 1,  0,  0,  0,  0,    dt,  0,  0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,                                                   0,
    //     //             0,  1,  0,  0,  0,     0, dt,  0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,
    //     //             0,  0,  1,  0,  0,     0,  0, dt,                                                   0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,
    //     //             0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //     //             0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //     //             0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,                                                   0,
    //     //             0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,
    //     //             0,  0,  0,  0,  0,     0,  0,  1,                                                   0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,
    //     //             0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt),                                                   0,                                                   0,
    //     //             0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt),                                                   0,
    //     //             0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                    exp(-alpha * dt);
        
    //     this->z_.resize(4);
    //     this->H_.resize(4, 11);
    //     this->R_.resize(4, 4);
    //     double r[4] = {
    //         this->kf_param_.measure_noise_params[0],
    //         this->kf_param_.measure_noise_params[1],
    //         this->kf_param_.measure_noise_params[2],
    //         this->kf_param_.measure_noise_params[3],
    //     };
    //     this->R_ << r[0],    0,    0,    0,   
    //                    0, r[1],    0,    0,    
    //                    0,    0, r[2],    0,    
    //                    0,    0,    0, r[3];

    //     this->Q_.setIdentity(11, 11);

    //     setKF(this->dt_);
    // }

    void UniformModel::init()
    {
        double alpha = kf_param_.singer_params[0];
        double sigma = kf_param_.singer_params[5];

        this->x_.resize(6);
        this->P_.setIdentity(6, 6);
        this->F_.resize(6, 6);
        this->C_.resize(0, 0);
        this->z_.resize(4);
        this->H_.resize(4, 6);
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
        this->Q_.setIdentity(6, 6);
        double q[6] = {
            kf_param_.process_noise_params[0], kf_param_.process_noise_params[1], 
            kf_param_.process_noise_params[2], kf_param_.process_noise_params[3], 
            kf_param_.process_noise_params[4], kf_param_.process_noise_params[5]
        };
        this->Q_ << q[0], 0   , 0   , 0   , 0   , 0   ,
                    0   , q[1], 0   , 0   , 0   , 0   ,   
                    0   , 0   , q[2], 0   , 0   , 0   ,
                    0   , 0   , 0   , q[3], 0   , 0   ,
                    0   , 0   , 0   , 0   , q[4], 0   ,
                    0   , 0   , 0   , 0   , 0   , q[5];
        updateF();
        updateJf();
        updateH();
        updateJh();
    }

    void UniformModel::updateF()
    {
        this->F_ << 1,  0,  0,  0,  0,     0,    
                    0,  1,  0,  0,  0,     0,     
                    0,  0,  1,  0,  0,     0,    
                    0,  0,  0,  1,  0,     0,     
                    0,  0,  0,  0,  1,   dt_,     
                    0,  0,  0,  0,  0,     1;
    }
    void UniformModel::updateF(Eigen::MatrixXd& Ft, double dt)
    {
        Ft << 1,  0,  0,  0,  0,     0,    
              0,  1,  0,  0,  0,     0,     
              0,  0,  1,  0,  0,     0,    
              0,  0,  0,  1,  0,     0,     
              0,  0,  0,  0,  1,    dt,     
              0,  0,  0,  0,  0,     1;
    }

    void UniformModel::updateH()
    {
        this->H_ << 1, 0, 0,  sin(x_(4)), 0,  0,
                    0, 1, 0, -cos(x_(4)), 0,  0,
                    0, 0, 1,           0, 0,  0,
                    0, 0, 0,           0, 1,  0; 
    }
    void UniformModel::updateH(Eigen::MatrixXd& Ht, double dt)
    {
        Ht << 1, 0, 0,  sin(x_(4)), 0,  0,
              0, 1, 0, -cos(x_(4)), 0,  0,
              0, 0, 1,           0, 0,  0,
              0, 0, 0,           0, 1,  0; 
    }
    
    void UniformModel::updateJf()
    {
        this->Jf_ << 1, 0, 0, 0, 0,   0,
                     0, 1, 0, 0, 0,   0,
                     0, 0, 1, 0, 0,   0,
                     0, 0, 0, 1, 0,   0,
                     0, 0, 0, 1, dt_, 0,
                     0, 0, 0, 0, 0,   1;
    }
    void UniformModel::updateJf(Eigen::MatrixXd& Jft, double dt)
    {
        Jft << 1, 0, 0, 0,  0, 0,
               0, 1, 0, 0,  0, 0,
               0, 0, 1, 0,  0, 0,
               0, 0, 0, 1,  0, 0,
               0, 0, 0, 1, dt, 0,
               0, 0, 0, 0,  0, 1;
    }

    void UniformModel::updateJh()
    {
        this->Jh_ << 1, 0, 0, sin(x_(4)) , x_(3) * cos(x_(4)),  0,
                     0, 1, 0, -cos(x_(4)), x_(3) * sin(x_(4)),  0,
                     0, 0, 1, 0          , 0                 ,  0,
                     0, 0, 0, 0          , 1                 ,  0; 
    }
    void UniformModel::updateJh(Eigen::MatrixXd& Jht, double dt)
    {
        Jht << 1, 0, 0, sin(x_(4)) , x_(3) * cos(x_(4)),  0,
               0, 1, 0, -cos(x_(4)), x_(3) * sin(x_(4)),  0,
               0, 0, 1, 0          , 0                 ,  0,
               0, 0, 0, 0          , 1                 ,  0; 
    }

    // void UniformModel::setF(Eigen::MatrixXd& Ft, const double& dt)
    // {
    //     double alpha = kf_param_.singer_params[0];
    //     double sigma = kf_param_.singer_params[5];
    //     /*
    //           Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
    //     */
    //     Ft << 1,  0,  0,  0,  0,    dt,  0,  0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,                                                   0,
    //           0,  1,  0,  0,  0,     0, dt,  0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,
    //           0,  0,  1,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //           0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //           0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //           0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,                                                   0,
    //           0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,
    //           0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //           0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt),                                                   0,                                                   0,
    //           0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt),                                                   0,
    //           0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0;
    // }

    // void UniformModel::setC(Eigen::MatrixXd& Ct, const double& dt)
    // {
    //     double alpha = kf_param_.singer_params[0];
    //     Ct << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * (dt)) / alpha)),                                                                         0,                                                                        0,
    //                                                                                     0, 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),                                                                        0,
    //                                                                                     0,                                                                        0,                                                                        0,
    //                                                                                     0,                                                                        0,                                                                        0,
    //                                                                                     0,                                                                        0,                                                                        0,
    //                                                   dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,                                                                        0,
    //                                                                                     0,                                      dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,
    //                                                                                     0,                                                                        0,                                                                        0,
    //                                                                  1 - exp(-alpha * dt),                                                                        0,                                                                        0,
    //                                                                                     0,                                                     1 - exp(-alpha * dt),                                                                        0,
    //                                                                                     0,                                                                        0,                                                                        0;
    // }

    // void UniformModel::setKF(double dt)
    // {
    //     this->dt_ = dt;
    //     double alpha = kf_param_.singer_params[0];
    //     double sigma = kf_param_.singer_params[5];
    //     /*
    //                 Xc--Yc--Zc--r--theta--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
    //     */
    //     this->F_ << 1,  0,  0,  0,  0,    dt,  0,  0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,                                                   0,
    //                 0,  1,  0,  0,  0,     0, dt,  0,                                                   0, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,                                                   0,
    //                 0,  0,  1,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //                 0,  0,  0,  1,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //                 0,  0,  0,  0,  1,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //                 0,  0,  0,  0,  0,     1,  0,  0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,                                                   0,
    //                 0,  0,  0,  0,  0,     0,  1,  0,                                                   0,                      (1 - exp(-alpha * dt)) / alpha,                                                   0,
    //                 0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0,
    //                 0,  0,  0,  0,  0,     0,  0,  0,                                    exp(-alpha * dt),                                                   0,                                                   0,
    //                 0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                    exp(-alpha * dt),                                                   0,
    //                 0,  0,  0,  0,  0,     0,  0,  0,                                                   0,                                                   0,                                                   0;

    //     double q[2] = {this->kf_param_.process_noise_params[0], this->kf_param_.process_noise_params[1]};
    //     double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
    //     double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
    //     double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
    //     double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
    //     double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
    //     double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
    //     this->Q_ << 2 * pow(sigma, 2) * alpha * q11,                               0,                               0,    0,    0,    2 * pow(sigma, 2) * alpha * q12,                                  0,                                  0,    2 * pow(sigma, 2) * alpha* q13,                                 0,                                  0,    
    //                                               0, 2 * pow(sigma, 2) * alpha * q11,                               0,    0,    0,                                  0,    2 * pow(sigma, 2) * alpha * q12,                                  0,                                 0,    2 * pow(sigma, 2) * alpha* q13,                                  0,    
    //                                               0,                               0,                               1,    0,    0,                                  0,                                  0,                                  1,                                 0,                                 0,                                  1,   
    //                                               0,                               0,                               0, q[0],    0,                                  0,                                  0,                                  0,                                 0,                                 0,                                  0,   
    //                                               0,                               0,                               0,    0, q[1],                                  0,                                  0,                                  0,                                 0,                                 0,                                  0,   
    //                 2 * pow(sigma, 2) * alpha * q12,                               0,                               0,    0,    0,    2 * pow(sigma, 2) * alpha * q22,                                  0,                                  0,    2 * pow(sigma, 2) * alpha* q23,                                 0,                                  0,   
    //                                               0, 2 * pow(sigma, 2) * alpha * q12,                               0,    0,    0,                                  0,    2 * pow(sigma, 2) * alpha * q22,                                  0,                                 0,    2 * pow(sigma, 2) * alpha* q23,                                  0,   
    //                                               0,                               0,                               1,    0,    0,                                  0,                                  0,                                  1,                                 0,                                 0,                                  1,  
    //                 2 * pow(sigma, 2) * alpha * q13,                               0,                               0,    0,    0,    2 * pow(sigma, 2) * alpha * q23,                                  0,                                  0,   2 * pow(sigma, 2) * alpha * q33,                                 0,                                  0,  
    //                                               0, 2 * pow(sigma, 2) * alpha * q13,                               0,    0,    0,                                  0,    2 * pow(sigma, 2) * alpha * q23,                                  0,                                 0,   2 * pow(sigma, 2) * alpha * q33,                                  0,  
    //                                               0,                               0,                               1,    0,    0,                                  0,                                  0,                                  1,                                 0,                                 0,                                  1;
        
    //     // dt /= 2;
    //     this->C_ << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * (dt)) / alpha)),                                                                      0,                                                                        0,
    //                                                                                        0, 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),                                                                        0,
    //                                                                                        0,                                                                        0,                                                                        0,
    //                                                                                        0,                                                                        0,                                                                        0,
    //                                                                                        0,                                                                        0,                                                                        0,
    //                                                      dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,                                                                        0,
    //                                                                                        0,                                      dt - (1 - exp(-alpha * dt) / alpha),                                                                        0,
    //                                                                                        0,                                                                        0,                                                                        0,
    //                                                                     1 - exp(-alpha * dt),                                                                        0,                                                                        0,
    //                                                                                        0,                                                     1 - exp(-alpha * dt),                                                                        0,
    //                                                                                        0,                                                                        0,                                                                        0;
        
    //     // this->H_ << 1, 0, 0, -sin(rangle_), 0, 0, 0, 0, 0, 0, 0,
    //     //             0, 1, 0,  cos(rangle_), 0, 0, 0, 0, 0, 0, 0,
    //     //             0, 0, 1,             0, 0, 0, 0, 0, 0, 0, 0,
    //     //             0, 0, 0,             0, 1, 0, 0, 0, 0, 0, 0;
    // }

    // void UniformModel::setKF(double dt)
    // {
    //     this->dt_ = dt;
    //     double alpha = kf_param_.singer_params[0];
    //     double sigma = kf_param_.singer_params[5];
    //     /*
    //                 Xc--Yc--Zc--r--theta--omega--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
    //     */
    //     this->F_ << 1,  0,  0,  0,  0,     0,    
    //                 0,  1,  0,  0,  0,     0,     
    //                 0,  0,  1,  0,  0,     0,    
    //                 0,  0,  0,  1,  0,     0,     
    //                 0,  0,  0,  0,  1,    dt,     
    //                 0,  0,  0,  0,  0,     1;

    //     // this->F_ << 1,  0,  0,  0,  0,     0,    dt,  0,  0,
    //     //             0,  1,  0,  0,  0,     0,     0, dt,  0,
    //     //             0,  0,  1,  0,  0,     0,     0,  0, dt,
    //     //             0,  0,  0,  1,  0,     0,     0,  0,  0,
    //     //             0,  0,  0,  0,  1,    dt,     0,  0,  0,
    //     //             0,  0,  0,  0,  0,     1,     0,  0,  0,
    //     //             0,  0,  0,  0,  0,     0,     1,  0,  0,
    //     //             0,  0,  0,  0,  0,     0,     0,  1,  0,
    //     //             0,  0,  0,  0,  0,     0,     0,  0,  1;
    // }

    // void UniformModel::setF(Eigen::MatrixXd& Ft, const double& pred_dt)
    // {
    //     double alpha = kf_param_.singer_params[0];
    //     double sigma = kf_param_.singer_params[5];
    //     /*
    //                 Xc--Yc--Zc--r--theta--omega--vx--vy--vz------------------------ax-------------------------------------------------------------------------------ay---------------------------------------------------az
    //     */
    //     Ft << 1,  0,  0,  0,  0,      0,    
    //                 0,  1,  0,  0,  0,      0,     
    //                 0,  0,  1,  0,  0,      0,    
    //                 0,  0,  0,  1,  0,      0,     
    //                 0,  0,  0,  0,  1,pred_dt,     
    //                 0,  0,  0,  0,  0,      1;

    //     // this->F_ << 1,  0,  0,  0,  0,     0,    dt,  0,  0,
    //     //             0,  1,  0,  0,  0,     0,     0, dt,  0,
    //     //             0,  0,  1,  0,  0,     0,     0,  0, dt,
    //     //             0,  0,  0,  1,  0,     0,     0,  0,  0,
    //     //             0,  0,  0,  0,  1,    dt,     0,  0,  0,
    //     //             0,  0,  0,  0,  0,     1,     0,  0,  0,
    //     //             0,  0,  0,  0,  0,     0,     1,  0,  0,
    //     //             0,  0,  0,  0,  0,     0,     0,  1,  0,
    //     //             0,  0,  0,  0,  0,     0,     0,  0,  1;
    // }
} // armor_processor
