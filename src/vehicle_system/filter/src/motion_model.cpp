/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:36:22
 * @LastEditTime: 2023-05-29 22:23:00
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/src/motion_model.cpp
 */
#include "../include/motion_model.hpp"

namespace filter
{
    CV::CV()
    {
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

    SingerModel::SingerModel(const vector<double>* ekf_param, int SP, int MP, int CP)
    {
        Init(SP, MP, CP);
        kf_param_.process_noise_params = ekf_param[0];
        kf_param_.measure_noise_params = ekf_param[1];
        cout << "singer model: alpha:" << ekf_param[0][0] << " r:" << ekf_param[1][0] << endl;
        init();
    }
    
    SingerModel::SingerModel()
    {
    }

    SingerModel::SingerModel(int SP, int MP, int CP)
    {
        Init(SP, MP, CP);
        if (cp_ == 1)
        {
            kf_param_.process_noise_params = {8.0, 0.0030};
            kf_param_.measure_noise_params = {1.0};
        }
        else if (cp_ == 3)
        {
            kf_param_.process_noise_params = {20.0, 8.0, 8.0, 0.0025, 0.0030, 0.0030};
            kf_param_.measure_noise_params = {1.0, 1.0, 1.0};
        }
        init();
    }

    SingerModel::~SingerModel()
    {
    }

    void SingerModel::updateF()
    {
        assert(cp_ == 1 || cp_ == 3);
        if (cp_ == 1)
        {
            double alpha = kf_param_.process_noise_params[0];
            double dt = dt_;
            F_ << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  
                  0, 1,  (1 - exp(-alpha * dt)) / alpha,
                  0, 0,  exp(-alpha * dt);
        }
        else if (cp_ == 3)
        {
            double alpha_x = kf_param_.process_noise_params[0];
            double alpha_y = kf_param_.process_noise_params[1];
            double alpha_z = kf_param_.process_noise_params[2];
            double dt = dt_;
            // cout << "alpha:" << alpha_x << " " << alpha_y << " " << alpha_z << " dt" << dt << endl;

            F_ << 1, 0, 0, dt, 0, 0, (alpha_x * dt - 1 + exp(-alpha_x * dt)) / alpha_x / alpha_x, 0, 0, 
                  0, 1, 0, 0, dt, 0, 0, (alpha_y * dt - 1 + exp(-alpha_y * dt)) / alpha_y / alpha_y, 0, 
                  0, 0, 1, 0, 0, dt, 0, 0, (alpha_z * dt - 1 + exp(-alpha_z * dt)) / alpha_z / alpha_z, 
                  0, 0, 0, 1, 0, 0, (1 - exp(-alpha_x * dt)) / alpha_x, 0, 0,
                  0, 0, 0, 0, 1, 0, 0, (1 - exp(-alpha_y * dt)) / alpha_y, 0,
                  0, 0, 0, 0, 0, 1, 0, 0, (1 - exp(-alpha_z * dt)) / alpha_z,
                  0, 0, 0, 0, 0, 0, exp(-alpha_x * dt), 0, 0,
                  0, 0, 0, 0, 0, 0, 0, exp(-alpha_y * dt), 0,
                  0, 0, 0, 0, 0, 0, 0, 0, exp(-alpha_z * dt);
        }
    }

    void SingerModel::updateF(Eigen::MatrixXd& Ft, double pred_dt)
    {
        assert(cp_ == 1 || cp_ == 3);

        if (cp_ == 1)
        {
            double alpha = kf_param_.process_noise_params[0];
            double dt = pred_dt;
            Ft << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  
                0, 1, (1 - exp(-alpha * dt)) / alpha,
                0, 0, exp(-alpha * dt);
        }
        else if (cp_ == 3)
        {
            double alpha_x = kf_param_.process_noise_params[0];
            double alpha_y = kf_param_.process_noise_params[1];
            double alpha_z = kf_param_.process_noise_params[2];
            double dt = pred_dt;
            // cout << "alpha:" << alpha_x << " " << alpha_y << " " << alpha_z << " pred_dt" << pred_dt << endl;
            
            Ft << 1, 0, 0, dt, 0, 0, (alpha_x * dt - 1 + exp(-alpha_x * dt)) / alpha_x / alpha_x, 0, 0, 
                  0, 1, 0, 0, dt, 0, 0, (alpha_y * dt - 1 + exp(-alpha_y * dt)) / alpha_y / alpha_y, 0, 
                  0, 0, 1, 0, 0, dt, 0, 0, (alpha_z * dt - 1 + exp(-alpha_z * dt)) / alpha_z / alpha_z, 
                  0, 0, 0, 1, 0, 0, (1 - exp(-alpha_x * dt)) / alpha_x, 0, 0,
                  0, 0, 0, 0, 1, 0, 0, (1 - exp(-alpha_y * dt)) / alpha_y, 0,
                  0, 0, 0, 0, 0, 1, 0, 0, (1 - exp(-alpha_z * dt)) / alpha_z,
                  0, 0, 0, 0, 0, 0, exp(-alpha_x * dt), 0, 0,
                  0, 0, 0, 0, 0, 0, 0, exp(-alpha_y * dt), 0,
                  0, 0, 0, 0, 0, 0, 0, 0, exp(-alpha_z * dt);
        }
    }

    void SingerModel::updateH()
    {
        assert(cp_ == 1 || cp_ == 3);
        if (cp_ == 1)
        {
            H_ << 1, 0, 0;
        }
        else if (cp_ == 3)
        {
            H_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0, 0, 0, 0;
        }
    }

    void SingerModel::updateH(Eigen::MatrixXd& Ht, double dt)
    {
        Ht = H_;
    }

    void SingerModel::updateJf()
    {
        Jf_ = F_;
    }

    void SingerModel::updateJf(Eigen::MatrixXd& Jft, double dt)
    {
        Jft = Jf_;
    }

    void SingerModel::updateJh()
    {
        Jh_ = H_;
    }

    void SingerModel::updateJh(Eigen::MatrixXd& Jht, double dt)
    {
        Jht = Jh_;
    }

    void SingerModel::init()
    {
        updateF();
        updateC();
        updateH();
        updateQ();
        
        if (cp_ == 1)
        {
            double r = kf_param_.measure_noise_params[0];
            R_ << r;
        }
        else if (cp_ == 3)
        {
            double r1 = kf_param_.measure_noise_params[0];
            double r2 = kf_param_.measure_noise_params[1];
            double r3 = kf_param_.measure_noise_params[2];
            R_ << r1, 0,  0, 
                  0,  r2, 0,
                  0,  0,  r3;
        }
    }

    void SingerModel::updateC()
    {
        assert(cp_ == 1 || cp_ == 3);
        if (cp_ == 1)
        {
            double alpha = kf_param_.process_noise_params[0];
            double dt = dt_;
            C_ << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                dt - (1 - exp(-alpha * dt) / alpha),
                1 - exp(-alpha * dt);
        }
        else if (cp_ == 3)
        {
            double alpha_x = kf_param_.process_noise_params[0];
            double alpha_y = kf_param_.process_noise_params[1];
            double alpha_z = kf_param_.process_noise_params[2];
            double dt = dt_;
            C_ << 1 / alpha_x * (-dt + alpha_x * dt * dt / 2 + (1 - exp(-alpha_x * dt) / alpha_x)), 0, 0,
                  0, 1 / alpha_y * (-dt + alpha_y * dt * dt / 2 + (1 - exp(-alpha_y * dt) / alpha_y)), 0,
                  0, 0, 1 / alpha_z * (-dt + alpha_z * dt * dt / 2 + (1 - exp(-alpha_z * dt) / alpha_z)),
                  dt - (1 - exp(-alpha_x * dt) / alpha_x), 0, 0,
                  0, dt - (1 - exp(-alpha_y * dt) / alpha_y), 0,
                  0, 0, dt - (1 - exp(-alpha_z * dt) / alpha_z),
                  1 - exp(-alpha_x * dt), 0, 0,
                  0, 1 - exp(-alpha_y * dt), 0,
                  0, 0, 1 - exp(-alpha_z * dt);
        }
        return;
    }

    void SingerModel::updateC(MatrixXd& C, const double pred_dt)
    {
        assert(cp_ == 1 || cp_ == 3);
        if (cp_ == 1)
        {
            double alpha = kf_param_.process_noise_params[0];
            double dt = pred_dt;
            C << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                dt - (1 - exp(-alpha * dt) / alpha),
                1 - exp(-alpha * dt);
        }
        else if (cp_ == 3)
        {
            double alpha_x = kf_param_.process_noise_params[0];
            double alpha_y = kf_param_.process_noise_params[1];
            double alpha_z = kf_param_.process_noise_params[2];
            double dt = pred_dt;
            C << 1 / alpha_x * (-dt + alpha_x * dt * dt / 2 + (1 - exp(-alpha_x * dt) / alpha_x)), 0, 0,
                  0, 1 / alpha_y * (-dt + alpha_y * dt * dt / 2 + (1 - exp(-alpha_y * dt) / alpha_y)), 0,
                  0, 0, 1 / alpha_z * (-dt + alpha_z * dt * dt / 2 + (1 - exp(-alpha_z * dt) / alpha_z)),
                  dt - (1 - exp(-alpha_x * dt) / alpha_x), 0, 0,
                  0, dt - (1 - exp(-alpha_y * dt) / alpha_y), 0,
                  0, 0, dt - (1 - exp(-alpha_z * dt) / alpha_z),
                  1 - exp(-alpha_x * dt), 0, 0,
                  0, 1 - exp(-alpha_y * dt), 0,
                  0, 0, 1 - exp(-alpha_z * dt);
        }
        return;
    }

    void SingerModel::updateQ()
    {
        updateQ(dt_);
    }

    void SingerModel::updateQ(double dt)
    {
        assert(cp_ == 1 || cp_ == 3);
        if (cp_ == 1)
        {
            double alpha = kf_param_.process_noise_params[0];
            double sigma = kf_param_.process_noise_params[1];
            double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
            double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
            double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
            double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
            double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
            double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
            Q_ << 2 * pow(sigma, 2) * alpha * q11, 2 * pow(sigma, 2) * alpha * q12, 2 * pow(sigma, 2) * alpha* q13,
                  2 * pow(sigma, 2) * alpha* q12, 2 * pow(sigma, 2) * alpha* q22, 2 * pow(sigma, 2) * alpha* q23,
                  2 * pow(sigma, 2) * alpha* q13, 2 * pow(sigma, 2) * alpha * q23, 2 * pow(sigma, 2) * alpha * q33;
        }
        else if (cp_ == 3)
        {
            double alpha_x = kf_param_.process_noise_params[0];
            double alpha_y = kf_param_.process_noise_params[1];
            double alpha_z = kf_param_.process_noise_params[2];
            double sigma_x = kf_param_.process_noise_params[3];
            double sigma_y = kf_param_.process_noise_params[4];
            double sigma_z = kf_param_.process_noise_params[5];
            double x_q11 = 2 * pow(sigma_x, 2) * alpha_x / (2 * pow(alpha_x, 5)) * (1 - exp(-2 * alpha_x * dt) + 2 * alpha_x * dt + 2 * pow(alpha_x * dt, 3) / 3 - 2 * pow(alpha_x * dt, 2) - 4 * alpha_x * dt * exp(-alpha_x * dt));
            double x_q12 = 2 * pow(sigma_x, 2) * alpha_x / (2 * pow(alpha_x, 4)) * (exp(-2 * alpha_x * dt) + 1 - 2 * exp(-alpha_x * dt) + 2 * alpha_x * dt * exp(-alpha_x * dt) - 2 * alpha_x * dt + pow(alpha_x * dt, 2));
            double x_q13 = 2 * pow(sigma_x, 2) * alpha_x / (2 * pow(alpha_x, 3)) * (1 - exp(-2 * alpha_x * dt) - 2 * alpha_x * dt * exp(-alpha_x * dt));
            double x_q22 = 2 * pow(sigma_x, 2) * alpha_x / (2 * pow(alpha_x, 3)) * (4 * exp(-alpha_x * dt) - 3 - exp(-2 * alpha_x * dt) + 2 * alpha_x * dt);
            double x_q23 = 2 * pow(sigma_x, 2) * alpha_x / (2 * pow(alpha_x, 2)) * (exp(-2 * alpha_x * dt) + 1 - 2 * exp(-alpha_x * dt));
            double x_q33 = 2 * pow(sigma_x, 2) * alpha_x / (2 * alpha_x) * (1 - exp(-2 * alpha_x * dt));
            double y_q11 = 2 * pow(sigma_y, 2) * alpha_y / (2 * pow(alpha_y, 5)) * (1 - exp(-2 * alpha_y * dt) + 2 * alpha_y * dt + 2 * pow(alpha_y * dt, 3) / 3 - 2 * pow(alpha_y * dt, 2) - 4 * alpha_y * dt * exp(-alpha_y * dt));
            double y_q12 = 2 * pow(sigma_y, 2) * alpha_y / (2 * pow(alpha_y, 4)) * (exp(-2 * alpha_y * dt) + 1 - 2 * exp(-alpha_y * dt) + 2 * alpha_y * dt * exp(-alpha_y * dt) - 2 * alpha_y * dt + pow(alpha_y * dt, 2));
            double y_q13 = 2 * pow(sigma_y, 2) * alpha_y / (2 * pow(alpha_y, 3)) * (1 - exp(-2 * alpha_y * dt) - 2 * alpha_y * dt * exp(-alpha_y * dt));
            double y_q22 = 2 * pow(sigma_y, 2) * alpha_y / (2 * pow(alpha_y, 3)) * (4 * exp(-alpha_y * dt) - 3 - exp(-2 * alpha_y * dt) + 2 * alpha_y * dt);
            double y_q23 = 2 * pow(sigma_y, 2) * alpha_y / (2 * pow(alpha_y, 2)) * (exp(-2 * alpha_y * dt) + 1 - 2 * exp(-alpha_y * dt));
            double y_q33 = 2 * pow(sigma_y, 2) * alpha_y / (2 * alpha_y) * (1 - exp(-2 * alpha_y * dt));
            double z_q11 = 2 * pow(sigma_z, 2) * alpha_z / (2 * pow(alpha_z, 5)) * (1 - exp(-2 * alpha_z * dt) + 2 * alpha_z * dt + 2 * pow(alpha_z * dt, 3) / 3 - 2 * pow(alpha_z * dt, 2) - 4 * alpha_z * dt * exp(-alpha_z * dt));
            double z_q12 = 2 * pow(sigma_z, 2) * alpha_z / (2 * pow(alpha_z, 4)) * (exp(-2 * alpha_z * dt) + 1 - 2 * exp(-alpha_z * dt) + 2 * alpha_z * dt * exp(-alpha_z * dt) - 2 * alpha_z * dt + pow(alpha_z * dt, 2));
            double z_q13 = 2 * pow(sigma_z, 2) * alpha_z / (2 * pow(alpha_z, 3)) * (1 - exp(-2 * alpha_z * dt) - 2 * alpha_z * dt * exp(-alpha_z * dt));
            double z_q22 = 2 * pow(sigma_z, 2) * alpha_z / (2 * pow(alpha_z, 3)) * (4 * exp(-alpha_z * dt) - 3 - exp(-2 * alpha_z * dt) + 2 * alpha_z * dt);
            double z_q23 = 2 * pow(sigma_z, 2) * alpha_z / (2 * pow(alpha_z, 2)) * (exp(-2 * alpha_z * dt) + 1 - 2 * exp(-alpha_z * dt));
            double z_q33 = 2 * pow(sigma_z, 2) * alpha_z / (2 * alpha_z) * (1 - exp(-2 * alpha_z * dt));
            Q_ << x_q11, 0,     0,     x_q12, 0,     0,     x_q13, 0,     0,
                0,     y_q11, 0,     0,     y_q12, 0,     0,     y_q13, 0,   
                0,     0,     z_q11, 0,     0,     z_q12, 0,     0,     z_q13, 
                x_q12, 0,     0,     x_q22, 0,     0,     x_q23, 0,     0,
                0,     y_q12, 0,     0,     y_q22, 0,     0,     y_q23, 0,   
                0,     0,     z_q12, 0,     0,     z_q22, 0,     0,     z_q23,
                x_q13, 0,     0,     x_q23, 0,     0,     x_q33, 0,     0,
                0,     y_q13, 0,     0,     y_q23, 0,     0,     y_q33, 0,   
                0,     0,     z_q13, 0,     0,     z_q23, 0,     0,     z_q33;
        }
    }
    
    UniformModel::UniformModel()
    {

    }

    UniformModel::UniformModel(int SP, int MP, int CP)
    {
        radius_ = 0.20;
        dt_ = 0.015;
        kf_param_.process_noise_params = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        kf_param_.measure_noise_params = {1.0, 1.0, 1.0, 1.0};
        Init(SP, MP, CP);
    }
    
    UniformModel::UniformModel(const vector<double>* kf_param, int SP, int MP, int CP)
    {
        radius_ = 0.20;
        dt_ = 0.015;
        kf_param_.process_noise_params = kf_param[0];
        kf_param_.measure_noise_params = kf_param[1];
        cout << "uniform_model: p1:" << kf_param[0].at(0) << " r1:" << kf_param[1].at(0) << endl; 
        Init(SP, MP, CP);
    }

    UniformModel::~UniformModel()
    {

    }

    void UniformModel::init()
    {
        // double alpha = kf_param_.singer_params[0];
        // double sigma = kf_param_.singer_params[5];
        this->cp_ = 0;

        this->x_.resize(6);
        this->P_.setIdentity(6, 6);
        this->F_.resize(6, 6);
        this->Jf_.resize(6, 6);
        this->C_.resize(3, 1);
        this->z_.resize(4);
        this->H_.resize(4, 6);
        this->Jh_.resize(4, 6);
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
                     0, 0, 0, 0, 1, dt_,
                     0, 0, 0, 0, 0,   1;
    }
    void UniformModel::updateJf(Eigen::MatrixXd& Jft, double dt)
    {
        Jft << 1, 0, 0, 0,  0, 0,
               0, 1, 0, 0,  0, 0,
               0, 0, 1, 0,  0, 0,
               0, 0, 0, 1,  0, 0,
               0, 0, 0, 0,  1, dt,
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
} // filter
