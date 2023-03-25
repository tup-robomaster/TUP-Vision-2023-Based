/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-11 13:18:53
 * @LastEditTime: 2023-03-22 02:23:37
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/filter/singer_model.cpp
 */
#include "../../include/filter/singer_model.hpp"

namespace armor_processor
{
    SingerModel::SingerModel(const vector<double> singer_param, int SP, int MP, int CP)
    {
        F_ = Eigen::MatrixXd::Identity(SP, SP);
        H_ = Eigen::MatrixXd::Zero(MP, SP);
        C_ = Eigen::MatrixXd::Zero(SP, CP);            
        P_ = Eigen::MatrixXd::Zero(SP, SP);
        Q_ = Eigen::MatrixXd::Identity(SP, SP);
        R_ = Eigen::MatrixXd::Zero(MP, MP);
        singer_param_ = singer_param;
        init();
    }
    
    SingerModel::SingerModel()
    {

    }

    SingerModel::SingerModel(int SP, int MP, int CP)
    {
        F_ = Eigen::MatrixXd::Identity(SP, SP);
        H_ = Eigen::MatrixXd::Zero(MP, SP);
        C_ = Eigen::MatrixXd::Zero(SP, CP);            
        P_ = Eigen::MatrixXd::Zero(SP, SP);
        Q_ = Eigen::MatrixXd::Identity(SP, SP);
        R_ = Eigen::MatrixXd::Zero(MP, MP);
        singer_param_ = {0.80, 5.0, 0.10, 0.80, 0.80, 0.20, 1.0, 1.0, 5.0};
        init();
    }

    SingerModel::~SingerModel()
    {
    }

    void SingerModel::init()
    {
        cout << "singer_param:" <<  singer_param_[0] << " " << singer_param_[1] << " " << singer_param_[2] << " " <<  singer_param_[3]
            << " " <<  singer_param_[4] << " " <<  singer_param_[5] << " " <<  singer_param_[6] << " " <<  singer_param_[7] << endl;
        double alpha = singer_param_[0];
        double dt = singer_param_[4];

        F_ << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  
                            0, 1, (1 - exp(-alpha * dt)) / alpha,
		                    0, 0, exp(-alpha * dt);
        H_ << 1, 0, 0;
        C_ << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
                            dt - (1 - exp(-alpha * dt) / alpha),
                            1 - exp(-alpha * dt);
        double p = singer_param_[6];
        P_ << p, 0, 0,
            0, p, 0,
            0, 0, p;
        double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
        double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
        double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
        double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
        double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
        double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
        double sigma = singer_param_[5];
        Q_ << 2 * pow(sigma, 2) * alpha * q11, 2 * pow(sigma, 2) * alpha * q12, 2 * pow(sigma, 2) * alpha* q13,
                            2 * pow(sigma, 2) * alpha* q12, 2 * pow(sigma, 2) * alpha* q22, 2 * pow(sigma, 2) * alpha* q23,
		                    2 * pow(sigma, 2) * alpha* q13, 2 * pow(sigma, 2) * alpha* q23, 2 * pow(sigma, 2) * alpha* q33;
        double meaCov = singer_param_[7];
        R_ << meaCov;
    }

    void SingerModel::setF(MatrixXd& F, const double& dt, const double& alpha)
    {
        F << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  
            0, 1, (1 - exp(-alpha * dt)) / alpha,
            0, 0, exp(-alpha * dt);
    }

    void SingerModel::setH(MatrixXd& H, const double& coeff)
    {
        H << coeff, 0, 0;
        return;
    }

    void SingerModel::setC(MatrixXd& C, const double& dt, const double& alpha)
    {
        C << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)),
            dt - (1 - exp(-alpha * dt) / alpha),
            1 - exp(-alpha * dt);
        return;
    }

    void SingerModel::setP(MatrixXd& P, const double& p)
    {
        P <<  p, 0, 0,
              0, p, 0,
              0, 0, p;
        return;
    }

    void SingerModel::setQ(MatrixXd& Q, const double& dt, const double& alpha, const double& acc)
    {
        double q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
        double q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
        double q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
        double q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
        double q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
        double q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
        
        double sigma = singer_param_[5];
        // if(acc > 0)
        // {
        //     sigma = ((4 - M_PI) / M_PI) * pow(singer_param_[1] - acc, 2);
        // }
        // else
        // {
        //     sigma = ((4 - M_PI) / M_PI) * pow(singer_param_[1] + acc, 2);
        // }
        Q << 2 * pow(sigma, 2) * alpha * q11, 2 * pow(sigma, 2) * alpha * q12, 2 * pow(sigma, 2) * alpha* q13,
            2 * pow(sigma, 2) * alpha* q12, 2 * pow(sigma, 2) * alpha* q22, 2 * pow(sigma, 2) * alpha* q23,
            2 * pow(sigma, 2) * alpha* q13, 2 * pow(sigma, 2) * alpha* q23, 2 * pow(sigma, 2) * alpha* q33;
        return;
    }

    void SingerModel::setR(MatrixXd& R, const double& cov)
    {
        R << cov;
        return;
    }
} //armor_processor