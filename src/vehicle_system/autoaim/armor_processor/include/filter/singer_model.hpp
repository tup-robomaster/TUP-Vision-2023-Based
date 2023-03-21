/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-11 12:52:40
 * @LastEditTime: 2023-03-11 21:59:34
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/singer_model.hpp
 */
#ifndef SINGER_MODEL_HPP_
#define SINGER_MODEL_HPP_

#include "./kalman_filter.hpp"
#include "../prediction/param_struct.hpp"

using namespace Eigen;
namespace armor_processor
{
    class SingerModel
    {
        MatrixXd F_;
        MatrixXd H_;
        MatrixXd C_;            
        MatrixXd P_;
        MatrixXd Q_;
        MatrixXd R_;
        
        vector<double> singer_param_;
    public:
        SingerModel();
        SingerModel(int SP, int MP, int CP);
        SingerModel(const vector<double> singer_param, int SP, int MP, int CP);
        ~SingerModel();

        void init();
        
        MatrixXd F() const { return this->F_; }
        MatrixXd H() const { return this->H_; }
        MatrixXd C() const { return this->C_; }
        MatrixXd P() const { return this->P_; }
        MatrixXd Q() const { return this->Q_; }
        MatrixXd R() const { return this->R_; }

        void setF(MatrixXd& F, const double& dt, const double& alpha);
        void setH(MatrixXd& H, const double& coeff);
        void setC(MatrixXd& C, const double& dt, const double& alpha);
        void setP(MatrixXd& P, const double& p);
        void setQ(MatrixXd& Q, const double& dt, const double& alpha, const double& sigma);
        void setR(MatrixXd& R, const double& cov);
    };
} //armor_processsor

#endif