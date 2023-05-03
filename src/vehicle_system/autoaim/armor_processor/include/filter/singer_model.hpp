/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-11 12:52:40
 * @LastEditTime: 2023-05-04 02:29:29
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/singer_model.hpp
 */
#ifndef SINGER_MODEL_HPP_
#define SINGER_MODEL_HPP_

#include "./kalman_filter.hpp"
#include "../prediction/param_struct.hpp"

using namespace Eigen;
namespace armor_processor
{
    class SingerModel : public KalmanFilter
    {
    public:
        void updateF();
        void updateF(Eigen::MatrixXd& Ft, double dt);
        void updateH();
        void updateH(Eigen::MatrixXd& Ht, double dt);
        void updateJf();
        void updateJf(Eigen::MatrixXd& Jft, double dt);
        void updateJh();
        void updateJh(Eigen::MatrixXd& Jht, double dt);
    
    public:
        SingerModel();
        SingerModel(int SP, int MP, int CP);
        SingerModel(const vector<double> singer_param, int SP, int MP, int CP);
        virtual SingerModel* Clone() { return new SingerModel(*this); } 
        ~SingerModel();

        void init();
        vector<double> singer_param_;
    
    public:
        void setF(MatrixXd& F, const double& dt, const double& alpha);
        void setH(MatrixXd& H, const double& coeff);
        void setC(MatrixXd& C, const double& dt, const double& alpha);
        void setP(MatrixXd& P, const double& p);
        void setR(MatrixXd& R, const double& cov);
        void setQ(const double acc);
        void setQ(const double& dt, const double& alpha, const double& acc);
        void setQ(const double& dt, const double& alpha, const double& acc, MatrixXd* Q = nullptr);
    };
} //armor_processsor

#endif