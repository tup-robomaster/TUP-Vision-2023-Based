/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:00:19
 * @LastEditTime: 2023-05-29 22:24:53
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/kalman_filter.hpp
 */
#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <cassert>

using namespace std;
using namespace Eigen;
namespace filter
{
    struct KFParam
    {
        vector<double> process_noise_params;
        vector<double> measure_noise_params;
    };
    
    class KalmanFilter
    {
        public:
            KalmanFilter();
            KalmanFilter(KFParam kf_param);
            virtual KalmanFilter* Clone()
            {
                return new KalmanFilter(*this);
            }
            ~KalmanFilter();

            /**
             * @brief Initialize matrix dimension
             * 
             */
            void Init(int stateParams, int measureParams, int controlParams);

            /**
             * @brief 预测状态向量和协方差矩阵
             * 
             */
            void Predict();
            void Predict(const double& dt);

            /**
             * @brief 更新状态转移矩阵 
             * 
             */        
            virtual void updateF() {}
            virtual void updateF(Eigen::MatrixXd& Ft, double dt) 
            {
                Ft = F_;
            }
            
            /**
             * @brief 更新量测矩阵
            */
            virtual void updateH() {}
            virtual void updateH(Eigen::MatrixXd& Ht, double dt)
            {
                Ht = H_; 
            }

            /**
             * @brief 更新状态向量
             * 
             */
            void Update(const Eigen::VectorXd& z);
            void Update(const Eigen::VectorXd& z, int mp);
            void updateOnce(const double& dt, const Eigen::VectorXd* z);

            /**
             * @brief 更新状态向量（EKF）
             * 
             */
            void UpdateEKF(const Eigen::MatrixXd& z);

            /**
             * @brief 系统模型的雅可比矩阵
             * 
             */
            virtual void updateJf()
            {
                this->Jf_ = this->F_;
            }
            virtual void updateJf(Eigen::MatrixXd& Jft, double dt)
            {
                Jft = Jf_;
            }

            /**
             * @brief 量测模型的雅可比矩阵
            */
            virtual void updateJh()
            {
                this->Jh_ = this->H_;
            }
            virtual void updateJh(Eigen::MatrixXd& Jht, double dt)
            {
                Jht = Jh_;
            }

            /**
             * @brief 返回模型似然值
             * 
             */
            double getLikelihoodValue() const;
            
            Eigen::VectorXd x() const { return this->x_; };
            Eigen::MatrixXd P() const { return this->P_; };
            Eigen::MatrixXd S() const { return this->S_; };
            Eigen::MatrixXd F() const { return this->F_; };
            // Eigen::VectorXd* x() { return &this->x_; };
            // Eigen::MatrixXd* P() { return &this->P_; };
            void setStateCoveriance(const Eigen::MatrixXd& P) 
            {
                this->P_ = P;
            }

            void setState(const Eigen::VectorXd& x)
            {
                this->x_ = x;
            }

        public:
            //状态向量
            Eigen::VectorXd x_;

            //过程协方差矩阵
            Eigen::MatrixXd P_;

            //状态转移矩阵
            Eigen::MatrixXd F_;

            //测量矩阵
            Eigen::MatrixXd H_;

            //测量协方差矩阵
            Eigen::MatrixXd R_;

            //状态协方差矩阵
            Eigen::MatrixXd Q_;

            //状态转移矩阵的雅可比形式
            Eigen::MatrixXd Jf_;

            //测量矩阵的雅可比形式
            Eigen::MatrixXd Jh_;

            //控制矩阵
            Eigen::MatrixXd C_;

            //残差的协方差矩阵
            Eigen::MatrixXd S_;

            //测量向量
            Eigen::VectorXd z_;
        
        public:
            double likelihood_; //似然值
            double dt_ = 0.015; //时间量
            int cp_ = 0; //控制量个数
        
        public:
            KFParam kf_param_;
            void setRCoeff(double& r, int idx)
            {
                switch (idx)
                {
                case 1:
                    kf_param_.measure_noise_params[0] = r;
                    break;
                case 2:
                    kf_param_.measure_noise_params[1] = r;
                    break;
                case 3:
                    kf_param_.measure_noise_params[2] = r;
                    break;
                case 4:
                    kf_param_.measure_noise_params[3] = r;
                    break;
                default:
                    break;
                }
            }

            void setQCoeff(double& q, int idx)
            {
                switch (idx)
                {
                case 1:
                    kf_param_.process_noise_params[0] = q;
                    break;
                case 2:
                    kf_param_.process_noise_params[1] = q;
                    break;
                case 3:
                    kf_param_.process_noise_params[2] = q;
                    break;
                case 4:
                    kf_param_.process_noise_params[3] = q;
                    break;
                case 5:
                    kf_param_.process_noise_params[4] = q;
                    break;
                case 6:
                    kf_param_.process_noise_params[5] = q;
                    break;
                default:
                    break;
                }
            }
    };
} // filter

#endif // KALMAN_FILTER_HPP_