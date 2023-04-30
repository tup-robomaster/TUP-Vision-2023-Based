/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:00:19
 * @LastEditTime: 2023-04-30 17:55:02
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/kalman_filter.hpp
 */
#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <cassert>

using namespace std;
namespace armor_processor
{
    struct KFParam
    {
        vector<double> process_noise_params;
        vector<double> measure_noise_params;
        vector<double> singer_params;
    };

    class KalmanFilter
    {
        public:
            KalmanFilter();
            KalmanFilter(KFParam kf_param);
            ~KalmanFilter();
            virtual KalmanFilter* Clone()
            {
                return new KalmanFilter(*this);
            }

            /**
             * @brief Initialize matrix dimension
             * 
             */
            void Init(int stateParams, int measureParams, int controlParams);

            /**
             * @brief 初始化卡尔曼滤波器
             * @param x_in 初始化状态向量
             * @param P_in 初始化状态协方差矩阵
             * @param F_in 状态转移矩阵
             * @param H_in 测量矩阵
             * @param R_in 测量协方差矩阵
             * @param Q_in 过程协方差矩阵
             */
            void Init(Eigen::MatrixXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
                Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);
            
            /**
             * @brief 初始化卡尔曼滤波器
             * @param x_in 初始化状态向量
             * @param P_in 初始化状态协方差矩阵
             * @param F_in 状态转移矩阵
             * @param H_in 测量矩阵
             * @param R_in 测量协方差矩阵
             * @param Q_in 过程协方差矩阵
             * @param J_in 雅可比矩阵
             */
            void Init(Eigen::MatrixXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
            Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in, Eigen::MatrixXd& J_in);
            
            /**
             * @brief 预测状态向量和协方差矩阵
             * 
             */
            void Predict();
            void Predict(const double& dt);
            virtual void updatePrediction() {}
            virtual void updateMeasurement() {}

            /**
             * @brief 更新状态向量
             * 
             */
            void Update(const Eigen::VectorXd& z);
            void Update(const Eigen::VectorXd& z, int mp);
            void Update(const Eigen::VectorXd& z, double rangle);
            void updateOnce(const double& dt, const Eigen::VectorXd* z);

            /**
             * @brief 更新状态向量（EKF）
             * 
             */
            void UpdateEKF(const Eigen::MatrixXd& z);

            /**
             * @brief 雅可比矩阵
             * 
             */
            void UpdateJacobians();

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

            //雅可比矩阵
            Eigen::MatrixXd J_;

            //控制矩阵
            Eigen::MatrixXd C_;

            //残差的协方差矩阵
            Eigen::MatrixXd S_;

            //测量向量
            Eigen::VectorXd z_;
        
        public:
            double likelihood_; //似然值
            double dt_ = 0.015; //时间量
            int cp_; //控制量个数
        
        public:
            // double r1_, r2_, r3_, r4_;
            // double q1_, q2_, q3_, q4_, q5_, q6_;
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
} // armor_processor

#endif // KALMAN_FILTER_HPP_