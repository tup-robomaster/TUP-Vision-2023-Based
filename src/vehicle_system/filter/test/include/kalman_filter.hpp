/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:00:19
 * @LastEditTime: 2023-05-29 22:02:12
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/kalman_filter.hpp
 */
#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace kalman_filter
{
    class KalmanFilter
    {
        public:
            KalmanFilter();
            ~KalmanFilter();

            /**
             * @brief 初始化卡尔曼滤波器
             * @param x_in 初始化状态向量
             * @param P_in 初始化状态协方差矩阵
             * @param F_in 状态转移矩阵
             * @param H_in 测量矩阵
             * @param R_in 测量协方差矩阵
             * @param Q_in 过程协方差矩阵
             */
            void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
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
            void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
            Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in, Eigen::MatrixXd& J_in);

            
            /**
             * @brief 预测状态向量和协方差矩阵
             * 
             */
            void Predict();

            /**
             * @brief 更新状态向量
             * 
             */
            void Update(const Eigen::VectorXd& z);

            /**
             * @brief 更新状态向量（EKF）
             * 
             */
            void UpdateEKF(const Eigen::VectorXd& z);

            //状态向量
            Eigen::VectorXd x_;

            //状态协方差矩阵
            Eigen::MatrixXd P_;

            //状态转移矩阵
            Eigen::MatrixXd F_;

            //测量矩阵
            Eigen::MatrixXd H_;

            //测量协方差矩阵
            Eigen::MatrixXd R_;

            //过程协方差矩阵
            Eigen::MatrixXd Q_;

            //雅可比矩阵
            Eigen::MatrixXd J_;
    };
} // kalman_filter

#endif // KALMAN_FILTER_HPP_