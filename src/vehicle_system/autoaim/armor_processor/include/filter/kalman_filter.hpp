/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-31 19:00:19
 * @LastEditTime: 2022-11-20 21:52:43
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/kalman_filter.hpp
 */
#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace armor_processor
{
    class KalmanFilter
    {
        public:
            KalmanFilter();
            ~KalmanFilter();

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

            /**
             * @brief 更新状态向量
             * 
             */
            void Update(const Eigen::MatrixXd& z);

            /**
             * @brief 更新状态向量（EKF）
             * 
             */
            void UpdateEKF(const Eigen::MatrixXd& z);

            /**
             * @brief Jacobians
             * 
             */
            void UpdateJacobians();

        public:
            //状态向量
            Eigen::MatrixXd x_;

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

            //Control matrix
            Eigen::MatrixXd C_;
        
        private:
            int cp_;

    };
} // armor_processor

#endif // KALMAN_FILTER_HPP_