/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 14:01:05
 * @LastEditTime: 2022-09-06 02:15:57
 * @FilePath: /tup_2023/src/global_user/src/global_user.cpp
 */
#include "../include/global_user/global_user.hpp"

namespace global_user
{
    global_user::global_user()
    {
        config_path[0] = config_file_autoaim;
        config_path[1] = config_file_buff;
    }

    float calcTriangleArea(cv::Point2f pts[3])
    {
        /**
         * @brief caculate the areas of triangle
         * @param apexes
         * @return area
         */
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

        auto p = (a + b + c) / 2.f;
        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    float calcTetragonArea(cv::Point2f pts[4])
    {
        return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
    }

    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
    {
        /**
         * @brief transform rotatedMatrix to euler angle
         * 
         */
        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;
        
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }

        return {z, y, x};
    }

    Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2)
    {
        Eigen::Vector3d delta_euler;
        //将Roll表示范围由[-PI,PI]转换至[0，2PI]
        for (int i = 0; i < 3;i++)
        {
            if (euler2[i] <= 0)
                euler2[i] += CV_2PI;
            if (euler1[i] <= 0)
                euler1[i] += CV_2PI;
        }

        for (int i = 0; i < 3;i++)
        {
            if (euler2[i] > 0 && euler2[i] < (CV_PI / 2) && euler1[i] > (3 * CV_PI / 2))
                delta_euler[i] = CV_2PI + euler2[i] - euler1[i];
            else if (euler2[i] > (3 * CV_PI / 2) && euler1[i] > 0 && euler1[i] < (CV_PI / 2))
                delta_euler[i] = -CV_2PI + euler2[i] - euler1[i];
            else
                delta_euler[i] = euler2[i] - euler1[i];
        }

        return delta_euler;
    }
    
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta)
    {
        Eigen::Matrix3d R_x;
        Eigen::Matrix3d R_y;
        Eigen::Matrix3d R_z;
        // Calculate rotation about x axis
        R_x <<
            1,       0,              0,
            0,       cos(theta[2]),   -sin(theta[2]),
            0,       sin(theta[2]),   cos(theta[2]);
        // Calculate rotation about y axis
        R_y <<
            cos(theta[1]),    0,      sin(theta[1]),
            0,               1,      0,
            -sin(theta[1]),   0,      cos(theta[1]);
        // Calculate rotation about z axis
        R_z <<
            cos(theta[0]),    -sin(theta[0]),      0,
            sin(theta[0]),    cos(theta[0]),       0,
            0,               0,                  1;
        // Combined rotation matrix
        return R_z * R_y * R_x;
    }

    Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler)
    {
        Eigen::AngleAxisd rotVec;
        Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
        Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));

        rotVec = roll_angle * yaw_angle * pitch_angle;
        return rotVec;
    }

    double rangedAngleRad(double &angle)
    {
        if (fabs(angle) >= CV_PI)
        {
            angle -= (angle / fabs(angle)) * CV_2PI;
            angle = rangedAngleRad(angle);
        }
        return angle;
    }
    
    
};

