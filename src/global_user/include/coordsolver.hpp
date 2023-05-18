/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 03:13:13
 * @LastEditTime: 2023-05-18 23:44:10
 * @FilePath: /TUP-Vision-2023-Based/src/global_user/include/coordsolver.hpp
 */

#include <yaml-cpp/yaml.h>

// #include <fmt/color.h>
// #include <fmt/format.h>
// #include <glog/logging.h>

//ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "global_user/global_user.hpp"

using namespace global_user;
using namespace cv;
namespace coordsolver
{
    struct PnPInfo
    {
        Eigen::Vector3d armor_cam;
        Eigen::Vector3d armor_world;
        Eigen::Vector3d R_cam;
        Eigen::Vector3d R_world;
        Eigen::Vector3d euler;
        Eigen::Matrix3d rmat;
        Eigen::Quaterniond quat_cam;
        double rangle;
        bool is_solver_success;
    };

    class CoordSolver
    {
    public:
        CoordSolver();
        ~CoordSolver();
        
        bool loadParam(std::string coord_path, std::string param_name);

        double dynamicCalcPitchOffset(Eigen::Vector3d &xyz);
        
        PnPInfo pnp(const std::vector<cv::Point2f> &points_pic, const Eigen::Matrix3d& rmat_gimbal, const Eigen::Vector3d& translation, enum TargetType type, int method);
        
        // Eigen::Vector3d camToWorld(const Eigen::Vector3d &point_camera,const Eigen::Matrix3d &rmat);
        // Eigen::Vector3d worldToCam(const Eigen::Vector3d &point_world,const Eigen::Matrix3d &rmat);
        // Eigen::Vector2d getAngle(Eigen::Vector3d &xyz_cam, Eigen::Matrix3d &rmat);

        Eigen::Vector3d staticCoordOffset(Eigen::Vector3d &xyz);
        Eigen::Vector2d staticAngleOffset(Eigen::Vector2d &angle);
        Eigen::Vector2d getAngle(Eigen::Vector3d xyz_cam, Eigen::Matrix3d rmat_gimbal, Eigen::Vector3d translation);
        bool setStaticAngleOffset(const Eigen::Vector2d& static_angle_offset);
        double getBulletSpeed();

        inline double calcYaw(Eigen::Vector3d &xyz);
        inline double calcPitch(Eigen::Vector3d &xyz);
        Eigen::Vector2d calcYawPitch(Eigen::Vector3d &xyz);
        bool setBulletSpeed(double speed);
        cv::Point2f reproject(Eigen::Vector3d &xyz);
        cv::Point2f getHeading(Eigen::Vector3d &xyz_cam);

    private:
        YAML::Node param_node;
        int max_iter = 10;          // 使用迭代法求解pitch补偿的最大迭代次数
        float stop_error = 0.001;   // 停止迭代的最小误差(单位m)
        int R_K_iter = 50;          // 龙格库塔法求解落点的迭代次数
        Eigen::Vector3d xyz_offset = {0.0, 0.0, 0.0}; // x,y,z坐标偏移量(相机至枪管),单位m
        
        // Eigen::Vector3d t_iw;
        // Eigen::Matrix4d transform_ic;
        // Eigen::Matrix4d transform_ci;

        Eigen::Vector2d angle_offset = {0.0, 0.0};
        cv::Mat intrinsic = cv::Mat(3, 3, CV_64FC1);
        cv::Mat dis_coeff = cv::Mat(1, 5, CV_64FC1);

        double bullet_speed = 16.0;   
        // const double k = 0.01903;                //25°C,1atm,小弹丸
        const double k = 0.00556;                //25°C,1atm,大弹丸
        // const double k = 0.00530;                //25°C,1atm,发光大弹丸
        const double g = 9.781;
       
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        rclcpp::Logger logger_;
    };
} //coordsolver