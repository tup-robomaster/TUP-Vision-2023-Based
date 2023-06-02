/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 02:36:09
 * @LastEditTime: 2023-05-29 22:28:28
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/particle_filter.hpp
 */
#include <iostream>
#include <random>
#include "../../../global_user/include/global_user/global_user.hpp"

namespace filter
{
    using global_user::initMatrix;
    
    class ParticleFilter 
    {
    public:
        ParticleFilter(YAML::Node &config,const std::string param_name);
        ParticleFilter();
        ~ParticleFilter();

        Eigen::VectorXd predict();
        bool initParam(YAML::Node &config,const std::string param_name);
        bool initParam(ParticleFilter parent);
        bool update(Eigen::VectorXd measure);
        bool is_ready;
    private:
        bool resample();

        int vector_len;
        int num_particle;

        Eigen::MatrixXd process_noise_cov;
        Eigen::MatrixXd observe_noise_cov;
        Eigen::MatrixXd weights;

        Eigen::MatrixXd matrix_estimate;
        Eigen::MatrixXd matrix_particle;
        Eigen::MatrixXd matrix_weights;
    };
    
} // namespace filter

