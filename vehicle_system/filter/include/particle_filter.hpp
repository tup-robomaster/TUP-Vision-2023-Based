/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 02:36:09
 * @LastEditTime: 2022-09-13 22:24:08
 * @FilePath: /tup_2023/src/vehicle_system/filter/include/particle_filter.hpp
 */
#include "filter/filter.hpp"
#include <iostream>
#include <random>

namespace filter
{
    using global_user::initMatrix;
    
    class ParticleFilter : public filter
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

