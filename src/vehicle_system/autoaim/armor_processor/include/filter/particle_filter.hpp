/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 21:16:37
 * @LastEditTime: 2022-10-25 23:47:47
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/particle_filter.hpp
 */
#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#pragma once
#include <iostream>

#include <random>
#include <vector>
#include <ctime>
#include <unistd.h>

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
// #include <opencv2/opencv.hpp>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace std;
using namespace Eigen;

namespace armor_processor
{
    class ParticleFilter
    {
    public:
        ParticleFilter(YAML::Node &config,const string param_name);
        ParticleFilter();
        ~ParticleFilter();

        Eigen::VectorXd predict();
        bool initParam(YAML::Node &config,const string param_name);
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
} //namespace armor_processor

#endif