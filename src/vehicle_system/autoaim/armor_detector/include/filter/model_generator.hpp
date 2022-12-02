/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 17:45:11
 * @LastEditTime: 2022-11-29 10:29:22
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/filter/model_generator.hpp
 */
#ifndef MODEL_GENERATOR_HPP_
#define MODEL_GENERATOR_HPP_

#include "./imm.hpp"

namespace armor_detector
{
    class ModelGenerator
    {
    
    public:
        ModelGenerator();
        ~ModelGenerator();
        
        static std::shared_ptr<IMM> generateIMMModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CV> generateCVModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CA> generateCAModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CT> generateCTModel(const Eigen::VectorXd& x, const double& w, const double& dt);
    };
} //armor_processor

#endif