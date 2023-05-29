/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 17:45:11
 * @LastEditTime: 2023-05-29 22:32:41
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/model_generator.hpp
 */
#ifndef MODEL_GENERATOR_HPP_
#define MODEL_GENERATOR_HPP_

#include "./imm.hpp"

namespace filter
{
    class ModelGenerator
    {
    public:
        ModelGenerator();
        // ModelGenerator(IMMParam imm_param);
        ~ModelGenerator();

        static IMMParam imm_param_;
        static std::shared_ptr<IMM> generateIMMModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CV> generateCVModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CA> generateCAModel(const Eigen::VectorXd& x, const double& dt);
        static std::shared_ptr<CT> generateCTModel(const Eigen::VectorXd& x, const double& w, const double& dt);
    };

    // vector<double> trans_prob_params = {0.6, 0.3, 0.05, 0.05,
    //                                     0.5, 0.4, 0.05, 0.05,
    //                                     0.1, 0.1, 0.75, 0.05,
    //                                     0.1, 0.1, 0.05, 0.75};
    // vector<double> model_prob_params = {0.4, 0.3, 0.15, 0.15};
    // IMMParam ModelGenerator::imm_param_ = IMMParam{vector<double>{0.6, 0.3, 0.05, 0.05,
    //                                                               0.5, 0.4, 0.05, 0.05,
    //                                                               0.1, 0.1, 0.75, 0.05,
    //                                                               0.1, 0.1, 0.05, 0.75}, vector<double>{0.4, 0.3, 0.15, 0.15}};
} //filter

#endif