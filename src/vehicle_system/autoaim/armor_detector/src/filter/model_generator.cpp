/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 17:45:26
 * @LastEditTime: 2022-11-29 16:23:20
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/filter/model_generator.cpp
 */
#include "../../include/filter/model_generator.hpp"

namespace armor_detector
{
    ModelGenerator::ModelGenerator(){}
    ModelGenerator::~ModelGenerator(){}

    std::shared_ptr<IMM> ModelGenerator::generateIMMModel(const Eigen::VectorXd& x, const double& dt)
    {
        std::shared_ptr<IMM> imm_ptr = std::shared_ptr<IMM>(new IMM());
        
        // std::cout << 13 << std::endl;
        auto cv = generateCVModel(x, dt);
        // std::cout << 14 << std::endl;

        auto ca = generateCAModel(x, dt);
        // std::cout << 15 << std::endl;

        auto ct_pos = generateCTModel(x, 0.1, dt);
        // std::cout << 16 << std::endl;

        auto ct_neg = generateCTModel(x, -0.1, dt);
        // std::cout << 17 << std::endl;

        imm_ptr->addModel(cv);
        imm_ptr->addModel(ca);
        imm_ptr->addModel(ct_pos);
        imm_ptr->addModel(ct_neg);

        Eigen::MatrixXd trans_prob = Eigen::MatrixXd::Zero(4, 4);
        trans_prob << 0.2, 0.6, 0.1, 0.1,
                      0.2, 0.6, 0.1, 0.1,
                      0.2, 0.6, 0.1, 0.1,
                      0.2, 0.6, 0.1, 0.1;
        Eigen::VectorXd model_prob = Eigen::VectorXd::Zero(4);
        model_prob << 0.2, 0.6, 0.1, 0.1;

        imm_ptr->init(x, model_prob, trans_prob);

        // std::cout << 18 << std::endl;

        return imm_ptr;
    }
    
    std::shared_ptr<CV> ModelGenerator::generateCVModel(const Eigen::VectorXd& x, const double& dt)
    {
        std::shared_ptr<CV> cv_ptr = std::shared_ptr<CV>(new CV());
        cv_ptr->init(x, dt);
        return cv_ptr;
    }

    std::shared_ptr<CA> ModelGenerator::generateCAModel(const Eigen::VectorXd& x, const double& dt)
    {
        std::shared_ptr<CA> ca_ptr = std::shared_ptr<CA>(new CA());
        ca_ptr->init(x, dt);
        return ca_ptr;
    }

    std::shared_ptr<CT> ModelGenerator::generateCTModel(const Eigen::VectorXd& x, const double& w, const double& dt)
    {
        std::shared_ptr<CT> ct_ptr = std::shared_ptr<CT>(new CT(w));
        ct_ptr->init(x, dt);
        return ct_ptr;
    }

} //armor_processor