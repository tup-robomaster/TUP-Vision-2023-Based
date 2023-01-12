/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 17:45:26
 * @LastEditTime: 2023-01-12 15:30:29
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/filter/model_generator.cpp
 */
#include "../../include/filter/model_generator.hpp"

namespace armor_processor
{
    ModelGenerator::ModelGenerator(){}
    ModelGenerator::~ModelGenerator(){}

    std::shared_ptr<IMM> ModelGenerator::generateIMMModel(const Eigen::VectorXd& x, const double& dt)
    {
        std::shared_ptr<IMM> imm_ptr = std::shared_ptr<IMM>(new IMM());
        
        auto cv = generateCVModel(x, dt);
        auto ca = generateCAModel(x, dt);
        auto ct_pos = generateCTModel(x, 0.1, dt);
        auto ct_neg = generateCTModel(x, -0.1, dt);
        imm_ptr->addModel(cv);
        imm_ptr->addModel(ca);
        imm_ptr->addModel(ct_pos);
        imm_ptr->addModel(ct_neg);

        Eigen::MatrixXd trans_prob = Eigen::MatrixXd::Zero(4, 4);
        //TODO:debug...
        trans_prob << 0.6, 0.3, 0.05, 0.05,
                      0.5, 0.4, 0.05, 0.05,
                      0.1, 0.1, 0.75, 0.05,
                      0.1, 0.1, 0.05, 0.75;
        Eigen::VectorXd model_prob = Eigen::VectorXd::Zero(4);
        model_prob << 0.4, 0.3, 0.15, 0.15;

        imm_ptr->init(x, model_prob, trans_prob);

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