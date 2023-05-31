/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 17:45:26
 * @LastEditTime: 2023-05-29 22:23:13
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/src/model_generator.cpp
 */
#include "../include/model_generator.hpp"

namespace filter
{
    vector<double> trans_prob_params = {0.6, 0.3, 0.05, 0.05,
                                        0.5, 0.4, 0.05, 0.05,
                                        0.1, 0.1, 0.75, 0.05,
                                        0.1, 0.1, 0.05, 0.75};
    vector<double> model_prob_params = {0.4, 0.3, 0.15, 0.15};
    IMMParam ModelGenerator::imm_param_ = IMMParam{trans_prob_params, model_prob_params};

    ModelGenerator::ModelGenerator(){}
    // ModelGenerator::ModelGenerator(IMMParam imm_param)
    // {
    //     imm_param_ = imm_param;
    // }
    ModelGenerator::~ModelGenerator(){}

    /**
     * @brief 生成IMM模型
     * 
     * @param x 状态向量
     * @param dt 时间量
     * @return std::shared_ptr<IMM> 
     */
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
        double tP[] = {imm_param_.imm_model_trans_prob_params[0], imm_param_.imm_model_trans_prob_params[1], imm_param_.imm_model_trans_prob_params[2], imm_param_.imm_model_trans_prob_params[3],
            imm_param_.imm_model_trans_prob_params[4], imm_param_.imm_model_trans_prob_params[5], imm_param_.imm_model_trans_prob_params[6], imm_param_.imm_model_trans_prob_params[7],
            imm_param_.imm_model_trans_prob_params[8], imm_param_.imm_model_trans_prob_params[9], imm_param_.imm_model_trans_prob_params[10], imm_param_.imm_model_trans_prob_params[11],
            imm_param_.imm_model_trans_prob_params[12], imm_param_.imm_model_trans_prob_params[13], imm_param_.imm_model_trans_prob_params[14], imm_param_.imm_model_trans_prob_params[15]};
        double tP0 = (1.0 - (tP[0] + tP[1] + tP[2]));
        double tP1 = (1.0 - (tP[4] + tP[5] + tP[6]));
        double tP2 = (1.0 - (tP[8] + tP[9] + tP[10]));
        double tP3 = (1.0 - (tP[12] + tP[13] + tP[14]));
        trans_prob << tP[0], tP[1], tP[2], tP0,
                      tP[4], tP[5], tP[6], tP1,
                      tP[8], tP[9], tP[10],tP2, 
                      tP[12],tP[13],tP[14],tP3;
        Eigen::VectorXd model_prob = Eigen::VectorXd::Zero(4);
        double mP[] = {imm_param_.imm_model_prob_params[0], imm_param_.imm_model_prob_params[1], imm_param_.imm_model_prob_params[2], imm_param_.imm_model_prob_params[3]};
        double mP0 = (1.0 - (mP[0] + mP[1] + mP[2]));
        model_prob << mP[0], mP[1], mP[2], mP0;

        imm_ptr->init(x, model_prob, trans_prob);

        return imm_ptr;
    }
    
    /**
     * @brief 生成CV（匀速运动）模型
     * 
     * @param x 状态向量
     * @param dt 时间量
     * @return std::shared_ptr<CV> 
     */
    std::shared_ptr<CV> ModelGenerator::generateCVModel(const Eigen::VectorXd& x, const double& dt)
    {
        std::shared_ptr<CV> cv_ptr = std::shared_ptr<CV>(new CV());
        cv_ptr->init(x, dt);
        return cv_ptr;
    }

    /**
     * @brief 同上
     * 
     * @param x 
     * @param dt 
     * @return std::shared_ptr<CA> 
     */
    std::shared_ptr<CA> ModelGenerator::generateCAModel(const Eigen::VectorXd& x, const double& dt)
    {
        std::shared_ptr<CA> ca_ptr = std::shared_ptr<CA>(new CA());
        ca_ptr->init(x, dt);
        return ca_ptr;
    }

    /**
     * @brief 同上
     * 
     * @param x 
     * @param w 
     * @param dt 
     * @return std::shared_ptr<CT> 
     */
    std::shared_ptr<CT> ModelGenerator::generateCTModel(const Eigen::VectorXd& x, const double& w, const double& dt)
    {
        std::shared_ptr<CT> ct_ptr = std::shared_ptr<CT>(new CT(w));
        ct_ptr->init(x, dt);
        return ct_ptr;
    }

} //filter