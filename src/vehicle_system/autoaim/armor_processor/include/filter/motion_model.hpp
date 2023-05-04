/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:13:56
 * @LastEditTime: 2023-05-04 02:29:20
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/motion_model.hpp
 */
#include "./kalman_filter.hpp"
#include "../prediction/param_struct.hpp"

namespace armor_processor
{
    class CV : public KalmanFilter
    {
    private:
        void updatePrediction();
        void updateMeasurement();
    public:
        CV();
        ~CV();
        void init(const Eigen::VectorXd& x, const double& dt);
        virtual CV* Clone() {return new CV(*this);}
    };

    class CA : public KalmanFilter
    {
    private:
        void updatePrediction();
        void updateMeasurement();

    public:
        CA();
        ~CA();
        void init(const Eigen::VectorXd& x, const double& dt);
        virtual CA* Clone() { return new CA(*this); }
    };
    
    class CT : public KalmanFilter
    {
    private:
        void updatePrediction();
        void updateMeasurement();

        const double w_;
    public:
        CT(const double& w);
        ~CT();
        void init(const Eigen::VectorXd& x, const double& dt);
        virtual CT* Clone() { return new CT(*this); }
    };

    class UniformModel : public KalmanFilter
    {
    public:
        void updateF();
        void updateF(Eigen::MatrixXd& Ft, double dt);
        void updateH();
        void updateH(Eigen::MatrixXd& Ht, double dt);
        void updateJf();
        void updateJf(Eigen::MatrixXd& Jft, double dt);
        void updateJh();
        void updateJh(Eigen::MatrixXd& Jht, double dt);
        
    public:
        UniformModel();
        UniformModel(const KFParam kf_param);
        virtual UniformModel* Clone() { return new UniformModel(*this); } 
        ~UniformModel();
        
        void init();

    public:
        double radius_ = 0.20;
        double rangle_ = 0.0;
    };
} // armor_processor

