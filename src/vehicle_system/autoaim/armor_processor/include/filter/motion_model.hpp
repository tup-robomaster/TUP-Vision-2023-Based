/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:13:56
 * @LastEditTime: 2022-11-30 11:14:37
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/filter/motion_model.hpp
 */
#include "./kalman_filter.hpp"

namespace armor_processor
{
    class CV : public KalmanFilter
    {
    private:
        void updatePrediction();
        void updateMeasurement();
        void setCoeff(const double& coeff);

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
        void setCoeff(const double& coeff);

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
        void setCoeff(const double& coeff);
        const double w_;
    public:
        CT(const double& w);
        ~CT();
        void init(const Eigen::VectorXd& x, const double& dt);
        virtual CT* Clone() { return new CT(*this); }
    };
} // armor_processor

