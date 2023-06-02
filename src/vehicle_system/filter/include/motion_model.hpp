/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-26 12:13:56
 * @LastEditTime: 2023-05-29 22:21:14
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/include/motion_model.hpp
 */
#include "./kalman_filter.hpp"

namespace filter
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

    class SingerModel : public KalmanFilter
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
        SingerModel();
        SingerModel(int SP, int MP, int CP);
        SingerModel(const vector<double>* ekf_param, int SP, int MP, int CP);
        virtual SingerModel* Clone() { return new SingerModel(*this); } 
        ~SingerModel();

        void init();
    public:
        void updateC(); 
        void updateC(Eigen::MatrixXd& C, const double dt);
        void updateQ();
        void updateQ(double dt);
        
        // void updateQ(const double acc);
        // void updateQ(const double& dt, const double& alpha, const double& acc);
        // void updateQ(const double& dt, const double& alpha, const double& acc, MatrixXd* Q = nullptr);
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
        UniformModel(int SP, int MP, int CP);
        UniformModel(const vector<double>* kf_param, int SP, int MP, int CP);
        virtual UniformModel* Clone() { return new UniformModel(*this); } 
        ~UniformModel();
        
        void init();

    public:
        double radius_ = 0.20;
        double rangle_ = 0.0;
    };
} // filter

