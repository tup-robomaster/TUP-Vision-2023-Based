/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-06 16:45:33
 * @LastEditTime: 2022-11-13 11:50:09
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/imm.hpp
 */
#include "./imm_system_model.hpp"

//Model(CA、CV、CTRV) 
template<typename T, template<class> class CovarianceBase = filter::Base>
class CVModel : public filter::LinearSystemModel<CVModelState<T>, CVModelControl<T>, CovarianceBase>
{
public:
    typedef CVModelState<T> S;
    typedef CVModelControl<T> C;

    S f(const S& x, const C& u, const int& t) const
    {
        S x_;
        
        x_.v_x() = x.v_x();
        x_.v_y() = x.v_y();
        x_.x() = x.x() + x.v_x() * t;
        x_.y() = x.y() + x.v_y() * t;

        return x_;
    }

protected:
    void updateJacobians(const S& x, const C& u, const int& t)
    {   //CV模型的雅可比矩阵（还是原状态转移矩阵）
        //
        this->F.setZero();

        //X对X的偏导数
        this->F(S::X, S::X) = 1;

        //Y对Y的偏导数
        this->F(S::Y, S::Y) = 1;

        //Vx对Vx的偏导数
        this->F(S::Vx, S::Vx) = 1;

        //Vy对Vy的偏导数
        this->F(S::Vy, S::Vy) = 1;

        //X对Vx的偏导数
        this->F(S::X, S::Vx) = t;

        //Y对Vy的偏导数
        this->F(S::Y, S::Vy) = t;
    }
};

template<typename T, template<class> class CovarianceBase = filter::Base>
class CAModel : public filter::LinearSystemModel<CAModelState<T>, CAModelControl<T>, CovarianceBase>
{
public:
    typedef CAModelState<T> S;
    typedef CAModelControl<T> C;

    S f(const S& x, const C& u, const int& t)
    {
        S x_;

        x_.a_x() = x.a_x();
        x_.a_y() = x.a_y();

        x_.v_x() = x.v_x() + x.a_x() * t;
        x_.v_y() = x.v_y() + x.a_y() * t;

        x_.x() = x.x() + x.v_x() * t + (1 / 2) * x.a_x() * t * t;
        x_.y() = x.x() + x.v_y() * t + (1 / 2) * x.a_y() * t * t; 

        return x_;
    }

public:
    void updateJacobians(const S& x, const C& u, const int& t)
    {   //CA模型的雅可比矩阵（还是原状态转移矩阵）
        //矩阵初始化全部置为0
        this->F.setZero();

        //求偏导数
        this->F(S::X, S::X) = 1;
        this->F(S::Y, S::Y) = 1;
        this->F(S::Vx, S::Vx) = 1;
        this->F(S::Vy, S::Vy) = 1;
        this->F(S::ax, S::ax) = 1;
        this->F(S::ay, S::ay) = 1;

        this->F(S::X, S::Vx) = t;
        this->F(S::X, S::ax) = (1 / 2) * t * t;
        this->F(S::Y, S::Vy) = t;
        this->F(S::Y, S::ay) = (1 / 2) * t * t;
        this->F(S::Vx, S::ax) = t;
        this->F(S::Vy, S::ay) = t;
    }
};

template<typename T, template<class> class CovarianceBase = filter::Base>
class CTRVModel : public filter::LinearSystemModel<CTRVModelState<T>, CTRVModelControl<T>, CovarianceBase>
{
public:
    typedef CTRVModelState<T> S;
    typedef CTRVModelControl<T> C;

    S f(const S& x, const C& u, const float& t) const
    {
        S x_;

        if(x.w() == 0)
        {
            x_.v() = x.v();
            x_.w() = x.w();
            x_.theta() = x.theta();

            x_.x() = x.x() + x.v() * t * cos(x.theta());
            x_.y() = x.y() + x.v() * t * sin(x.theta());
        }
        else
        {
            x_.x() = x.x() + (x.v() / x.w()) * (sin(x.w() * t + x.theta()) - sin(x.theta()));
            x_.y() = x.y() + (x.v() / x.w()) * (cos(x.theta()) - cos(x.w() * t + x.theta()));
            x_.v() = x.v();
            x_.theta() = x.theta() + x.w() * t;
            x_.w() = x.w();
        }

        return x_;
    }

public:
    void updateJacobians(const S& x, const C& u, const float& t)
    {   //CTRV模型的雅可比矩阵
        //初始化雅可比矩阵各项为0
        this->F.setZero();

        this->F(S::X, S::X) = 1;
        this->F(S::Y, S::Y) = 1;
        this->F(S::V, S::V) = 1;
        this->F(S::Theta, S::Theta) = 1;
        this->F(S::W, S::W) = 1;

        // std::cout << t << std::endl;
        if(x.w() == 0)
        {
            this->F(S::X, S::V) = t * cos(x.theta());
            this->F(S::X, S::Theta) = -x.v() * t * sin(x.theta());
            this->F(S::Y, S::V) = t * sin(x.theta());
            this->F(S::Y, S::Theta) = -x.v() * t * cos(x.theta());
            this->F(S::Theta, S::W) = t;
        }
        else
        {
            //TODO:fixing
            this->F(S::X, S::V) = (1 / x.w()) * (sin(x.theta() + x.w() * t) - sin(x.theta()));
            this->F(S::X, S::Theta) = (x.x() / x.w()) * (cos(x.w() * t + x.theta()) - cos(x.theta()));
            this->F(S::X, S::W) = ((x.v() * t) / x.w()) * cos(x.w() * t + x.theta()) - (x.v() / (x.w() * x.w())) * (sin(x.w() * t + x.theta()) - sin(x.theta()));
            this->F(S::Y, S::V) = (1 / x.w()) * (-cos(x.theta() + x.w() * t) + cos(x.theta()));
            this->F(S::Y, S::Theta) = (x.x() / x.w()) * (sin(x.w() * t + x.theta()) - sin(x.theta()));
            this->F(S::Y, S::W) = ((x.v() * t) / x.w()) * sin(x.w() * t + x.theta()) - (x.v() / (x.w() * x.w())) * (-cos(x.w() * t + x.theta()) - cos(x.theta()));
            this->F(S::Theta, S::W) = t;
        }

        //TODO:
        this->W.setIdentity();
    }
};

template<class T, template<class> class CovarianceBase = filter::Base>
class CTModel : public filter::LinearSystemModel<ModelState<T>, ModelControl<T>, CovarianceBase>
{
    typedef ModelState<T> S;
    typedef ModelControl<T> C;

public:
    /**
     * @brief 已知转弯角速度w的常速转弯（CT）模型
     * @param x 状态量
     * @param u 控制量
     * @param w 转弯角速度
     * @param t 时间量
    */
    S f(const S& x, const C& u, const double& w, const double& t) const
    {
        S x_;

        if(w != 0)
        {
            x_.x0() = x.x0() + (sin(w * t) / w) * x.x1() + ((cos(w * t) - 1) / w) * x.x3(); //X
            x_.x1() = x.x1() * cos(w * t) - x.x3() * sin(w * t);                            //Vx
            x_.x2() = x.x2() + (sin(w * t) / w) * x.x3() + ((1 - cos(w * t)) / w) * x.x1(); //Y
            x_.x3() = x.x3() * cos(w * t) + x.x1() * sin(w * t);                            //Vy
        }
        else
        {
            x_.x0() = x.x0() + x.x1() * t;
            x_.x1() = x.x1();
            x_.x2() = x.x2() + x.x3() * t;
            x_.x3() = x.x3();
        }

        return x_;
    }

    /**
     * @brief 转速未知的常速转弯模型
     * 
     */
    S f(const S& x, const C& u, const double& t) const
    {
        
    }


public:
    void updateJacobians(const S& x, const C& u, const double& w, const double& t)
    {
        this->F.setZero();

        if(w != 0)
        {
            this->F(S::X0, S::X0) = 1;
            this->F(S::X0, S::X1) = sin(w * t) / w;
            this->F(S::X0, S::X3) = (cos(w * t) - 1) / w;
            this->F(S::X1, S::X1) = cos(w * t);
            this->F(S::X1, S::X3) = -sin(w * t);
            this->F(S::X2, S::X1) = (1 - cos(w * t)) / w;
            this->F(S::X2, S::X2) = 1;
            this->F(S::X2, S::X3) = sin(W * t) / w;
            this->F(S::X3, S::X1) = sin(W * t);
            this->F(S::X3, S::X3) = cos(W * t);
        }
        else
        {
            this->F(S::X0, S::X0) = 1;
            this->F(S::X0, S::X1) = t;
            this->F(S::X1, S::X1) = 1;
            this->F(S::X2, S::X2) = 1;
            this->F(S::X2, S::X3) = t;
            this->F(S::X3, S::X3) = 1;
        }

        //TODO:
        this->W.setIdentity();
    }
}

template<typename T>
class PositionMeasurement : public filter::Vector<T, 6>
{
    typedef filter::Vector<T, 6> Vector;
public:
    //TODO:
    constexpr static size_t X0 = 0;
    constexpr static size_t X1 = 1;
    constexpr static size_t X2 = 2;
    constexpr static size_t X3 = 3;
    constexpr static size_t X4 = 4;
    constexpr static size_t X5 = 5;

public:
    Vector x() const {return (*this);}
    T x0() const {return (*this)[X0];}
    T x1() const {return (*this)[X1];}
    T x2() const {return (*this)[X2];}
    T x3() const {return (*this)[X3];}
    T x4() const {return (*this)[X4];}
    T x5() const {return (*this)[X5];}

    Vector& x() {return (*this);}
    T& x0() {return (*this)[X0];}
    T& x1() {return (*this)[X1];}
    T& x2() {return (*this)[X2];}
    T& x3() {return (*this)[X3];}
    T& x4() {return (*this)[X4];}
    T& x5() {return (*this)[X5];}
};

template<typename T, template<class> class CovarianceBase = filter::Base>
class PositionMeasurementModel : public filter::LinearMeasurementModel<CTRVModelState<T>, PositionMeasurement<T>, CovarianceBase>
{
public:
    typedef ModelState<T> S;
    typedef PositionMeasurement<T> M;

    PositionMeasurementModel()
    {
        this->H.setIdentity();
        this->V.setIdentity();
    }

    //状态向量映射到观测空间
    M h(const S& x) const
    {
        M m_;

        m_.x0() = x.x0();
        m_.x1() = x.x1();
        m_.x2() = x.x2();
        m_.x3() = x.x3();
        m_.x4() = x.x4();
        m_.x5() = x.x5();

        return m_;
    }

protected:
    void updateJacobians(const S& x)
    {
        //
        this->H.setZero();

        //偏导数
        this->H(M::X0, S::X0) = 1;
        this->H(M::X1, S::X1) = 1;
        this->H(M::X2, S::X2) = 1;
        this->H(M::X3, S::X3) = 1;
        this->H(M::X4, S::X4) = 1;
        this->H(M::X5, S::X5) = 1;
    }
};

template<typename T>
class IMM
{
private:
    typedef CVModelState<T> CVState;
    typedef CVModelControl<T> CVControl; 
    typedef CAModelState<T> CAState;
    typedef CAModelControl<T> CAControl;
    typedef CTRVModelState<T> CTRVState;
    typedef CTRVModelControl<T> CTRVControl;
    typedef ModelState<T> State;
    typedef PositionMeasurementModel<T> PositionModel;

    using Matrix = Eigen::MatrixXd;
    using Vector = Eigen::VectorXd;

    template<template<class> class CovarianceBase = filter::Base>
    using SystemModel = filter::LinearSystemModel<ModelState<T>, ModelControl<T>, CovarianceBase>;

private:
    std::vector<std::shared_ptr<filter::ExtendKalmanFilter<State>>> models_;

    Matrix transfer_prob_;  //马尔可夫状态转移矩阵
    Matrix P_;              //状态协方差矩阵
    Matrix X_;              //

    Vector c_;              //状态交互后各模型概率    
    Vector model_prob_;     //模型概率
    Vector x_;              //目标状态向量

    size_t model_num_;      //模型数量
    size_t state_num_;      //状态向量元素个数

public:
    IMM(){}
    ~IMM(){}

    //添加运动模型
    //CV
    void addCVModel(const std::shared_ptr<filter::ExtendKalmanFilter<CVState>> model);
    // //CA
    void addCAModel(const std::shared_ptr<filter::ExtendKalmanFilter<CAState>> model);
    // //CTRV
    void addCTRVModel(const std::shared_ptr<filter::ExtendKalmanFilter<CTRVState>> model);

    //初始化（目标状态、状态协方差、模型概率和马尔可夫状态转移矩阵）
    void init(
        const Vector& x,
        const Matrix& P,
        const Matrix& model_prob,
        const Matrix& transfer_prob
    );

    /**
     * @brief IMM算法流程
     * 
     */
    // step1:输入交互
    void stateInteraction();

    // step2:滤波
    void updateState();
    void updateState(std::vector<SystemModel>& system_model, std::vector<PositionModel>& measurement_model, const Eigen::VectorXd& z, const double& dt);

    // step3:模型概率更新
    void updateModelProb();

    // step4:估计融合
    void estimateFusion();
};