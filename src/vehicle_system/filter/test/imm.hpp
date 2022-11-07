/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-06 16:45:33
 * @LastEditTime: 2022-11-08 00:25:13
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
        // std::cout << 1 << std::endl;
        this->W.setIdentity();
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

    using Matrix = Eigen::MatrixXd;
    using Vector = Eigen::VectorXd;

private:
    std::vector<std::shared_ptr<filter::ExtendKalmanFilter<State>>> models_;

    Matrix transfer_prob_;
    Matrix P_;
    Matrix X_;

    Vector c_;
    Vector model_prob_;
    Vector x_;

    size_t model_num_;
    size_t state_num_;

public:
    IMM();
    ~IMM();

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

    // step3:模型概率更新
    void updateModelProb();

    // step4:估计融合
    void estimateFusion();
};