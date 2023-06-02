 /*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 22:59:40
 * @LastEditTime: 2022-11-20 17:01:21
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/system_model.cpp
 */
#include "./system_model.hpp"

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
        double ax = 0.05;
        double ay = 0.01;

        x_.x() = x.x() + x.v_x() * t + ax * pow(t, 2) * 0.5;
        x_.y() = x.y() + x.v_y() * t + ay * pow(t, 2) * 0.5;
        x_.v_x() = x.v_x() + ax * t;
        x_.v_y() = x.v_y() + ay * t;

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

        //匀速运动模型加速度理论为0，实际上速度会发生轻微变化，把加速度作为服从高斯分布的随机扰动白噪声加入
        // double q = 0.3;
        this->W.setZero();
        this->W(S::X, S::X) = 0.5 * pow(t, 2);
        this->W(S::Y, S::Y) = 0.5 * pow(t, 2);
        this->W(S::Vx, S::X) = t;
        this->W(S::Vy, S::Y) = t;
        // this->W(S::X, S::X) = pow(q, 2) * pow(t, 2) * 0.5;
        // this->W(S::Y, S::Y) = pow(q, 2) * pow(t, 2) * 0.5;
        // this->W(S::Vx, S::Vx) = pow(q, 2) * t;
        // this->W(S::Vy, S::Vy) = pow(q, 2) * t;
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
        double ax = 0.02;
        double ay = 0.01;

        x_.a_x() = x.a_x() + ax;
        x_.a_y() = x.a_y() + ay;

        x_.v_x() = x.v_x() + x.a_x() * t + ax * t;
        x_.v_y() = x.v_y() + x.a_y() * t + ay * t;

        x_.x() = x.x() + x.v_x() * t + (1 / 2) * x.a_x() * t * t + 0.5 * ax * pow(t, 2);
        x_.y() = x.x() + x.v_y() * t + (1 / 2) * x.a_y() * t * t + 0.5 * ay * pow(t, 2); 

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

        this->W.SetZero();
        this->W(S::X, S::X) = 0.5 * pow(t, 2);
        this->W(S::Y, S::Y) = 0.5 * pow(t, 2);
        this->W(S::Vx, S::X) = t;
        this->W(S::Vy, S::Y) = t;
        this->W(S::ax, S::X) = 1;
        this->W(S::ay, S::Y) = 1;
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
        double av = 0.01;
        double aw = 0.01;

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
            x_.x() = x.x() + (x.v() / x.w()) * (sin(x.w() * t + x.theta()) - sin(x.theta())) + 0.5 * cos(x.theta()) * pow(t, 2) * av;
            x_.y() = x.y() + (x.v() / x.w()) * (cos(x.theta()) - cos(x.w() * t + x.theta())) + 0.5 * cos(x.theta()) * pow(t, 2) * aw;
            x_.v() = x.v() + av * t;
            x_.theta() = x.theta() + x.w() * t + 0.5 * pow(t, 2) * aw;
            x_.w() = x.w() + t * aw;
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
        // this->W.setIdentity();
        // double q = 0.5; 
        this->W.setZero();
        this->W(S::X, S::X) = 0.5 * cos(x.theta()) * pow(t, 2);
        this->W(S::Y, S::X) = 0.5 * sin(x.theta()) * pow(t, 2);
        this->W(S::V, S::X) = t;
        this->W(S::Theta, S::Y) = 0.5 * pow(t, 2);
        this->W(S::W, S::Y) = t;
    }
};

template<typename T, template<class> class CovarianceBase = filter::Base>
class SingerModel : public filter::LinearSystemModel<SingerModelState<T>, SingerModelControl<T>, CovarianceBase>
{
private:
    typedef SingerModelState<T> S;
    typedef SingerModelControl<T> C;

    T alpha_;
    T a_max_;
    T p_max_;
    T p0_;
    T sigma_;
    T q_;

public:
    SingerModel()
    {
        alpha_ = 5.0;
        a_max_ = 1.0;
        p_max_ = 0.2;
        p0_ = 0.3;
        sigma_ = sqrt((pow(a_max_, 2) * (1 + 4 * p_max_ - p0_)) / 3);
    }
    SingerModel(T alpha, T a_max, T p_max, T p0, T q)
    {
        alpha_ = alpha;
        a_max_ = a_max;
        p_max_ = p_max;
        p0_ = p0;
        q_ = q;
        sigma_ = sqrt((pow(a_max_, 2) * (1 + 4 * p_max_ - p0_)) / 3);
    }

    S f(const S& x, const C& u, const float& dt) const
    {
        // double alpha = 0.01;

        S x_;

        if(alpha_ != 0)
        {
            x_.x() = x.x() + x.v() * dt + ((alpha_ * dt - 1 + exp(-alpha_ * dt)) / pow(alpha_, 2)) * x.a();
            x_.v() = x.v() + ((1 - exp(-alpha_ * dt)) / alpha_) * x.a();
            x_.a() = x.a() * exp(-alpha_ * dt);
        }
        else
        {
            x_.x() = x.x() + x.v() * dt;
            x_.v() = x.v();
            x_.a() = x.a();
        }
        return x_;
    }

public:
    void updateJacobians(const S& x, const C& u, const float& dt)
    {
        // double alpha = 0.01;
        // double a_max = 5;
        // double p_max = 0.5;
        // double p0 = 0.01;
        // double sigma = sqrt((pow(a_max, 2) * (1 + 4 * p_max - p0)) / 3);
        
        this->F.setZero();

        // S x_ = x;
 
        //状态转移矩阵的雅可比矩阵
        if(alpha_ != 0)
        {   
            this->F(S::X, S::X) = 1;
            this->F(S::X, S::V) = dt;
            this->F(S::X, S::A) = ((alpha_ * dt - 1 + exp(-alpha_ * dt)) / pow(alpha_, 2));
            this->F(S::V, S::V) = 1;
            this->F(S::V, S::A) = (1 - exp(-alpha_ * dt)) / alpha_;
            this->F(S::A, S::A) = exp(-alpha_ * dt);
        }
        else
        {
            // this->F(S::X, S::X) = 1;
            // this->F(S::X, S::V) = dt;
            // this->F(S::V, S::V) = 1;
            // this->F(S::A, S::A) = 1;

            this->F(S::X, S::X) = 1;
            this->F(S::X, S::V) = dt;
            this->F(S::X, S::A) = 0.5 * pow(dt, 2);
            this->F(S::V, S::V) = 1;
            this->F(S::V, S::A) = dt;
            this->F(S::A, S::A) = 1;
        }

        this->W.setZero();
        double q[3][3] = {0};
        calProcessNoiseCov(q, dt);
        //过程噪声协方差矩阵
        if(alpha_ != 0)
        {
            this->W(S::X, S::X) = 2 * alpha_ * pow(sigma_, 2) * q[0][0];
            this->W(S::X, S::V) = 2 * alpha_ * pow(sigma_, 2) * q[0][1];
            this->W(S::X, S::A) = 2 * alpha_ * pow(sigma_, 2) * q[0][2];
            this->W(S::V, S::X) = 2 * alpha_ * pow(sigma_, 2) * q[1][0];
            this->W(S::V, S::V) = 2 * alpha_ * pow(sigma_, 2) * q[1][1];
            this->W(S::V, S::A) = 2 * alpha_ * pow(sigma_, 2) * q[1][2];
            this->W(S::A, S::X) = 2 * alpha_ * pow(sigma_, 2) * q[2][0];
            this->W(S::A, S::V) = 2 * alpha_ * pow(sigma_, 2) * q[2][1];
            this->W(S::A, S::A) = 2 * alpha_ * pow(sigma_, 2) * q[2][2];
        }
        else
        {
            // double q_ = 0.05;
            this->W(S::X, S::X) = q_ * q[0][0];
            this->W(S::X, S::V) = q_ * q[0][1];
            this->W(S::X, S::A) = q_ * q[0][2];
            this->W(S::V, S::X) = q_ * q[1][0];
            this->W(S::V, S::V) = q_ * q[1][1];
            this->W(S::V, S::A) = q_ * q[1][2];
            this->W(S::A, S::X) = q_ * q[2][0];
            this->W(S::A, S::V) = q_ * q[2][1];
            this->W(S::A, S::A) = q_ * q[2][2];
            // this->W.setIdentity();
        }
    }

public:
    void calProcessNoiseCov(double (*q)[3], const double& dt)
    {
        if(alpha_ != 0)
        {
            q[0][0] = (1 - exp(-2 * alpha_ * dt) + 2 * alpha_ * dt + ((2 * pow(alpha_, 3) * pow(dt, 3)) / 3) - 2 * pow(alpha_, 2) * pow(dt, 2) - 4 * alpha_ * dt * exp(-alpha_ * dt)) / (2 * pow(alpha_, 5));
            q[0][1] = (exp(-2 * alpha_ * dt) + 1 - 2 * exp(-alpha_ * dt) + 2 * alpha_ * dt * exp(-alpha_ * dt) - 2 * alpha_ * dt + pow(alpha_, 2) * pow(dt, 2)) / (2 * pow(alpha_, 4));
            q[0][2] = (1 - exp(-2 * alpha_ * dt) - 2 * alpha_ * dt * exp(-alpha_ * dt)) / (2 * pow(alpha_, 3));
            q[1][0] = 0;
            q[1][1] = (4 * exp(-alpha_ * dt) - 3 - exp(-2 * alpha_ * dt) + 2 * alpha_ * dt) / (2 * pow(alpha_, 3));
            q[1][2] = (exp(-2 * alpha_ * dt) + 1 - 2 * exp(-alpha_ * dt)) / (2 * pow(alpha_, 2));
            q[2][0] = 0;
            q[2][1] = 0;
            q[2][2] = (1 - exp(-2 * alpha_ * dt)) / (2 * alpha_);
        }
        else
        {
            q[0][0] = pow(dt, 5) * 0.05;
            q[0][1] = pow(dt, 4) * 0.125;
            q[0][2] = pow(dt, 3) / 6;
            q[1][0] = pow(dt, 4) * 0.125;
            q[1][1] = pow(dt, 3) / 3;
            q[1][2] = pow(dt, 2) * 0.5;
            q[2][0] = pow(dt, 3) / 6;
            q[2][1] = pow(dt, 2) * 0.5;
            q[2][2] = dt;
        }
    }

    void setParam(double& alpha, double& a_max, double& p_max, double& p0)
    {
        this->alpha_ = alpha;
        this->a_max_ = a_max;
        this->p_max_ = p_max;
        this->p0_ = p0;
        this->sigma_ = sqrt((pow(a_max_, 2) * (1 + 4 * p_max_ - p0_)) / 3);
    }

    void set_alpha(double& alpha)
    {
        this->alpha_ = alpha;
    }

    void set_a_max(double& a_max)
    {
        this->a_max_ = a_max;
    }

    void set_p_max(double& p_max)
    {
        this->p_max_ = p_max;
    }

    void set_p0(double& p0)
    {
        this->p0_ = p0;
    }

    void set_sigma()
    {
        this->sigma_ = sqrt((pow(a_max_, 2) * (1 + 4 * p_max_ - p0_)) / 3);
    }
};