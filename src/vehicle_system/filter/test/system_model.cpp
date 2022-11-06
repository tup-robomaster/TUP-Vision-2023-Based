 /*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-03 22:59:40
 * @LastEditTime: 2022-11-06 15:02:55
 * @FilePath: /filter/test/system_model.cpp
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
        
        x_.v_x() = x.v_x();
        x_.v_y() = x.v_y();
        x_.x() = x.x() + x.v_x() * t;
        x_.y() = x.y() + x.v_y() * t;

        return x_;
    }

protected:
    void updateJacobians(const S& x, const C& u, const int& t)
    {
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
    {
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
    {
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