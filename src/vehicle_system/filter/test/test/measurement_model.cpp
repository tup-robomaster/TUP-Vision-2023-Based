/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-04 13:40:21
 * @LastEditTime: 2022-11-19 10:23:56
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/measurement_model.cpp
 */
#include "./system_model.hpp"

template<typename T>
class PositionMeasurement : public filter::Vector<T, 5>
{
public:
    //TODO:
    constexpr static size_t X = 0;
    constexpr static size_t Y = 1;
    constexpr static size_t V = 2;
    constexpr static size_t Theta = 3;
    constexpr static size_t W = 4;

public:
    T x() const {return (*this)[X];}
    T y() const {return (*this)[Y];}
    T v() const {return (*this)[V];}
    T theta() const {return (*this)[Theta];}
    T w() const {return (*this)[W];}

    T& x() {return (*this)[X];}
    T& y() {return (*this)[Y];}
    T& v() {return (*this)[V];}
    T& theta() {return (*this)[Theta];}
    T& w() {return (*this)[W];}
};

template<typename T, template<class> class CovarianceBase = filter::Base>
class PositionMeasurementModel : public filter::LinearMeasurementModel<CTRVModelState<T>, PositionMeasurement<T>, CovarianceBase>
{
public:
    typedef CTRVModelState<T> S;
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

        m_.x() = x.x();
        m_.y() = x.y();
        m_.v() = x.v();
        m_.theta() = x.theta();
        m_.w() = x.w();

        return m_;
    }

protected:
    void updateJacobians(const S& x)
    {
        //
        this->H.setZero();

        //偏导数
        this->H(M::X, S::X) = 1;
        this->H(M::Y, S::Y) = 1;
        this->H(M::V, S::V) = 1;
        this->H(M::Theta, S::Theta) = 1;
        this->H(M::W, S::W) = 1;
    }
};

template<class T>
class SingerPositionMeasurement : public filter::Vector<T, 3>
{
    typedef filter::Vector<T, 3> Vector;

public:
    static constexpr size_t X = 0;
    static constexpr size_t V = 1;
    static constexpr size_t A = 2;

    Vector s() const {return (*this);}
    T x() const {return (*this)[X];}
    T v() const {return (*this)[V];}
    T a() const {return (*this)[A];}

    Vector& s() {return (*this);}
    T& x() {return (*this)[X];}
    T& v() {return (*this)[V];}
    T& a() {return (*this)[A];}
};

template<class T, template<class> class CovarianceBase = filter::Base>
class SingerPositionMeasurementModel : public filter::LinearMeasurementModel<SingerModelState<T>, SingerPositionMeasurement<T>, CovarianceBase>
{
    typedef SingerModelState<T> S;
    typedef SingerPositionMeasurement<T> M;

public:
    SingerPositionMeasurementModel()
    {
        this->H.setIdentity();
        this->V.setIdentity();
    }

    M h(const S& x) const
    {
        M x_;

        x_.x() = x.x();
        x_.v() = x.v();
        x_.a() = x.a();

        return x_;        
    }

    void updateJacobians(const S& x)
    {
        this->H.setZero();

        //
        this->H(M::X, M::X) = 1;
        this->H(M::V, M::V) = 1;
        this->H(M::A, M::A) = 1;
    }
};