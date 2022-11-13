/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-07 16:34:37
 * @LastEditTime: 2022-11-13 10:21:42
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/imm_system_model.hpp
 */
#include "../include/motion_model/linear_system_model.hpp"
#include "../include/motion_model/system_model.hpp"
#include "../include/extend_kalman_filter.hpp"

template<typename T>
class CVModelState : public filter::Vector<T, 6>
{
    typedef filter::Vector<T, 6> Vector;

    constexpr static size_t X = 0;
    //Y-position
    constexpr static size_t Y = 1;
    //X-velocity
    constexpr static size_t Vx = 2;
    //Y-velocity
    constexpr static size_t Vy = 3;
    constexpr static size_t N0 = 4;
    constexpr static size_t N1 = 5;

public:
    Vector xState() const {return (*this);}

    T x() const {return (*this)[X];}
    T y() const {return (*this)[Y];}
    T v_x() const {return (*this)[Vx];}
    T v_y() const {return (*this)[Vy];}

    Vector& xState() {return (*this);} 

    T& x() {return (*this)[X];}
    T& y() {return (*this)[Y];}
    T& v_x() {return (*this)[Vx];}
    T& v_y() {return (*this)[Vy];}
};

template<typename T>
class CAModelState : public filter::Vector<T, 6>
{
protected:
    typedef filter::Vector<T, 6> Vector;
    
    //vector param index
    constexpr static size_t X = 0;
    constexpr static size_t Y = 1;
    constexpr static size_t Vx = 2;
    constexpr static size_t Vy = 3;
    constexpr static size_t ax = 4;
    constexpr static size_t ay = 5;

public:
    Vector xState() const {return (*this);}
    
    T x() const {return (*this)[X];}
    T y() const {return (*this)[Y];}
    T v_x() const {return (*this)[Vx];}
    T v_y() const {return (*this)[Vy];}
    T a_x() const {return (*this)[ax];}
    T a_y() const {return (*this)[ay];}

    Vector& xState() {return (*this);}
    T& x() {return (*this)[X];}
    T& y() {return (*this)[Y];}
    T& v_x() {return (*this)[Vx];}
    T& v_y() {return (*this)[Vy];}
    T& a_x() {return (*this)[ax];}
    T& a_y() {return (*this)[ay];}
};

template<typename T>
class CTRVModelState : public filter::Vector<T, 6>
{
public:
    typedef filter::Vector<T, 6> Vector;
    
    //vector param index
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t V = 2;     //速度
    static constexpr size_t Theta = 3; //偏航角
    static constexpr size_t W = 4;     //偏航角速度
    static constexpr size_t N0 = 5;

public:
    Vector xState() const {return (*this);}
    T x() const {return (*this)[ X ];}
    T y() const {return (*this)[ Y ];}
    T v() const {return (*this)[ V ];}
    T theta() const {return (*this)[ Theta ];}
    T w() const {return (*this)[ W ];}

    Vector& xState() {return (*this);}
    T& x() {return (*this)[ X ];}
    T& y() {return (*this)[ Y ];}
    T& v() {return (*this)[ V ];}
    T& theta() {return (*this)[ Theta ];}
    T& w() {return (*this)[ W ];}
};

template<typename T>
class ModelState : public filter::Vector<T, 6>
{
protected:
    typedef filter::Vector<T, 6> Vector;

    static constexpr size_t X0 = 0;
    static constexpr size_t X1 = 1;
    static constexpr size_t X2 = 2;
    static constexpr size_t X3 = 3;
    static constexpr size_t X4 = 4;
    static constexpr size_t X5 = 5;

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

template<typename T>
class CVModelControl : public filter::Vector<T, 2>
{
public:
    //vector param index
    constexpr static size_t Vx = 0;
    constexpr static size_t Vy = 1;

    T v_x() const {return (*this)[Vx];}
    T v_y() const {return (*this)[Vy];}
    
    T& v_x() {return (*this)[Vx];}
    T& v_y() {return (*this)[Vy];}
};

template<typename T>
class CAModelControl : public filter::Vector<T, 4>
{
public:
    constexpr static size_t Vx = 0;
    constexpr static size_t Vy = 1;
    constexpr static size_t ax = 2;
    constexpr static size_t ay = 3;

    T v_x() const {return (*this)[Vx];}
    T v_y() const {return (*this)[Vy];}
    T a_x() const {return (*this)[ax];}
    T a_y() const {return (*this)[ay];}

    T& v_x() {return (*this)[Vx];}
    T& v_y() {return (*this)[Vy];}
    T& a_x() {return (*this)[ax];}
    T& a_y() {return (*this)[ay];}
};

template<typename T>
class CTRVModelControl : public filter::Vector<T, 5>
{
public:
    constexpr static size_t V = 0;
    constexpr static size_t Theta = 1;
    constexpr static size_t W = 2;

    T v() const {return (*this)[V];}
    T theta() const {return (*this)[Theta];}
    T w() const {return (*this)[W];}
    
    T& v() {return (*this)[V];}
    T& theta() {return (*this)[Theta];}
    T& w() {return (*this)[W];}
};

template<typename T>
class ModelControl : public filter::Vector<T, 6>
{
    typedef filter::Vector<T, 6> Vector;
    static constexpr size_t X0 = 0;
    static constexpr size_t X1 = 1;
    static constexpr size_t X2 = 2;
    static constexpr size_t X3 = 3;
    static constexpr size_t X4 = 4;
    static constexpr size_t X5 = 5;

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

