
#ifndef SYSTEM_MODEL_HPP_
#define SYSTEM_MODEL_HPP_

#include "../include/extend_kalman_filter.hpp"
#include "../include/motion_model/system_model.hpp"

#include <iostream>
#include <random>
#include <chrono>
#include <ceres/ceres.h>

template<typename T>
class CVModelState : public filter::Vector<T, 4>
{
protected:
    //vector param index
    //X-position
    constexpr static size_t X = 0;
    //Y-position
    constexpr static size_t Y = 1;
    //X-velocity
    constexpr static size_t Vx = 2;
    //Y-velocity
    constexpr static size_t Vy = 3;

public:
    T x() const {return (*this)[X];}
    T y() const {return (*this)[Y];}
    T v_x() const {return (*this)[Vx];}
    T v_y() const {return (*this)[Vy];}

    T& x() {return (*this)[X];}
    T& y() {return (*this)[Y];}
    T& v_x() {return (*this)[Vx];}
    T& v_y() {return (*this)[Vy];}
};

template<typename T>
class CAModelState : public filter::Vector<T, 6>
{
protected:
    //vector param index
    constexpr static size_t X = 0;
    constexpr static size_t Y = 1;
    constexpr static size_t Vx = 2;
    constexpr static size_t Vy = 3;
    constexpr static size_t ax = 4;
    constexpr static size_t ay = 5;

public:
    T x() const {return (*this)[X];}
    T y() const {return (*this)[Y];}
    T v_x() const {return (*this)[Vx];}
    T v_y() const {return (*this)[Vy];}
    T a_x() const {return (*this)[ax];}
    T a_y() const {return (*this)[ay];}

    T& x() {return (*this)[X];}
    T& y() {return (*this)[Y];}
    T& v_x() {return (*this)[Vx];}
    T& v_y() {return (*this)[Vy];}
    T& a_x() {return (*this)[ax];}
    T& a_y() {return (*this)[ay];}
};

template<typename T>
class CTRVModelState : public filter::Vector<T, 5>
{
public:
    //vector param index
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t V = 2;     //速度
    static constexpr size_t Theta = 3; //偏航角
    static constexpr size_t W = 4;     //偏航角速度

public:
    T x() const {return (*this)[ X ];}
    T y() const {return (*this)[ Y ];}
    T v() const {return (*this)[ V ];}
    T theta() const {return (*this)[ Theta ];}
    T w() const {return (*this)[ W ];}

    T& x() {return (*this)[ X ];}
    T& y() {return (*this)[ Y ];}
    T& v() {return (*this)[ V ];}
    T& theta() {return (*this)[ Theta ];}
    T& w() {return (*this)[ W ];}
};

template <class T>
class SingerModelState : public filter::Vector<T, 3>
{   
    // typedef filter::Vector<T, 3> Vector;

public:
    static constexpr size_t X = 0;
    static constexpr size_t V = 1;
    static constexpr size_t A = 2;

    // Vector s() const {return (*this);}
    T x() const {return (*this)[X];}
    T v() const {return (*this)[V];}
    T a() const {return (*this)[A];}

    // Vector& s() {return (*this);}
    T& x() {return (*this)[X];}
    T& v() {return (*this)[V];}
    T& a() {return (*this)[A];}
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
class SingerModelControl : public filter::Vector<T, 3>
{
    // typedef filter::Vector<T, 3> Vector;

public:
    static constexpr size_t X = 0;
    static constexpr size_t V = 1;
    static constexpr size_t A = 2;

    // Vector s() const {return (*this);}
    T x() const {return (*this)[X];}
    T v() const {return (*this)[V];}
    T a() const {return (*this)[A];}

    // Vector& s() {return (*this);}
    T& x() {return (*this)[X];}
    T& v() {return (*this)[V];}
    T& a() {return (*this)[A];}
};

#endif