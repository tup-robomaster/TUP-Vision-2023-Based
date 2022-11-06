/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-04 19:04:11
 * @LastEditTime: 2022-11-06 15:56:41
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/main.cpp
 */
#include <cmath>
#include <iostream>
#include <chrono>

#include "./system_model.cpp"
#include "./measurement_model.cpp"

#include <matplotlibcpp.h>

typedef double T;

//CTRV model
typedef CTRVModelState<T> State;
typedef CTRVModelControl<T> Control;
typedef CTRVModel<T> SystemModel;
typedef PositionMeasurement<T> PosMeasure;
typedef PositionMeasurementModel<T> PosModel;

namespace Plt = matplotlibcpp;

int main(int argc, char** argv)
{
    auto time_start = std::chrono::steady_clock::now();
    //状态向量
    State x;
    x.setZero();
    // x.x() = 2.5;
    // x.y() = 2.5;
    // x.v() = 0.5;
    // x.theta() = 0.5;
    // x.w() = 5;

    //控制向量
    Control u;

    //运动模型
    SystemModel sys_model;

    //观测模型
    PosModel pos_model;

    //随机噪声
    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<T> noise(0, 1);

    //ekf
    filter::ExtendKalmanFilter<State> predictor;
    filter::ExtendKalmanFilter<State> ekf;

    T system_noise = 0.1;
    T pos_noise = 0.025;

    std::vector<double>X_pred(100), Y_pred(100), V_pred(100), Theta_pred(100), W_pred(100);
    std::vector<double>X_ekf(100), Y_ekf(100), V_ekf(100), Theta_ekf(100), W_ekf(100);
    std::vector<double>X_(100), Y_(100), V_(100), Theta_(100), W_(100);
    std::vector<double>Pred_X(100), Pred_Y(100);
    
    //simulate
    float r = 0.25;
    float x0 = 2.5;
    float y0 = 2.5;
    float v = 0.8;
    float theta = 0.4;
    float w = 3.0;

    bool is_init = false;;
    
    float xx = x0;
    float xy = y0;
    x.x() = xx;
    x.y() = xy;
    x.theta() = theta;
    x.v() = v;
    x.w() = w;

    for(float t = 0; t < 5.0; t += 0.05)
    {
        // x.x() = x0 + r * ceres::cos(w * (ii)) + v * (ii) * ceres::cos(theta);
        // x.y() = y0 + r * ceres::sin(w * (ii)) + v * (ii) * ceres::sin(theta);
        // x.x() = x.x() + (v / w) * (ceres::sin(w * ii + theta) - ceres::sin(theta));
        // x.y() = x.y() + (v / w) * (ceres::cos(theta) - ceres::cos(w * ii + theta));
        
        xx = xx + (v / w) * (sin(w * t + theta) - sin(theta));
        xy = xy + (v / w) * (cos(theta) - cos(w * t + theta));
        x.x() = xx;
        x.y() = xy;
        x.theta() = x.theta() + t * w;

        //控制向量初始化
        u.v() = 0;
        u.w() = 0;
        u.theta() = 0;

        if(!is_init)
        {
            predictor.init(x);
            ekf.init(x);
            is_init = true;
        }

        //仿真系统
        auto x_pre = x;
        auto x_model = sys_model.f(x, u, t);

        x.x() += system_noise * noise(generator);
        x.y() += system_noise * noise(generator);
        x.theta() += system_noise * noise(generator);
        
        //预测
        auto x_pred = predictor.predict(sys_model, u, t);
        auto x_ekf = ekf.predict(sys_model, u, t);

        //更新
        PosMeasure pos = pos_model.h(x_pre);
        // // x_pred = predictor.update(pos_model, pos);
        x_ekf = ekf.update(pos_model, pos);

        // std::cout << "Initial value:" << x.x() << "," << x.y() << "," << x.v() << "," << x.theta() << "," << x.w() << std::endl;
        // std::cout << "ekf value:" << x_ekf.x() << "," << x_ekf.y() << "," << x_ekf.v() << "," << x_ekf.theta() << "," << x_ekf.w() << std::endl;
    
        X_pred.push_back(x_model.x());
        Y_pred.push_back(x_model.y());

        X_ekf.push_back(x_ekf.x());
        Y_ekf.push_back(x_ekf.y());

        X_.push_back(xx);
        Y_.push_back(xy);

        Pred_X.push_back(x_pred.x());
        Pred_Y.push_back(x_pred.y());
    }   
    
    // 
    Plt::figure_size(640, 480);
    Plt::named_plot("X_", X_, Y_);
    Plt::named_plot("X_ekf", X_ekf, Y_ekf);
    // Plt::named_plot("X_pred", Pred_X, Pred_Y);
    // Plt::named_plot("X_model", X_pred, Y_pred);
    Plt::xlim(2.50, 4.50);
    Plt::ylim(2.50, 16.00);
    Plt::title("EKF figure");
    Plt::legend();
    Plt::show();

    return 0;
}