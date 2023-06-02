/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-05 21:13:18
 * @LastEditTime: 2022-11-06 15:02:30
 * @FilePath: /filter/test/figure.cpp
 */
#include "matplotlibcpp.h"
#include <cmath>

namespace Plt = matplotlibcpp;

int main()
{
    float x0 = 2.5;
    float y0 = 2.5;
    float v = 0.8;
    float w = 3;
    float theta = 0.4;

    std::vector<double> x(60), y(60);
    float xx = x0;
    float yy = y0;
    for(float t = 0; t < 4; t += 0.05)
    {
        xx = xx + (v / w) * (sin(w * t + theta) - sin(theta));
        yy = yy + (v / w) * (cos(theta) - cos(w * t + theta));
        x.push_back(xx);
        y.push_back(yy);
    }

    Plt::figure_size(640, 480);
    Plt::named_plot("CTRV_model", x, y);
    // Plt::named_plot("X_ekf", X_ekf, Y_ekf);
    // Plt::named_plot("X_pred", X_pred, Y_pred);
    Plt::xlim(2.5, 4.0);
    Plt::ylim(2.5, 4.0);
    Plt::title("EKF figure");
    Plt::legend();
    Plt::show();
}
