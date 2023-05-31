/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-11-12 17:54:32
 * @LastEditTime: 2022-11-12 20:08:46
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/filter/test/ekf_based_imm.cpp
 */
#include "imm.hpp"

typedef float T;

typedef ModelState<T> State;
typedef CVModel<T> CV;
typedef CAModel<T> CA;
typedef CTRVModel<T> CTRV;

int main(int argc, char** argv)
{
    IMM<T> imm;
    std::shared_ptr<filter::ExtendKalmanFilter<T>> cv = std::shared_ptr<filter::ExtendKalmanFilter<T>>(CV());
    std::shared_ptr<filter::ExtendKalmanFilter<T>> ca = std::shared_ptr<filter::ExtendKalmanFilter<T>>(CA());
    std::shared_ptr<filter::ExtendKalmanFilter<T>> ctrv = std::shared_ptr<filter::ExtendKalmanFilter<T>>(CTRV());
    imm.addCVModel(cv);
    imm.addCAModel(ca);
    imm.addCTRVModel(ctrv);

    return 0;
}