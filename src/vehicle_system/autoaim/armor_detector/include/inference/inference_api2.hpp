/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-21 16:24:35
 * @LastEditTime: 2023-03-17 18:53:13
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/inference/inference_api2.hpp
 */
#ifndef INFERENCE_API2_HPP_
#define INFERENCE_API2_HPP_

//c++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

//openvino
#include <openvino/openvino.hpp>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Core>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace armor_detector
{
    struct ArmorObject : Object
    {
        int area;
        cv::Point2f apex[4];
    };

    class ArmorDetector
    {
    public:
        ArmorDetector();
        ~ArmorDetector();
        bool detect(cv::Mat &src, std::vector<ArmorObject>& objects);
        bool initModel(std::string path);
    private:
        int dw, dh;
        float rescale_ratio;

        ov::Core core;
        std::shared_ptr<ov::Model> model; // 网络
        ov::CompiledModel compiled_model; // 可执行网络
        ov::InferRequest infer_request;   // 推理请求
        ov::Tensor input_tensor;
        
        std::string input_name;
        std::string output_name;
        
        Eigen::Matrix<float, 3, 3> transfrom_matrix;
    };

} //namespace armor_detector

#endif