/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-21 16:24:35
 * @LastEditTime: 2022-12-21 19:04:03
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/inference/inference_api2.hpp
 */
#ifndef INFERENCE_API2_HPP_
#define INFERENCE_API2_HPP_

//C++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <openvino/openvino.hpp>
// #include <format_reader_ptr.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <fftw3.h>
#include <fmt/color.h>

namespace buff_detector
{
    struct BuffObject
    {
        Point2f apex[5];
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    class BuffDetector
    {
    public:
        BuffDetector();
        ~BuffDetector();
        
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
        
        Eigen::Matrix<float,3,3> transfrom_matrix;
    };
} // namespace buff_detector

#endif