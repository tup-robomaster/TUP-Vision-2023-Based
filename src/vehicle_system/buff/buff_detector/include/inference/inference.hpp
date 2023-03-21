/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-10 21:55:36
 * @LastEditTime: 2022-12-27 18:31:34
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/inference/inference.hpp
 */
#ifndef INFERENCE_HPP_
#define INFERENCE_HPP_

#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace std;
using namespace cv;
using namespace InferenceEngine;

using namespace global_user;

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

    class BuffDetector
    {

    private:
        Core ie;
        CNNNetwork network;                // 网络
        ExecutableNetwork executable_network;       // 可执行网络
        InferRequest infer_request;      // 推理请求
        MemoryBlob::CPtr moutput;
        string input_name;
        string output_name;
        
        Eigen::Matrix<float,3,3> transfrom_matrix;

    public:
        BuffDetector();
        ~BuffDetector();

        bool detect(Mat &src,vector<BuffObject>& objects);
        bool initModel(string path);

    };
} //namespace buff_detector

#endif