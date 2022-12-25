/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:47:15
 * @LastEditTime: 2022-12-26 01:27:44
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/inference/inference.h
 */
//c++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <fftw3.h>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace std;
using namespace cv;
using namespace InferenceEngine;

namespace armor_detector
{
    struct ArmorObject
    {
        Point2f apex[4];
        cv::Rect_<float> rect;
        int cls;
        int color;
        int area;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    class ArmorDetector
    {
    public:
        ArmorDetector();
        ~ArmorDetector();
        bool detect(Mat &src,vector<ArmorObject>& objects, int &dw, int &dh, float &rescale_ratio);
        bool initModel(string path);
    private:
        int dw, dh;
        float rescale_ratio;
        Core ie;
        CNNNetwork network;                // 网络
        ExecutableNetwork executable_network;       // 可执行网络
        InferRequest infer_request;      // 推理请求
        MemoryBlob::CPtr moutput;
        string input_name;
        string output_name;
        
        Eigen::Matrix<float,3,3> transfrom_matrix;
    };
} //namespace detector