#pragma once

#include <unistd.h>

#include <iostream>
#include <fstream> 

#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rmoss_util/debug.hpp"
#include "rmoss_util/image_utils.hpp"
#include "rmoss_cam/cam_client.hpp"

#include "debug.h"

using namespace std;
using namespace cv;

//字节数为4的结构体用于偏航校正
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

typedef struct
{
    float2uchar pitch_angle; //偏航角
    float2uchar yaw_angle;   //俯仰角
    float2uchar dis;  //目标距离
    
    int ismiddle;     //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，目前暂不使用，默认置0
    int isFindTarget; //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isSpinning;
    int nearFace;
} VisionData;

template<typename T>
bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
{
    int cnt = 0;
    for(int row = 0;row < matrix.rows();row++)
    {
        for(int col = 0;col < matrix.cols();col++)
        {
            matrix(row,col) = vector[cnt];
            cnt++;
        }
    }
    return true;
}

float calcTriangleArea(cv::Point2f pts[3]);
float calcTetragonArea(cv::Point2f pts[4]);

std::string symbolicToReal(string path);
std::string relativeToFull(string relative,string src);
string treeToPath(std::vector<string> &tree);
string getParent(string path);

std::vector<string> readLines(string file_path);
std::vector<string> generatePathTree(string path);

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2);
Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);
Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);