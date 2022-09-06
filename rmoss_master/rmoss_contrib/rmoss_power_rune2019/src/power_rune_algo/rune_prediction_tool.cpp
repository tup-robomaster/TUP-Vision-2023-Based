// Copyright 2020 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmoss_power_rune2019/rune_prediction_tool.hpp"

using namespace rmoss_power_rune2019;
using namespace std;
using namespace cv;

// https://stackoverflow.com/questions/44647239/how-to-fit-a-circle-to-a-set-of-points-with-a-constrained-radius
// fits a circle to these points using linear least-squares
bool RunePredictionTool::fitCircle(
  const std::vector<cv::Point2f> & points, cv::Point2f & centre,
  float & radius)
{
  int cols = 3;
  cv::Mat X(static_cast<int>(points.size()), cols, CV_64F);
  cv::Mat Y(static_cast<int>(points.size()), 1, CV_64F);
  cv::Mat C;
  if (int(points.size()) >= 3) {
    for (size_t i = 0; i < points.size(); i++) {
      X.at<double>(static_cast<int>(i), 0) = 2 * points[i].x;
      X.at<double>(static_cast<int>(i), 1) = 2 * points[i].y;
      X.at<double>(static_cast<int>(i), 2) = -1.0;
      Y.at<double>(
        static_cast<int>(i),
        0) = (points[i].x * points[i].x + points[i].y * points[i].y);
    }
    cv::solve(X, Y, C, cv::DECOMP_SVD);
  }
  std::vector<double> coefs;
  C.col(0).copyTo(coefs);
  centre.x = coefs[0];
  centre.y = coefs[1];
  radius = sqrt(coefs[0] * coefs[0] + coefs[1] * coefs[1] - coefs[2]);
  return true;
}

//2D旋转模型
cv::Point2f RunePredictionTool::rotatePoint2D(cv::Point2f center, cv::Point2f postion, float angle)
{
  cv::Point2f vec_target = postion - center;
  cv::Point2f vec_predict, result;
  //向量逆时针旋转angle
  float cosa, sina;
  sina = sin(angle);
  cosa = cos(angle);
  vec_predict.x = vec_target.x * cosa - vec_target.y * sina;
  vec_predict.y = vec_target.x * sina + vec_target.y * cosa;
  result = center + vec_predict;
  return result;
}
