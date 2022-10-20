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
#ifndef RMOSS_POWER_RUNE_2019_RUNE_PREDICTION_TOOL_H
#define RMOSS_POWER_RUNE_2019_RUNE_PREDICTION_TOOL_H

#include <opencv2/opencv.hpp>

namespace rmoss_power_rune2019
{

class RunePredictionTool
{
public:
  static bool fitCircle(
    const std::vector<cv::Point2f> & points, cv::Point2f & centre,
    float & radius);
  static cv::Point2f rotatePoint2D(cv::Point2f center, cv::Point2f postion, float angle);
};

}  // namespace rmoss_power_rune2019

#endif  //RMOSS_POWER_RUNE_2019_RUNE_PREDICTION_TOOL_H
