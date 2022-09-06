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
#ifndef RMOSS_POWER_RUNE_2019_RUNE_DETECTOR_H
#define RMOSS_POWER_RUNE_2019_RUNE_DETECTOR_H

#include <string>
#include "opencv2/opencv.hpp"
#include "rmoss_power_rune2019/rune_types.hpp"

namespace rmoss_power_rune2019
{

enum class ArmorColor {red, blue};
class RuneDetector
{
public:
  RuneDetector();
  ~RuneDetector();

public:
  void setTargetColor(ArmorColor color);
  int process(cv::Mat & img);
  std::vector<ArmorDescriptor> getAllArmors();
  //调试
  void printArmorDescriptor(ArmorDescriptor & armor);

private:
  // armor
  int preImg(cv::Mat & src, cv::Mat & dst);
  int getArmorDescriptor(cv::RotatedRect r, ArmorDescriptor & armor);
  int armorsDetect(cv::Mat & src_img, cv::Mat & thresh_img);

private:
  bool target_is_red_;
  std::vector<ArmorDescriptor> armors_;
};

} // namespace rmoss_power_rune2019

#endif // RMOSS_POWER_RUNE_2019_RUNE_DETECTOR_H
