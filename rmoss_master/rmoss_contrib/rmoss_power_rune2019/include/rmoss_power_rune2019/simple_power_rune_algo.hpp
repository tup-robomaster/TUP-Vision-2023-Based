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
#ifndef TASK_POWER_RUNE_2019_SIMPLE_POWER_RUNE_ALGO_HPP
#define TASK_POWER_RUNE_2019_SIMPLE_POWER_RUNE_ALGO_HPP

#include <string>

#include "opencv2/opencv.hpp"
#include "rmoss_power_rune2019/rune_detector.hpp"
#include "rmoss_power_rune2019/rune_prediction_tool.hpp"
#include "rmoss_power_rune2019/rune_state_machine.hpp"
#include "rmoss_util/mono_measure_tool.hpp"

namespace rmoss_power_rune2019
{

class SimplePowerRuneAlgo
{
public:
  SimplePowerRuneAlgo(std::shared_ptr<rmoss_util::MonoMeasureTool> mono_location_tool);
  ~SimplePowerRuneAlgo();

public:
  //set target
  void setTargetColor(ArmorColor color);
  //algorithm for small power rune
  int process4small(const cv::Mat & img);
  void clear4small();
  //algorithm for large power rune
  int process4large(const cv::Mat & img);
  void clear4large();
  //shoot
  cv::Point3f getShootTarget();
  void setReShoot();

private:
  int processNoraml(cv::Mat & img);
  int processStaticDebug(cv::Mat & img);

private:
  // tool
  RuneDetector rune_detector_;
  RuneStateMachine rune_state_machine_;
  std::shared_ptr<rmoss_util::MonoMeasureTool> mono_location_tool_;
  // const data
  cv::Rect roi_;
  cv::Point2f roi_offset_;
  std::vector<cv::Point3f> realpoints_;
  // rune circle data
  cv::Point2f rune_center_;
  float rune_radius_;
  std::vector<cv::Point2f> stored_points_;
  bool points_are_enough_;
  // task data
  cv::Point3f target_;
  bool is_reshoot_;
};

}  //namespace rmoss_power_rune2019

#endif  //TASK_POWER_RUNE_2019_SIMPLE_POWER_RUNE_ALGO_HPP
