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

#ifndef RMOSS_AUTO_AIM__SIMPLE_AUTO_AIM_ALGO_HPP_
#define RMOSS_AUTO_AIM__SIMPLE_AUTO_AIM_ALGO_HPP_
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "rmoss_auto_aim/armor_detector.hpp"
#include "rmoss_util/mono_measure_tool.hpp"

namespace rmoss_auto_aim
{
// 自瞄算法封装。
typedef struct _ArmorTarget
{
  ArmorDescriptor armorDescriptor;
  cv::Point3f postion;
  bool isBigArmor = false;
  float distance = 0;  // 单位cm
  float move2dCast = 0;  // 距离中心点的代价
  float track2dCast = 0;  // 与上次目标的2D距离代价
  float track3dCast = 0;  // 与上次目标的3D距离代价
} ArmorTarget;

class SimpleAutoAimAlgo
{
public:
  SimpleAutoAimAlgo();

public:
  void set_camera_info(std::vector<double> camera_intrinsic, std::vector<double> camera_distortion);
  void set_target_color(bool is_red);
  int process(const cv::Mat & img, float current_pitch);
  ArmorTarget getTarget();
  void setTrack(bool is_track);

private:
  // 工具类
  ArmorDetector armor_detector_;
  rmoss_util::MonoMeasureTool mono_location_tool_;
  // 固定参数
  std::vector<cv::Point3f> mSmallArmorPoints;  // 小装甲板三维点
  std::vector<cv::Point3f> mBigArmorPoints;  // 大装甲板三维点
  // 最终目标
  ArmorTarget mTarget;
  // 其他参数
  bool mIsTrack{false};
  cv::Point2f mLastPoint2;
  cv::Point3f mLastPoint3;
};
}  // namespace rmoss_auto_aim

#endif  // RMOSS_AUTO_AIM__SIMPLE_AUTO_AIM_ALGO_HPP_
