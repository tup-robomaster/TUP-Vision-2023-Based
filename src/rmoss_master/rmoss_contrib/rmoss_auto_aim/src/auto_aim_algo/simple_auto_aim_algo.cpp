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

#include "rmoss_auto_aim/simple_auto_aim_algo.hpp"

#include <vector>

namespace rmoss_auto_aim
{

SimpleAutoAimAlgo::SimpleAutoAimAlgo()
{
  // 初始化装甲板参数
  float realWidth, realHeight, half_x, half_y;
  realHeight = 6.3;
  realWidth = 12.8;
  half_x = realWidth / 2;
  half_y = realHeight / 2;
  mSmallArmorPoints.emplace_back(half_x, half_y, 0);
  mSmallArmorPoints.emplace_back(-half_x, half_y, 0);
  mSmallArmorPoints.emplace_back(-half_x, -half_y, 0);
  mSmallArmorPoints.emplace_back(half_x, -half_y, 0);
  realWidth = 22.2;
  half_x = realWidth / 2;
  mBigArmorPoints.emplace_back(half_x, half_y, 0);
  mBigArmorPoints.emplace_back(-half_x, half_y, 0);
  mBigArmorPoints.emplace_back(-half_x, -half_y, 0);
  mBigArmorPoints.emplace_back(half_x, -half_y, 0);
}

void SimpleAutoAimAlgo::set_camera_info(
  std::vector<double> camera_intrinsic,
  std::vector<double> camera_distortion)
{
  mono_location_tool_.set_camera_info(camera_intrinsic, camera_distortion);
}


int SimpleAutoAimAlgo::process(const cv::Mat & img, float current_pitch)
{
  // step1.图像识别
  int ret;
  ret = armor_detector_.process(img);
  if (ret != 0) {
    return 1;   // 未发现目标
  }
  std::vector<ArmorDescriptor> armor_descriptors = armor_detector_.getArmorVector();
  // step2.位置解算
  std::vector<ArmorTarget> armor_target_vector, bigarmor_target_vector, smallarmor_target_vector;
  // 计算所有装甲板的数据(按大装甲计算),并根据距离哨兵云台的高度区分大小装甲板
  // 暂不区分
  auto imgCenterPoint = cv::Point2f(img.cols / 2, img.rows / 2);
  for (size_t i = 0; i < armor_descriptors.size(); i++) {
    ArmorTarget armor_target;
    armor_target.armorDescriptor = armor_descriptors[i];
    // 定位计算，PnP算法
    std::vector<cv::Point2f> points;
    for (int k = 0; k < 4; k++) {
      points.push_back(armor_descriptors[i].points[k]);
    }
    mono_location_tool_.solve_pnp(points, mSmallArmorPoints, armor_target.postion);
    armor_target.move2dCast =
      cv::norm(armor_descriptors[i].centerPoint - imgCenterPoint);  // 记录图像距离代价
    if (mIsTrack) {
      armor_target.track2dCast =
        cv::norm(armor_descriptors[i].centerPoint - mLastPoint2);   // 记录图像距离代价
      armor_target.track3dCast = cv::norm(armor_target.postion - mLastPoint3);  // 记录图像距离代价
    }
    // filter by height
    float cam_pitch, cam_yaw;
    mono_location_tool_.calc_view_angle(
      armor_target.armorDescriptor.centerPoint, cam_pitch,
      cam_yaw);
    float theta_w = cam_pitch + current_pitch;
    float distance_yz = sqrt(
      armor_target.postion.y * armor_target.postion.y + armor_target.postion.z *
      armor_target.postion.z);
    float height = -distance_yz * sin(theta_w);
    if (height > 10) {
      std::cout << "fillter by height:" << height << ",current_pitch:" << current_pitch <<
        ",theta:" << theta_w << std::endl;
      continue;
    }
    armor_target_vector.push_back(armor_target);
  }

  // step3.选择目标
  float moveMinCast, track2dMinCast, track3dMinCast;
  moveMinCast = track2dMinCast = track3dMinCast = 10000;
  bool findTarget = false;
  // mIsTrack=false;
  if (mIsTrack) {
    // 跟踪状态，选择上一次目标，2d跟踪
    for (size_t i = 0; i < armor_target_vector.size(); i++) {
      if (armor_target_vector[i].track2dCast < track2dMinCast) {
        track2dMinCast = armor_target_vector[i].track2dCast;
        mTarget = armor_target_vector[i];
        findTarget = true;
      }
    }
    if (mTarget.move2dCast < 500) {  // 如果2d移动误差较小时，选择3d跟踪
      for (size_t i = 0; i < armor_target_vector.size(); i++) {
        if (armor_target_vector[i].track3dCast < track3dMinCast) {
          track3dMinCast = armor_target_vector[i].track3dCast;
          mTarget = armor_target_vector[i];
          findTarget = true;
        }
      }
    }
  }
  if (!findTarget) {
    // 非跟踪状态，选择最中间目标，即移动误差最小
    for (size_t i = 0; i < armor_target_vector.size(); i++) {
      if (armor_target_vector[i].move2dCast < moveMinCast) {
        moveMinCast = armor_target_vector[i].move2dCast;
        mTarget = armor_target_vector[i];
        findTarget = true;
      }
    }
    // 跟踪
    mIsTrack = true;
  }
  if (!findTarget) {
    return 2;
  }
  // 记录2d,3d位置数据
  mLastPoint2 = mTarget.armorDescriptor.centerPoint;
  mLastPoint3 = mTarget.postion;
  return 0;
}

ArmorTarget SimpleAutoAimAlgo::getTarget()
{
  return mTarget;
}

void SimpleAutoAimAlgo::setTrack(bool is_track)
{
  mIsTrack = is_track;
}

void SimpleAutoAimAlgo::set_target_color(bool is_red)
{
  armor_detector_.set_target_color(is_red);
}
}  // namespace rmoss_auto_aim
