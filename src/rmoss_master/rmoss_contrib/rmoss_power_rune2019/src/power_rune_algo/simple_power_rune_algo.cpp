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
#include "rmoss_power_rune2019/simple_power_rune_algo.hpp"
#include "rmoss_util/debug.hpp"
#include "rmoss_util/image_utils.hpp"

using namespace std;
using namespace cv;
using namespace rmoss_power_rune2019;

SimplePowerRuneAlgo::SimplePowerRuneAlgo(
  std::shared_ptr<rmoss_util::MonoMeasureTool> mono_location_tool)
{
  //装甲板参数
  float realWidth, realHeight, half_x, half_y;
  realHeight = 15.1;
  realWidth = 26;
  half_x = realWidth / 2;
  half_y = realHeight / 2;
  //按坐标系顺序取
  realpoints_.emplace_back(half_x, half_y, 0);
  realpoints_.emplace_back(-half_x, half_y, 0);
  realpoints_.emplace_back(-half_x, -half_y, 0);
  realpoints_.emplace_back(half_x, -half_y, 0);
  is_reshoot_ = false;
  roi_ = Rect(0, 0, 1280, 512);
  roi_offset_ = Point2f(0, 0);
  //
  mono_location_tool_ = mono_location_tool;
  rune_state_machine_.clear();
  points_are_enough_ = false;
}

SimplePowerRuneAlgo::~SimplePowerRuneAlgo() {}

int SimplePowerRuneAlgo::process4large(const cv::Mat & img)
{
  Mat roi_img = img(roi_);
  //识别
  int ret = rune_detector_.process(roi_img);
  if (ret != 0) {
    return -1;     //未识别到，忽略跳过
  }
  std::vector<ArmorDescriptor> armors = rune_detector_.getAllArmors();
  //store center point of detected armors and fit circle
  if (!points_are_enough_) {
    for (size_t i = 0; i < armors.size(); i++) {
      stored_points_.push_back(armors[i].center);
    }
    if (stored_points_.size() > 50) {
      if (RunePredictionTool::fitCircle(stored_points_, rune_center_, rune_radius_)) {
        points_are_enough_ = true;
      } else {
        //re-store and re-fit if it's failed to fit circle
        stored_points_.clear();
      }
    }
  }
  //debug info
  for (size_t i = 0; i < armors.size(); i++) {
    RMOSS_DEBUG(rmoss_util::draw_4points(roi_img, armors[i].points));
  }
  RMOSS_DEBUG(imshow("result", roi_img));
  // update FMS of power rune
  auto code = rune_state_machine_.update(armors);
  if (code == FmsResultCode::wait0) {
    if (is_reshoot_) {
      code = FmsResultCode::shoot;
    }
  }
  if ((code != FmsResultCode::shoot) || (!points_are_enough_)) {
    //状态机不正常状态,或存储的点不够导致未拟合出圆
    return -2;
  }
  //目标装甲板
  ArmorDescriptor armor = rune_state_machine_.getTargetArmor();
  //预测2d (旋转模型)
  Point2f pred_point = RunePredictionTool::rotatePoint2D(
    rune_center_, armor.center,
    30 * CV_PI / 180);
  //3D target (get 3d point by using PnP,then pred 3d point by using 2d rotation model and tri)
  Point3f position;
  vector<Point2f> target_points;
  for (int i = 0; i < 4; i++) {
    target_points.push_back(armor.points[i] + roi_offset_);
  }
  mono_location_tool_->solve_pnp(target_points, realpoints_, position);
  //project pred_point to 3D
  target_ = mono_location_tool_->unproject(pred_point + roi_offset_, position.z);
  is_reshoot_ = false;
  //debug info
  rune_detector_.printArmorDescriptor(armor);
  printf(
    "target point:(%f,%f,%f)..predict point(%f,%f,%f)\n", position.x, position.y, position.z,
    target_.x, target_.y, target_.z);
  //for prediction circle
  RMOSS_DEBUG(cv::circle(roi_img, rune_center_, rune_radius_, cv::Scalar(0, 255, 0), 1, 8));
  RMOSS_DEBUG(cv::circle(roi_img, rune_center_, 5, cv::Scalar(0, 255, 0), -1, 8));
  //for target armor
  RMOSS_DEBUG(cv::circle(roi_img, armor.center, 5, rmoss_util::blue, -1, 8));
  RMOSS_DEBUG(rmoss_util::draw_4points(roi_img, armor.points));
  RMOSS_DEBUG(cv::circle(roi_img, pred_point, 5, rmoss_util::red, -1, 8));
  RMOSS_DEBUG(imshow("shoot_result", roi_img));
  RMOSS_DEBUG(waitKey(1));
  return 0;
}

int SimplePowerRuneAlgo::process4small(const cv::Mat & img)
{
  int ret;
  Mat roi_img = img(roi_);
  //识别
  ret = rune_detector_.process(roi_img);
  if (ret != 0) {
    return -1;     //识别原因，不打击
  }
  std::vector<ArmorDescriptor> armors = rune_detector_.getAllArmors();
  //img debug
  for (size_t i = 0; i < armors.size(); i++) {
    RMOSS_DEBUG(rmoss_util::draw_4points(roi_img, armors[i].points));
  }
  RMOSS_DEBUG(imshow("result", roi_img));
  // update FMS of power rune
  auto code = rune_state_machine_.update(armors);
  if (code != FmsResultCode::shoot) {
    //没有发生状态变化，无须打击
    return -2;
  }
  //目标装甲板
  ArmorDescriptor armor = rune_state_machine_.getTargetArmor();
  // PNP解算
  Point3f position;
  vector<Point2f> target_points;
  for (int i = 0; i < 4; i++) {
    target_points.push_back(armor.points[i] + roi_offset_);
  }
  mono_location_tool_->solve_pnp(target_points, realpoints_, position);
  target_ = position;
  is_reshoot_ = false;
  // debug
  // rune_detector_.printArmorDescriptor(armor);
  RMOSS_DEBUG(cv::circle(roi_img, armor.center, 5, rmoss_util::blue, -1, 8));
  RMOSS_DEBUG(rmoss_util::draw_4points(roi_img, armor.points));
  RMOSS_DEBUG(imshow("shoot_result", roi_img));
  RMOSS_DEBUG(waitKey(1));

  return 0;
}

void SimplePowerRuneAlgo::clear4small()
{
  rune_state_machine_.clear();
}

void SimplePowerRuneAlgo::clear4large()
{
  rune_state_machine_.clear();
  stored_points_.clear();
  points_are_enough_ = false;
}

void SimplePowerRuneAlgo::setTargetColor(ArmorColor color)
{
  rune_detector_.setTargetColor(color);
}

cv::Point3f SimplePowerRuneAlgo::getShootTarget()
{
  return target_ / 100;
}

void SimplePowerRuneAlgo::setReShoot() {is_reshoot_ = true;}
