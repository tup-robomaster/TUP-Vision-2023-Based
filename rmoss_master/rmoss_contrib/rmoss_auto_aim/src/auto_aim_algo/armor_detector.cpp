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

#include "rmoss_auto_aim/armor_detector.hpp"
#include "general.h"

#include <cmath>
#include <vector>

#include "rmoss_util/debug.hpp"
#include "rmoss_util/image_utils.hpp"

using cv::Mat;
using cv::Point2f;

namespace rmoss_auto_aim
{

ArmorDetector::ArmorDetector() {target_is_red_ = true;}
ArmorDetector::~ArmorDetector() {}

int ArmorDetector::process(cv::Mat img)
{
  vector<Object> objects;
  // vector<ArmorDescriptor> armors;
  mArmors.clear();
  cv::Mat input = img.clone();

  detector.initModel(network_path);
  input_size = {416,416};

  //若未检测到目标
  if (!detector.detect(input, objects))
  {
    RMOSS_DEBUG(cv::namedWindow("org_img", 0));
    RMOSS_DEBUG(cv::imshow("org_img", img));
    RMOSS_DEBUG(cv::waitKey(1));
    lost_cnt++;
    is_last_target_exists = false;
    last_target_area = 0;
    cout << "[Armor_detector] 没有识别到装甲板" << endl;
    return false;
  }

  ///------------------------将对象排序，保留面积较大的对象---------------------------------
  sort(objects.begin(),objects.end(),[](Object& prev, Object& next)
                                  {return prev.area > next.area;});
  //若对象较多保留前按面积排序后的前max_armors个
  if (objects.size() > max_armors)
      objects.resize(max_armors);
  ///------------------------生成装甲板对象----------------------------------------------
  for (auto object : objects)
  {
    //  default taget is red
    if (target_is_red_) 
    {
      //若目标颜色为红色或者无色
      if (object.color != 1)
        continue;
    } 
    else 
    {
      //若目标颜色为蓝色或者无色
      if (object.color != 0)
        continue;
    }

      ArmorDescriptor armor;
      armor.id = object.cls;
      armor.color = object.color;
      if (object.color == 0)
          armor.key = "B" + to_string(object.cls);
      if (object.color == 1)
          armor.key = "R" + to_string(object.cls);
      if (object.color == 2)
          armor.key = "N" + to_string(object.cls);
      memcpy(armor.points, object.apex, 4 * sizeof(cv::Point2f));
      for(int i = 0; i < 4; i++)
      {
          armor.points[i] += Point2f((float)roi_offset.x,(float)roi_offset.y);
      }

      Point2f apex_sum;
      for(auto apex : armor.points)
          apex_sum +=apex;
      armor.centerPoint = apex_sum / 4.f;
      mArmors.push_back(armor);
      // printArmorDescriptor(armor);
  }
  ///------------------------绘制装甲板----------------------------------------------
  RMOSS_DEBUG(
  for (auto armor :mArmors)
    {
        if (armor.color == 0)
            putText(img, "B" + to_string(armor.id),armor.points[0],FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0}, 2);
        if (armor.color == 1)
            putText(img, "R" + to_string(armor.id),armor.points[0],FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
        if (armor.color == 2)
            putText(img, "N" + to_string(armor.id),armor.points[0],FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
        for(int i = 0; i < 4; i++)
            line(img, armor.points[i % 4], armor.points[(i + 1) % 4], {0,255,0}, 1);
        // auto armor_center = coordsolver.reproject(armor.center3d_cam);
        // circle(input, armor_center, 4, {0, 0, 255}, 2);
    });
    RMOSS_DEBUG(cv::namedWindow("org_img", 0));
    RMOSS_DEBUG(cv::imshow("org_img", img));
    RMOSS_DEBUG(cv::waitKey(1));
  return 0;
}

std::vector<ArmorDescriptor> ArmorDetector::getArmorVector() {return mArmors;}

void ArmorDetector::printArmorDescriptor(ArmorDescriptor armor)
{
  std::cout << "-----------------------------------" << std::endl;
  std::cout << "装甲板ID" << armor.id << std::endl;
  std::cout << "装甲板Color" << armor.color << std::endl;
  std::cout << "-----------------------------------" << std::endl;
}

void ArmorDetector::set_target_color(bool is_red)
{
  if (target_is_red_ != is_red) {
    target_is_red_ = is_red;
  }
}
}  // namespace rmoss_auto_aim
