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
#ifndef RMOSS_POWER_RUNE_2019_RUNE_DEF_H
#define RMOSS_POWER_RUNE_2019_RUNE_DEF_H

#include "opencv2/opencv.hpp"

typedef struct _ArmorDescriptor
{
  int id;
  cv::RotatedRect r;
  cv::Point2f points[4];
  //info
  cv::Point2f center;    //中心点
  float armor_hight;    //灯条高度
  float armor_width;    //灯条宽度
  float armor_area;
  float armor_ratio_wh;    //装甲板宽高比，宽/高
} ArmorDescriptor;


#endif //RMOSS_POWER_RUNE_2019_RUNE_DEF_H
