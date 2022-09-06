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

#ifndef RMOSS_POWER_RUNE_2019_RUNE_STATE_MACHINE_H
#define RMOSS_POWER_RUNE_2019_RUNE_STATE_MACHINE_H

#include <string>
#include "opencv2/opencv.hpp"
#include "rmoss_power_rune2019/rune_types.hpp"

namespace rmoss_power_rune2019
{
enum class FmsResultCode {shoot, wait0, wait1, ignore};
typedef struct
{
  int state = 0;           //表示当前亮灯个数1-5,0表示初始状态
  cv::Point2f point[5];
  float err[5];
  float err_thre = 60;
  float err_num = 0;
  int errmax_id = 0;
  float err_sum = 0;
  int target_err_min_id = 0;
} RuneState;

// 能量机关识别算法部分，状态机。
class RuneStateMachine
{
public:
  RuneStateMachine() {}
  ~RuneStateMachine() {}

public:
  FmsResultCode update(std::vector<ArmorDescriptor> & armors);
  void clear();
  ArmorDescriptor getTargetArmor();

private:
  int calcNewState(
    std::vector<ArmorDescriptor> & armors, RuneState & old_state,
    RuneState & new_state);

private:
  RuneState state_;
  ArmorDescriptor target_armor_;
};
} // namespace rmoss_power_rune2019

#endif //RMOSS_POWER_RUNE_2019_RUNE_STATE_MACHINE_H
