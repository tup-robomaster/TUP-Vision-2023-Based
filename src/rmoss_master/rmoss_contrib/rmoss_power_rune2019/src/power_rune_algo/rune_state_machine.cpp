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

#include "rmoss_power_rune2019/rune_state_machine.hpp"

using namespace std;
using namespace cv;
using namespace rmoss_power_rune2019;

FmsResultCode RuneStateMachine::update(std::vector<ArmorDescriptor> & armors)
{
  //更新状态
  RuneState new_state;
  if (armors.size() == 1) {    //出现1个点，特殊情况（初始），单独考虑
    calcNewState(armors, state_, new_state);
    if (state_.state == 0) {
      target_armor_ = armors[0];
      state_ = new_state;
      return FmsResultCode::wait0;        //第一次识别目标，等待.
    } else if (state_.state == 1) {
      if (new_state.err_num == 1) {
        //重置state，开始打击
        target_armor_ = armors[0];
        state_ = new_state;
        return FmsResultCode::shoot;
      } else {
        // wait0
        target_armor_ = armors[0];
        state_ = new_state;
        return FmsResultCode::wait0;
      }
    } else if (state_.state == 2) {
      if (new_state.err_num == 0) {
        // ignore
        return FmsResultCode::ignore;
      } else {
        //错误，重置打击
        target_armor_ = armors[0];
        state_ = new_state;
        return FmsResultCode::shoot;
      }
    } else {
      //错误，重置打击
      target_armor_ = armors[0];
      state_ = new_state;
      return FmsResultCode::shoot;
    }
  } else {    //一般情况
    if (int(armors.size()) == state_.state) {
      //激活装甲板未更新
      calcNewState(armors, state_, new_state);
      //更新state
      state_ = new_state;
      if (new_state.err_num == 0) {
        // wait0
        target_armor_ = armors[new_state.target_err_min_id];
        return FmsResultCode::wait0;
      } else {
        // wait1
        return FmsResultCode::wait1;
      }
    } else if (int(armors.size()) == state_.state + 1) {
      //激活装甲板更新
      calcNewState(armors, state_, new_state);
      // check
      if (new_state.err_num == 1) {
        //更新state
        state_ = new_state;
        target_armor_ = armors[new_state.errmax_id];
        return FmsResultCode::shoot;
      } else {
        // ignore
        return FmsResultCode::ignore;
      }
    } else {
      // ignore
      return FmsResultCode::ignore;
    }
  }
}

int RuneStateMachine::calcNewState(
  std::vector<ArmorDescriptor> & armors,
  RuneState & old_state, RuneState & new_state)
{
  int len = armors.size();
  if (len <= 0 || len > 5) {
    return 1;
  }
  //状态
  new_state.state = len;
  new_state.err_num = 0;
  //保存状态点
  for (int i = 0; i < len; i++) {
    new_state.point[i] = armors[i].center;
  }
  //计算每个点的误差,统计误差个数
  float temp, tempmin;
  for (int i = 0; i < len; i++) {
    tempmin = 10000;
    for (int j = 0; j < old_state.state; j++) {
      temp = cv::norm(new_state.point[i] - old_state.point[j]);
      if (temp < tempmin) {
        tempmin = temp;
      }
    }
    new_state.err[i] = tempmin;
    if (new_state.err[i] > new_state.err_thre) {
      //cout << "new_state.err[i]:" << new_state.err[i] << endl;
      new_state.err_num++;
    }
  }
  //选择最大误差index
  float tempmax = 0;
  int id = 0;
  for (int j = 0; j < new_state.state; j++) {
    if (new_state.err[j] > tempmax) {
      tempmax = new_state.err[j];
      id = j;
    }
  }
  new_state.errmax_id = id;
  //选择与当前目标最小的误差点index
  float tempmin2, err_with_target;
  tempmin2 = 1000;
  for (int j = 0; j < new_state.state; j++) {
    err_with_target = cv::norm(new_state.point[j] - target_armor_.center);
    if (err_with_target < tempmin2) {
      tempmin2 = err_with_target;
      id = j;
    }
  }
  new_state.target_err_min_id = id;
  //计算误差和,暂未用
  float sum = 0;
  for (int i = 0; i < len; i++) {
    sum = sum + new_state.err[i];
  }
  new_state.err_sum = sum;
  return 0;
}

void RuneStateMachine::clear() {state_.state = 0;}

ArmorDescriptor RuneStateMachine::getTargetArmor()
{
  return target_armor_;
}
