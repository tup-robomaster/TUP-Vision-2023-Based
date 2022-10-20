# rmoss_contrib

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test (Galactic)](https://github.com/robomaster-oss/rmoss_contrib/actions/workflows/ci.yml/badge.svg)](https://github.com/robomaster-oss/rmoss_contrib/actions/workflows/ci.yml)

![](rmoss_bg.png)
RoboMasterOSS是一个面向RoboMaster的开源软件栈项目，目的是为RoboMaster机器人软件开发提供了一个快速的，灵活的开发工具，支持算法原型研究和robomaster比赛应用开发。

> [RoboMaster竞赛](https://www.robomaster.com/)，全称为`全国大学生机器人大赛RoboMaster机甲大师赛` 。
>
> - 全国大学生机器人[RoboMaster](https://www.robomaster.com/)大赛，是一个涉及“机器视觉”、“嵌入式系统设计”、“机械控制”、“人机交互”等众多机器人相关技术学科的机器人比赛。
> - 在RoboMaster 2019赛季中，参赛队伍需自主研发不同种类和功能的机器人，在指定的比赛场地内进行战术对抗，通过操控机器人发射弹丸攻击敌方机器人和基地。每局比赛7分钟，比赛结束时，基地剩余血量高的一方获得比赛胜利。
>
> 更多详情参考官网：[www.robomaster.com](https://www.robomaster.com/)

rmoss_contrib是RoboMaster OSS中的基础项目，为RoboMaster提供任务级功能模块包，如自动瞄准模块，能量机关模块等


## 1.主要模块

|        模块         |                 功能说明                 |
| :-----------------: | :--------------------------------------: |
|    `rmoss_auto_aim`    |   RoboMaster基础自瞄任务的简单算法实现   |
| `rmoss_power_rune2019` | RoboMaster2019能量机关任务的简单算法实现 |

## 2.使用说明

* 目前仅支持`ROS2 foxy`版本
* 依赖于
  * [rmoss_interfaces](https://github.com/robomaster-oss/rmoss_interfaces)
  * [rmoss_core](https://github.com/robomaster-oss/rmoss_core)

环境配置

```bash
#cd ros2 workspaces src
git clone https://github.com/robomaster-oss/rmoss_interfaces.git
git clone https://github.com/robomaster-oss/rmoss_core.git
git clone https://github.com/robomaster-oss/rmoss_contrib.git
#cd ros2 workspaces
colcon build
```

* 相关功能包使用详见相应package的README.md

## 3.维护者及开源许可证

Maintainer : Zhenpeng Ge,  zhenpeng.ge@qq.com

rmoss_core is provided under Apache License 2.0.

