# rmoss_interfaces

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build (Galactic)](https://github.com/robomaster-oss/rmoss_interfaces/actions/workflows/ci.yml/badge.svg)](https://github.com/robomaster-oss/rmoss_interfaces/actions/workflows/ci.yml)

ROS2 interfaces (.msg, .srv, .action) used in the RMOSS (RoboMaster OSS Projects)

### msg

* Gimbal.msg：云台数据。
* GimbalCmd.msg：云台控制命令。
* ShootCmd.msg：射击命令（后续可能会发生改动）。
* ChassisCmd.msg :底盘控制命令，避免使用（后续可能会发生改动），应该优先使用geometry_msgs/Twist。

### srv

* SetColor.srv：设置颜色（红，蓝），可用于设置自瞄等功能任务。
* GetCameraInfo.srv：获取相机信息，主要使用相机分辨率，相机内参和相机畸变，主要被rmoss_cam使用。
* GetTaskStatus.srv：获取任务节点状态，可用于决策节点，以及心跳机制监控各个模块是否正常运行。
* ControlTask.srv：控制任务节点，启动和停止。
* SetMode.srv/GetMode.srv : 将被弃用，避免使用。
