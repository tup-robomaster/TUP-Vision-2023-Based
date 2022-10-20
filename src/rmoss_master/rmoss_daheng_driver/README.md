# rmoss_daheng_driver

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test (Galactic)](https://github.com/robomaster-oss/rmoss_daheng_driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/robomaster-oss/rmoss_daheng_driver/actions/workflows/ci.yml)

![](rmoss_bg.png)

## 简介

`rmoss_daheng_driver`提供了适配了`rmoss_cam`的大恒USB3.0工业相机驱动，并基于`rmoss_cam`实现相机ROS节点。

文件说明：

* `daheng_cam.hpp.hpp/cpp` : 大恒相机设备实现。
* `daheng_cam_node.hpp/cpp` :  ROS顶层模块（基于`DaHengCam`和`CamServer`），实现大恒相机节点。

## 依赖

* `rmoss_interfaces`
* `rmoss_cam`
* `OpenCV 4.x`

## 使用方式

> 参照`rmoss_core/rmoss_cam`工具包中定义

launch方式运行：

```bash
ros2 launch rmoss_daheng_driver daheng_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)

colcon build --symlink-install