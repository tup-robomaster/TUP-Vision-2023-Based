# TUP-Vision-2023-Based
沈阳航空航天大学TUP战队2023赛季视觉基础程序开发仓库

## Version
Bata-0.1
程序基于ros-galactic框架

## 开发日志
Date:2022-10-23 加入2022.1版本的OpenVINO，并在armor_detector节点调试成功，在7代i7NUC上测试网络推理部分耗时在15ms左右。

Date:2022-10-20 调试发现网络推理部分加载模型出现异常，ros下加载.xml文件后初始化失败。

Date:2022-10-05 完成相机驱动功能包的开发，包括相机驱动和相机节点两部分，针对大恒、海康和usb相机。
