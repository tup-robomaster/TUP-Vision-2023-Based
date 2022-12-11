# TUP-Vision-2023-Based
沈阳航空航天大学TUP战队2023赛季视觉基础程序开发仓库

## Version
V1.0

程序基于ros-galactic框架

## 开发日志
Date:2022-12-11 Update buff node.

Date:2022-12-01 Merge armor_detector node and armor_processor node to one node.

Date:2022-11-11 添加ros2 parameter callback，配合rqt_reconfigure插件可以实现节点动态调参(目前仅可在usb相机驱动节点使用)。

Date:2022-11-06 Construct system models and filter algorithms template class.

Date:2022-11-05 Fixed serialport node bugs and armor processor node bugs.

Date:2022-10-28 加入tf2，serialport包将陀螺仪数据加入到tf树，world->gyro(+ transformation == world->camera)

Date:2022-10-23 加入2022.1版本的OpenVINO，并在armor_detector节点调试成功，在7代i7NUC上测试网络推理部分耗时在15ms～30ms左右。（10～15ms）

Date:2022-10-20 调试发现网络推理部分加载模型出现异常，ros下加载.xml文件后初始化失败。

Date:2022-10-05 完成相机驱动功能包的开发，包括相机驱动和相机节点两部分，针对大恒、海康和usb相机。
