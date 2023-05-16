# TUP-Vision-2023-Based
沈阳航空航天大学TUP战队2023赛季视觉基础程序开发仓库

## Version
V3.0

程序基于ros-galactic框架

## 1.Brief
    程序整体架构采用前后端分离的模式，以实现更加高效的模块化编程。自瞄分为检测模块和预测模块，检测节点订阅相机图像并将图像送入网络推理输出目标信息，预测节点则订阅目标信息并进行跟踪预测。

功能模块流程图：
>[autoaim](src/vehicle_system/autoaim/docs/Autoaim.jpg) \
>[sentry_autoaim](src/vehicle_system/autoaim/docs/sentry_autoaim.jpg) \
>[buff](src/vehicle_system/buff/docs/Buff.jpg)

## 2.Introduction
- Packages

|    Name   |      Function  | Description |
|    ---    |        ---     |    ---         |
|global_user|     定义全局接口 |包含通用函数/结构体/枚举类型|
|global_interface|自定义消息   |全局msg定义|
|serialport|      串口驱动     |定义与下位机的通信协议|
|camera_driver|相机驱动        | 包含大恒/海康/USB相机驱动接口|
|armor_detector| 装甲板检测节点 | 识别出待打击目标|
|armor_processor| 装甲板预测节点|对待打击目标进行预测|
|buff_detector| 能量机关检测节点|识别出待打击扇叶|
|buff_processor|能量机关预测节点| 对待打击扇叶进行预测|

- Custom messages

| Message           |          Topic            |   Description |
|  ---              |          ---              |      ---      |
| Autoaim.msg       | /armor_detector/armor_msg | 目标装甲板消息 |
| Buff.msg          | /buff_detector/buff_msg   | 目标能量机关消息 |
| CarPos.msg        | /car_pos |  车辆位置消息    |
| Decision.msg      | /robot_decision/decision  | 决策消息 | 
| DetectorArray.msg | /armor_detector/detections| 目标消息 |
| GameInfo.msg      | /game_info                |  比赛消息（下位机从裁判系统读入）|
| Gimbal.msg        | /armor_processor/gimbal_msg  /armor_processor/tracking_msg /buff_processor/gimbal_msg /buff_processor/tracking_msg | 云台消息（自瞄控制消息）|
| ObjHP.msg         |  /obj_hp                  | 目标血量消息（从裁判系统读入） |
| Serial.msg        | /serial_msg               |   串口消息|

## 3.Environment

|Library | URL |  Description |
| ---    | --- | ---          |
|OpenVINO| https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html| 目前使用的版本是OpenVINO2022.1，直接offline下载一键安装即可，需要给权限 |
|glog| https://github.com/google/glog/releases/tag/v0.5.0|
yaml-cpp| https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.5.3  | 需指定将yaml-cpp库编译成共享库
Ceres   | http://ceres-solver.org/installation.html | Click  "latest stable release"
Eigen   | https://gitlab.com/libeigen/eigen/-/releases/3.4.0       | 编译此库前首先对ceres库进行编译
OpenCV  |https://github.com/opencv/opencv/tree/4.2.0 \ https://github.com/opencv/opencv_contrib/tree/4.x | 编译时两个包放一块，注意编译时需传入指定参数 |
|海康相机库 | https://www.hikrobotics.com/cn/machinevision/service/download?module=0 | 编译安装时给权限
|大恒相机库 | https://www.daheng-imaging.com/index.php?m=content&c=index&a=lists&catid=59&czxt=9&sylx=21&syxj=44#mmd | 编译安装时给权限

## 4.Debug
- Prepare
  - 
  - 参数配置文件位置：src/global_user/config/autoaim.yaml(参数配置详解见:[Autoaim_Config](src/global_user/README.md)  )
  - 对应的launch文件位置：src/global_user/launch/autoaim_bringup.launch.py
  - 首先根据实际情况更改相机类型（camera_type）和型号(camera_name)（包括armor_detector空间和armor_processor空间下对应的参数），调试视频则把camera_type赋为3；
  - 与下位机通信调试时将配置文件中的using_imu参数改为true，同时把launch文件中的using_imu参数置为True；
  - 调试的参数主要是CS模型(singer_model)和IMM模型对应的参数(trans_prob_matrix\model_prob_vector\process_noise\measure_noise)。
- Compile
  - 
    
        colcon build --symlink-install 
        . install/setup.bash

- Command
  - 
        
        ros2 launch global_user autoaim_bringup.launch.py
- Startup
  - 

  - Step1:设置shell脚本权限
        
        sudo chmod 777 your_shell_script_path
  - Step2:配置程序启动首选项

        gnome-terminal -- your_shell_script_path
- Log
  -  
  - 日志保存路径设置

        sudo gedit ~/.bashrc
        export ROS_HOME=~/ros_logs
#### 2.能量机关调试
- Prepare
  - 
    
    - 参数配置文件位置：src/global_user/config/buff.yaml
    - 对应的launch文件位置：src/global_user/launch/buff_bringup.launch.py
    - 首先根据实际情况更改相机类型（camera_type）和型号(camera_name)（包括buff_detector空间和buff_processor空间下对应的参数），调试视频则把camera_type赋为3，同时更改launch文件中的相机类型；
    - 与下位机通信调试时将配置文件中的using_imu参数改为true，同时把launch文件中的using_imu参数置为True；
    - 调试时可以适当修改时间延迟量（delay_small和delay_big）。

- Compile
  - 
    
        colcon build --symlink-install 
        . install/setup.bash

- Command
  - 

        ros2 launch global_user buff_bringup.launch.py

## 5.Develop log

| Date |  Issue   |   Debug    |
| ---  |  ---     | ---   | 
|2023-04-24|添加新网络模型，区分大小装甲板。|
|2023-04-14|相机驱动节点加入ros2bag录制图像数据和回放数据功能|比赛前须打开此功能。|
|2023-03-xx| Fixed image transport delay problem.(0.5ms左右)| 将图像订阅者的qos的depth设置为1。|
|2022-12-11| Update buff node.| |
|2022-12-01| Merge armor_detector node and armor_processor node to one node.||
|2022-11-11| 添加ros2 parameter callback，配合rqt_reconfigure插件可以实现节点动态调参(目前仅可在usb相机驱动节点使用)。 |全局均可使用，普通数据类型可以直接修改，目前不支持对数组容器的动态修改。|
|2022-11-06| Construct system models and filter algorithms template class.
|2022-11-05| Fixed serialport node bugs and armor processor node bugs.
|2022-10-28| ~~加入tf2，serialport包将陀螺仪数据加入到tf树，world->gyro(+ transformation == world->camera)~~|     Delete    |
|2022-10-23| 加入2022.1版本的OpenVINO，并在armor_detector节点调试成功，在7代i7NUC上测试网络推理部分耗时在15ms～30ms左右。（10～15ms）|在步兵和英雄10代i5的nuc上调试程序总延时在6~8ms。|
|2022-10-20| 调试发现网络推理部分加载模型出现异常，ros下加载.xml文件后初始化失败。
|2022-10-05| 完成相机驱动功能包的开发，包括相机驱动和相机节点两部分，针对大恒、海康和usb相机。|迈德威视的驱动暂未添加，不可使用。|

## 6.Supplement
| Issue    |  Debug  |  Add   |
| ---            |   ---   | ----    |
|串口权限永久解决|    1) whoami --查看用户名 2) sudo usermod -aG dialout username|
|程序运行出现WARNING:selected interface "lo" is not multicast-capable: disabling multicast / ERROR:Failed to find a free participant index for domain 0| FIXED:创建一个脚本/etc/network/if-up.d/ros2-lo-multicast：#!/bin/sh ip link set lo multicast on|  |
Error:recvUC: malformed packet received from vendor 1.16 state parse| Issue:https://github.com/ros2/ros2/issues/1163|
|Error:编译时出现死机情况| 加入参数--parallel-workers threads_num(>=1)|

## 7.弹丸命中率测试

| Date       | Description |  ShootCount  |   HitNum   |   TP   |
| ---        | ---         |  ---   |  --- | --- |
|2023-04-06  | 步兵射击机动状态下的英雄 | 55 | 50  | 90.1%