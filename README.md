# TUP-Vision-2023-Based
沈阳航空航天大学TUP战队2023赛季视觉基础程序开发仓库

## Version
V1.0

程序基于ros-galactic框架

## 开发日志
Date:2023-03-xx Fixed image transport delay problem.(0.5ms左右)

Date:2022-12-11 Update buff node.

Date:2022-12-01 Merge armor_detector node and armor_processor node to one node.

Date:2022-11-11 添加ros2 parameter callback，配合rqt_reconfigure插件可以实现节点动态调参(目前仅可在usb相机驱动节点使用)。

Date:2022-11-06 Construct system models and filter algorithms template class.

Date:2022-11-05 Fixed serialport node bugs and armor processor node bugs.

Date:2022-10-28 加入tf2，serialport包将陀螺仪数据加入到tf树，world->gyro(+ transformation == world->camera)

Date:2022-10-23 加入2022.1版本的OpenVINO，并在armor_detector节点调试成功，在7代i7NUC上测试网络推理部分耗时在15ms～30ms左右。（10～15ms）

Date:2022-10-20 调试发现网络推理部分加载模型出现异常，ros下加载.xml文件后初始化失败。

Date:2022-10-05 完成相机驱动功能包的开发，包括相机驱动和相机节点两部分，针对大恒、海康和usb相机。

## 使用说明
### 1）Env
    OpenVINO:https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html
### 2）Compile
    colcon build --symlink-install
    . install/setup.bash
#### 1.自瞄调试
    调试说明：
    参数配置文件位置：src/global_user/config/autoaim.yaml
    对应的launch文件位置：src/global_user/launch/autoaim_bringup.launch.py
    1.首先根据实际情况更改相机类型（camera_type）和型号(camera_name)（包括armor_detector空间和armor_processor空间下对应的参数），调试视频则把camera_type赋为3；
    2.与下位机通信调试时将配置文件中的using_imu参数改为true，同时把launch文件中的using_imu参数置为True；
    3.调试的参数主要是CS模型(singer_model)和IMM模型对应的参数(trans_prob_matrix\model_prob_vector\process_noise\measure_noise)。

运行命令：

    ros2 launch global_user autoaim_bringup.launch.py

#### 2.能量机关调试
    调试说明：
    参数配置文件位置：src/global_user/config/buff.yaml
    对应的launch文件位置：src/global_user/launch/buff_bringup.launch.py
    1.首先根据实际情况更改相机类型（camera_type）和型号(camera_name)（包括buff_detector空间和buff_processor空间下对应的参数），调试视频则把camera_type赋为3，同时更改launch文件中的相机类型；
    2.与下位机通信调试时将配置文件中的using_imu参数改为true，同时把launch文件中的using_imu参数置为True；
    3.调试时可以适当修改时间延迟量（delay_small和delay_big）。

运行命令：

    ros2 launch global_user buff_bringup.launch.py

## Debug
1.串口权限永久解决：
    
    1) whoami --查看用户名
    2) sudo usermod -aG dialout username

2.程序运行出现WARNING:
    
    selected interface "lo" is not multicast-capable: disabling multicast / ERROR:Failed to find a free participant index for domain 0
FIXED:
    
    创建一个脚本/etc/network/if-up.d/ros2-lo-multicast：
        #!/bin/sh
        ip link set lo multicast on

3.Error:
    
    recvUC: malformed packet received from vendor 1.16 state parse
    
Issue:https://github.com/ros2/ros2/issues/1163
    
4.Error:
    
    编译时出现死机情况加入参数--parallel-workers threads_num(>=1)

## 弹丸命中率测试
### Date:2023-04-06 
    步兵射击机动状态下的英雄，55发中50。