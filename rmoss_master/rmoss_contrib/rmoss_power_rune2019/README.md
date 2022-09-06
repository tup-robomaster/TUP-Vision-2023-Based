# rmoss_power_rune2019模块

## 1.简介

rmoss_power_rune2019是rmoss_contrib 中的一个功能包，为RoboMaster2019赛季的能量机关任务实现了一个简单的算法，可供参考学习。

主要文件说明
* rune_detection.hpp/cpp : 能量机关的装甲板检测识别模块。
* rune_prediction_tool.h/cpp : 能量机关旋转预测模块，拟合圆，与绕点旋转功能。
* rune_state_machine.h/cpp : 能量机关状态机模块，实现激活（待打击）目标选择。
* simple_power_rune_algo.h/cpp : 能量机关封装算法模块实现（小，大能量机关）。 
* power_rune_node.hpp/cpp : 能量机关任务ROS顶层封装。
* power_rune_main.cpp : `PowerRuneNode` 的main()入口

## 2.快速使用

### 大能量机关测试（视频模拟相机）

> 目前不支持小能量机关

修改launch文件:launch/power_rune.launch

```python
video_path='/home/ubuntu/whole_motion.avi'
```

* 更改视频路径，将video_path改成自己能量机关视频路径
* 能量机关测试视频官方下载：[RoboMaster论坛链接](https://bbs.robomaster.com/thread-7914-1-1.html)：[百度云盘](https://pan.baidu.com/s/1OoIpbzZ2U8Qimadn3zyb6g)

launch启动

```bash
ros2 launch rmoss_power_rune2019 power_rune.launch.py 
```

使用client.py启动自瞄任务(修改配置文件，设置`auto_start`为`false`)

```bash
ros2 run rmoss_power_rune2019 client.py
```
* 按e启动大能量机关，按q暂停

查看相机图形

```bash
ros2 run rqt_image_view rqt_image_view
```

![](doc/imgs/power_rune.png)


debug信息（图像处理中间过程信息）

![](doc/imgs/power_rune_result.png)

若想取消图像debug信息，可以修改配置文件config/power_rune_params.yaml

```yaml
debug : False
```

### Rviz可视化打击点（TODO）

![](doc/imgs/power_rune_rviz.png)

* 可以用于评估识别误差

## 3.能量机关算法说明

* 参考文章：[关于RoboMaster2019能量机关任务](https://www.jianshu.com/p/83b509953198)
* 能量机关状态机模块，实现激活（待打击）目标选择，补充说明文档：[doc/rune_machine_state.md](doc/rune_machine_state.md)
