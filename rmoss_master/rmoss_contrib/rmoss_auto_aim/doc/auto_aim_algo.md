# 自瞄算法

#### sterp1.图像预处理

得到目标颜色的二值图，过滤其他目标。

源码位置：armor_detector.h/preImg()

* RGB通道分离
* 目标颜色二值图

```c++
imgTargetColor = img_r - 0.2 * img_g - 0.8 * img_b;
//然后膨胀操作，二值化
```

* 亮度二值图（目标通道）

```c++
imgBrightness = img_b;
//然后高斯滤波，二值化
```

* 目标颜色二值图与亮度二值图与运算：即颜色符合的发光区域。

```c++
bitwise_and(imgTargetColor, imgBrightness, dst);
```

#### sterp2.灯条识别

源码位置：armor_detector.h/process() , getLightDescriptor()

* 使用findContours()提取轮廓，然后fitEllipse()拟合椭圆
* 根据拟合的椭圆的长宽比，大小，倾斜角度等几何关系过滤，选择符合要求的椭圆作为灯条。

#### sterp3.灯条匹配

源码位置：armor_detector.h/process()，lightsMatch()

* 对于所有灯条，两两遍历匹配，更具几何关系过滤，选择符合要求的灯条对作为装甲板目标

#### sterp4.pnp解算

源码位置：auto_aim_algo.h，调用robot_tool模块中的MonoMeasureTool实现。

* 采用MonoMeasureTool，将装甲板目标矩形转化为3D点（PNP解算）

#### sterp5.云台角度

源码位置：task_auto_aim_example.h，调用projectile_trajectory_tool模块中的GAFTrajectoryTool实现。

* GAFTrajectoryTool为考虑空气阻力和重力的抛物线模型，通过目标3D点，计算云台控制角度pitch,yaw.