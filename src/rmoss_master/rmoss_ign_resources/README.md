# rmoss_ign_resources

rmoss_ign_resources为[rmoss_ign](https://github.com/robomaster-oss/rmoss_ign)项目中的RoboMaster Ignition Simulator提供公共基本模型资源，包括以下3种类型：

* 裁判系统模块组件：装甲板模块，枪口测速模块，指示灯模块等
* 标准射击弹丸：17mm荧光弹，42mm荧光弹
* 标准机器人：RoboMaster University AI Challenge2019标准步兵机器人模型

采用[xmacro](https://github.com/gezp/xmacro)方式定义了宏模块，可以方便的被复用。在机器人SDF模型模块化构建中，遵循`插件分离原则`，即对于SDF模型，将3D模型部分与控制器（也就是插件）不耦合在一起，这样可以更换使用自己的控制器，使用更加灵活。

## 裁判系统组件

RoboMaster2021赛季裁判系统

* rm21_armor_module（装甲板模块）：`small_armor`, `large_armor`, `small_armor_with_support`, `large_armor_with_support`
* rm21_light_indicator_module（灯条指示模块）：`light_indicator` 
* rm21_speed_monitor_module（枪口测速模块）：`speed_monitor_17mm`, `speed_monitor_42mm`.

* rm21_video_transmitter_module（图传模块）：`video_transmitter`.

* rm21_rfid_interaction_module（RFID模块）：暂不考虑，需要等待Ignition Gazebo的支持。

RoboMaster2022赛季裁判系统

* rm22_light_indicator_module, rm22_speed_monitor_module, rm21_video_transmitter_module 无变化。
* rm22_armor_module新增RMUA专用贴纸。

> 对于RM2022赛季，裁判系统变化不大，但由于考虑到命名一致性，应使用`rm22_*`系列模块，使用方式不发生变化，即`xmacro_block`的name保持不变。

## rmua19_standard_robot模型

RoboMaster University AI Challenge 2019 标准步兵机器人

 * 模型图纸来自官方[RoboMaster产品/ 机器人 / AI 机器人](https://www.robomaster.com/zh-CN/products/components/detail/1839)资料，使用[xmacro](https://github.com/gezp/xmacro) 进行SDF建模
 * 支持麦克拉姆轮地盘，使用[rmoss_ign](https://github.com/robomaster-oss/rmoss_ign)中的[麦克拉姆轮插件](https://github.com/robomaster-oss/rmoss_ign/tree/main/rmoss_ign_plugins/src/mecanum_drive2).
 * 支持云台（pitch,yaw）角度控制，使用Ignition官方插件`JointPositionController`.
 * 具有装甲板灯条发光效果，需使用插件，可测试自瞄等识别算法。

在标准机器人上可以加上自己的相机等传感器构建自己的机器人模型，例如

```xml
<sdf version="1.7">   
	<xmacro_include uri="model://rplidar_a2/model.sdf.xmacro" />
    <xmacro_include uri="model://industrial_camera/model.sdf.xmacro" />
    <xmacro_include uri="model://rmua19_standard_robot/rmua19_standard_robot.def.xmacro" />
	<xmacro_define_value name="global_armor_sticker_num1" value="model://rm21_armor_module/materials/textures/armor_sticker_num1.png" />
    <!--rmua19_standard_robot B model-->
    <model name="rmua19_standard_robot_b">
        <pose>0 0 0.15 0 0 0</pose>
        <xmacro_block name="rmua19_standard_robot" armor_sticker_map_uri="${armor_sticker_map_uri}"/>
        <!--rplidar a2-->
        <xmacro_block name="rplidar_a2" prefix="front_" parent="chassis" pose="0.155 0 0.1 0 0 0"
                update_rate="10" samples="400" visualize="true"/>
        <!--industrial camera-->
        <xmacro_block name="industrial_camera" prefix="front_" parent="gimbal_pitch" pose="0.1 0 0.045 0 0 0"
                            update_rate="30" horizontal_fov="1" width="640" height="480"/>
    </model>
</sdf>
```

* 这里只包含机器人3D模型，不包含控制插件，可自行选择合适的控制插件（如射击插件，麦轮插件，云台控制插件，灯条插件等）。

## 建模指南

采用[xmacro](https://github.com/gezp/xmacro) 宏工具进行SDF`模块化建模`,应该尽可能遵守以下几个原则：

* `插件分离原则` : 使用xmacro进行模块化时，插件应该和模型分离，这样方便使用自定义的插件作为控制器。
* `Collision简洁原则`：在构建机器人<collision>时，尽可能采用简单的几何（如box，cylinder, sphere）代替复杂mesh文件，必要时可以忽略部分collision，减小仿真计算量。

具体建模使用可参考[RMOSS Ign建模指南](https://robomaster-oss.github.io/rmoss_tutorials/#/developer_guides/rmoss_ign_modeling)。

> Tip: 使用SW，blender等3D建模工具将整个机器人分成几个模块（依照可活动关节分离），模块之间使用关节连接，然后再把各个模块导出STL或dea文件，连杆，关节的尺寸，以及之间的相对位姿可在3D建模工具测量得到。

## 版权及维护者

* SDF xml file is provided under Apache License 2.0.
* The copyright of 3D model(.dea,.stl,etc) belongs to the orginal author.

maintanter：Zhenpeng Ge, zhenpeng.ge@qq.com

