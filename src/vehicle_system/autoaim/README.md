# `Autoaim System`

* 此系统包含两个节点，分别对应armor_detector和armor_processor。
* armor_detector节点主要检测装甲板并识别装甲板运动状态，并发布目标位置信息；armor_processor节点主要对识别的目标进行预测，并将最终打击位置发布。

## `Autoaim Shooting Criterion`

|   |  | |  |   | Type of Robot |
|--:| -|-|--|---|---|

|Decision Msg|   Robot with manipulator(Ground)|Robot without manipulator(Ground)|Drone(Midair) |
|-------------|------------------------------|---------------------------------|-------------|
|HP      | 低于斩杀线（<= 75）         | 低于斩杀线（<= 100）      |           |
|Distance| 小于射击高命中率范围（<=3.0m）         | 小于开火有效范围（<= 6.0m)  |           |
|ID      |忽略工程/哨兵        |忽略工程       |           |
|Velocity | 小于速度上限       | 小于速度上限           |     |
| Acceleration               |  小于加速度上限     | 小于加速度上限            |        |
|Direction | 运动方向与预测方向一致       | 运动方向与预测方向一致         | 运动方向与预测方向一致           |
|offset         |小于位移偏移上限        | 小于位移偏移上限            | 小于位移偏移上限       |

## `自瞄逻辑`
- 整体流程

    自瞄部分包含4个节点，分别为相机驱动节点、装甲板检测节点、装甲板预测节点以及与下位机通信的串口驱动节点。程序首先从相机驱动节点开始，相机不断采集图像并发布；装甲板检测节点订阅图像并把图像送入网络进行推理识别装甲板目标，然后通过Pnp解算出装甲板三维位姿信息，之后对新增的装甲板进行tracker（追踪器）分配，从新增装甲板中选出打击车辆ID，并从tracker中匹配出ID对应的装甲板，发布目标装甲板信息；装甲板预测节点订阅目标装甲板的消息，并将目标信息输入卡尔曼滤波进行预测输出结果，转换成角度信息后发布；串口驱动节点订阅角度消息，转换数据类型后通过串口将消息发送给下位机，完成一次循环。
- 部分细节
    > 1.相机的采集帧率在200Hz左右，受限于装甲板检测节点的网络推理速度，实际相机发布的帧率不足100帧。调试时相机发布节点的Qos配置的depth应给与适当大小，否则会发生图像缓存区资源抢占，从而出现调试异常情况（例如：接收到的图像不完整、延时较大，甚至部分节点订阅不到图像）；    

