# 此包用于存放全局参数配置文件
## 1.预测点弹道补偿
- 考虑到预测坐标值与当前坐标值存在一定偏差，时间提前量仅根据当前坐标值计算会有一定误差。因此可以尝试对预测坐标值和时间提前量进行迭代优化，当前坐标值计算出的时间提前量作为迭代初值。

## 2.config目录文件参数配置说明
- autoaim.yaml 
    -
    /armor_detector

|参数名        |   释义        |  配置说明    |
| ---         |   ---        |    ---      |
|color          |  敌方装甲板颜色            |             |
|max_delta_t                    |   追踪器更新的时间间隔上限，大于该阈值则删除此tracker           |根据程序运行帧率修改             |
|max_lost_cnt                   |装甲板丢失的上限，超出则重置ROI             |             |
|max_armors_cnt                 |  视野中装甲板数量上限            |根据赛场情况更改             |
|hero_danger_zone          | 英雄危险距离，若视野中存在英雄机器人，且在此距离内，则优先选为追踪目标             |             |
|armor_type_wh_thres            |装甲板的宽高比阈值，判定大小装甲板  |根据大小装甲板的实际宽高比进行调整         |
|no_crop_thres                  | 禁用ROI的阈值             |             |
|no_crop_ratio          | 禁用ROI的比率             |             |
|full_crop_ratio          | 裁剪ROI的比率            |             |
|armor_roi_expand_ratio_width          |   装甲板区域扩大的比率（宽）           |用于roi区域内目标连续跟踪             |
|armor_roi_expand_ratio_height          | 装甲板区域扩大的比率（高）             |用于roi区域内目标连续跟踪             |
|armor_conf_high_thres          | 装甲板置信度高阈值（双阈值判定）             |根据环境情况修改             |
|shoot_delay          | 发弹延迟（从上位机发出指令，到弹丸打出枪管的时间间隔，估计值）             |根据不同车辆调整（电控粗测，手动整定），`自瞄开预测前得改！！！`             |
|max_dead_buffer           | 缓存的灰色装甲板数量上限，超出即认定目标死亡（此参数主要针对弹丸击中装甲板后出现的短暂熄灭情况）             |根据程序运行帧率修改             |
|max_delta_dist            |同一装甲板的距离跳变阈值             |  测试经验整定           |
|switch_max_dt           |装甲板切换的最大时间间隔              | 测试效果整定            |
|anti_spin_max_r_multiple         |              |             |
|anti_spin_judge_low_thres           |  装甲板跳变的低阈值            |             |
|anti_spin_judge_high_thres          | 装甲板跳变的高阈值             |             |
|delta_x_3d_high_thresh          |  目标水平方向跳变的高阈值（双阈值）            |  测试效果整定           |
|delta_x_3d_higher_thresh           | 目标水平方向跳变的更高阈值（双阈值）             |   测试效果整定          |
|delta_x_3d_low_thresh          | 目标水平方向跳变的低阈值（双阈值）             |   测试效果整定          |
|delta_x_3d_lower_thresh          | 目标水平方向跳变的更低阈值（双阈值）             |  测试效果整定           |
|delta_y_3d_high_thresh          |   同上（距离方向）         | 测试效果整定            |
|delta_y_3d_higher_thresh        |   同上           |测试效果整定             |
|delta_y_3d_low_thresh           |    同上          |测试效果整定             |
|delta_y_3d_lower_thresh           | 同上             | 测试效果整定            |
|camera_name           |  相机型号，诸如"KE0200110074"（大恒）/"00J90630561"（海康）            |`跑程序前得改！！！` |
|camera_type          |  相机类型，用于区分不同相机，(daheng: 0 / hik: 1 / mvs: 2 / usb: 3)            |`跑程序前得改！！！`   |
|camera_param_path            | 相机配置参数路径             |             |
|network_path           | 网络权重路径             |             |
|save_data            |  是否保存数据            |             |
|save_path          |      数据保存路径        |             |
|save_dataset         |     是否保存数据集         |             |
|debug         |        是否开启调试模式，此模式会发布一些调试需要的功能和消息      |`调试程序前得改！！！`             |
|use_serial         | 是否使用串口，用于装甲板的三维坐标转换到云台坐标系下。             |  若为true，串口驱动节点需能正常工作并发布陀螺仪数据；false则不使用陀螺仪数据。`调试程序前得改！！！`           |
|use_roi          | 是否开启ROI             | 车辆间装甲板不断发生切换可以开启            |
|detect_red          | 是否检测红色            |             |
|show_img        |    是否显示图像          |`调试程序前得改！！！`             |
|show_fps        |    是否显示帧率          |调试可视化             |
|show_aim_cross         |   是否画出十字线           | 调试可视化            |
|show_all_armors       |     是否框出识别到的所有装甲板         | 调试可视化，`调试时得改！！！`            |
|print_letency      |    是否打印耗时信息（包含网络推理耗时/数据处理耗时/节点运行总延时等）          |调试时输出             |
|sync_transport        | 是否开启同步接收功能（用于同步图像和IMU数据）            |此种策略耗时较高，目前还是异步接收消息             |
|print_target_info       |是否打印目标信息              | 调试时输出            | \


-  autoaim.yaml  
    -  
    - armor_processor 

|参数名        |   释义        |  配置说明    |
| ---         |   ---        |    ---      |
|bullet_speed   |初始默认弹速  |电控反馈（可以取弹速波动的中间值），`打弹前得改！！！`  |
|max_dt | 最大时间跨度，大于该值则重置预测器           |             |
|max_v                          |   装甲板速度上限           |判断装甲板是否发生切换             |
|max_cost            | 非线性优化的误差上限             | 拟合效果整定            |
|min_fitting_lens           |非线性优化的最少观测数据量              | 测试效果整定            |
|shoot_delay                  |发弹延迟（从上位机发出指令，到弹丸打出枪管的时间间隔，估计值）             |根据不同车辆调整（电控粗测，手动整定），`自瞄开预测前得改！！！`             |
|max_offset_value          |          |             |
|reserve_factor         |储备系数，用于判断卡尔曼滤波是否发散              |根据预测效果整定             |
|debug         |        是否开启调试模式，此模式会发布一些调试需要的功能和消息      |`调试程序前得改！！！`             |
|use_serial         | 是否使用串口，用于装甲板的三维坐标转换到云台坐标系下。             |  若为true，串口驱动节点需能正常工作并发布陀螺仪数据；false则不使用陀螺仪数据。`调试程序前得改！！！`           |
|show_img        |    是否显示图像          |`调试程序前得改！！！`             |
|draw_predict      |画出滤波预测的速度曲线           | 调试可视化            |
|show_predict          |画出预测轨迹点              |调试可视化          |
|show_aim_cross            |在图像上画出十字线      |调试可视化             |
|show_fps          |预测帧率显示   | 调试可视化           |
|print_delay           |打印程序耗时       | 调试输出          |
|sync_transport        | 是否开启同步接收功能（用于同步图像和IMU数据）            |此种策略耗时较高，目前还是异步接收消息             |
|show_marker            | marker可视化消息发布，rviz2或foxglove中可视化三维预测点 |`三维可视化调试时得打开!!!`     |
|camera_type          |  相机类型，用于区分不同相机，(daheng: 0 / hik: 1 / mvs: 2 / usb: 3)            |`跑程序前得改！！！`   |
|camera_name           |相机型号，诸如"KE0200110074"（大恒）/"00J90630561"（海康）|`跑程序前得改！！！` |
|camera_param_path  |相机参数路径，包含相机内参以及相机和IMU联合标定结果              | `相机或IMU重新标定后得改！！！`            |
|filter_param_path           | 滤波参数路径       |             |
|uniform_ekf_process_noise_param          |整车模型过程噪声参数          | 预测效果整定，`调试自瞄预测时可改`            |
|uniform_ekf_measure_noise_param          |整车模型测量噪声参数           | 预测效果整定，`调试自瞄预测时可改`   |
|singer_model_process_param           |singer模型过程噪声参数          |预测效果整定，`调试自瞄预测时可改`   |
|singer_model_measure_param          |singer模型测量噪声参数          |预测效果整定，`调试自瞄预测时可改`    |
|trans_prob_matrix          |IMM模型状态转移矩阵      |预测效果整定       |
|model_prob_vector          |各模型概率向量     | 预测效果整定     |
|process_noise        | IMM模型过程噪声参数    | 预测效果整定   |
|measure_noise           | IMM模型测量噪声参数        | 预测效果整定       | \

## 3.launch目录文件参数配置说明
- autoaim_bringup.launch.py
    -
    - launch文件声明参数配置

| 参数名               |释义               |配置说明          |
|---                  | ---              |   --- |
|camera_param_file    |相机参数文件路径     |       |
|autoaim_param_file   | 自瞄参数文件路径    |       |
|camera_type          | 相机类型（hik/daheng/mvs/usb）           | `运行此launch文件前得修改！！！`      |
|use_serial            | 是否使用串口   | 若为false，serialport节点不会启动，即无串口调试，`调试时得改！！！`      |
|debug_pred           | （条件运行）自瞄预测节点和串口通信节点的同进程启动   | 若为false，自瞄预测节点独立进程运行；若为true，串口通信节点和自瞄预测节点会被加载进同一个进程，在container中运行      |

    - 节点参数配置（serialport节点）
| 参数名 | 释义       | 配置说明         |
|---    | ---       | ---             |
|using_port   | 是否使用串口       |若为false，串口节点运行时不发布下位机发布的消息，`无串口调试时得开！！！`     | 
|tracking_target   | 是否关闭预测       |`自瞄调试时得改！！！`      | 
|print_serial_info   | 是否打印串口消息       |调试时输出      | 
|print_referee_info   | 是否打印裁判系统消息       | 调试时输出     | 
    
