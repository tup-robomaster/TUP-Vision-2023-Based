# armor_detector功能包
## 说明：此功能包作为自瞄模块的两个节点之一，完成装甲板识别+小陀螺识别，并发布最终打击的装甲板三维点坐标和其对应的时间戳。
#
# 代码内部逻辑
## include文件夹下inference.h文件是网络推理部分，用于装甲板识别；spinning_detector.hpp文件用于小陀螺识别。 detector.hpp文件则是整个功能的实现，并通过detector_node节点创建topic将最终结果发布。
#
# 调试指南
## 此功能包依赖另外两个自定义功能包global_user和global_interface，调试此模块请先对以上两个功能进行编译。
#
# 调试日志
## date: 2022-10-23 使用OpenVINO API2.0替换旧版本。
问题记录：在NUC上调试时出现全局参数读入失败的情况，但是重启电脑后就解决了




