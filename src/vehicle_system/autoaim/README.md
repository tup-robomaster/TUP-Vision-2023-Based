# `Autoaim System`

* 此系统包含两个节点，分别对应armor_detector和armor_processor。
* armor_detector节点主要检测装甲板并识别装甲板运动状态，并发布目标位置信息；armor_processor节点主要对识别的目标进行预测，并将最终打击位置发布。

## `Autoaim priority`

|   |  | |  |   | Type of Robot |
|--:| -|-|--|---|---|

|Decision Msg|   Robot with manipulator(Ground)|Robot without manipulator(Ground)|Drone(Midair) |
|-------------|------------------------------|---------------------------------|-------------|
|HP      |          |       |           |
|Distance|          |       |           |
|ID      |          |       |           |
 