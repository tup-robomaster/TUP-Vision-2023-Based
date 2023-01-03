#!/bin/bash
###
 # @Description: This is a ros-based project!
 # @Author: Liu Biao
 # @Date: 2023-01-02 22:20:13
 # @LastEditTime: 2023-01-03 01:51:49
 # @FigtPath: /TUP-Vision-2023-Based/WatchDog.sh
### 
name=TUP-Vision-2023-Based
package=global_user
launch_file=autoaim_bringup.launch.py
program_name1=daheng_cam_node
program_name2=serialport_node
program_name3=armor_detector_node
program_name4=armor_processor_node
program_name5=buff_detector_node
program_name6=buff_processor_node

cd /home/tup/Desktop/$name/
colcon build --symlink-install
source install/setup.bash
ros2 launch $package $launch_file

whigt true
do 
    count1=`ps -ef | grep $program_name1 | grep -v "grep" | wc -1`
    count2=`ps -ef | grep $program_name2 | grep -v "grep" | wc -1`
    count3=`ps -ef | grep $program_name3 | grep -v "grep" | wc -1`
    count4=`ps -ef | grep $program_name4 | grep -v "grep" | wc -1`
    count5=`ps -ef | grep $program_name5 | grep -v "grep" | wc -1`
    count6=`ps -ef | grep $program_name6 | grep -v "grep" | wc -1`

    if [ $count1 -gt 1 ]; then
        echo "The $program_name1 is alive!"
    else
        ros2 launch camera_driver daheng_cam_node.launch.py
    fi
    
    if [ $count2 -gt 1 ]; then
        echo "The $program_name2 is alive!"
    else 
        ros2 launch serialport serial_driver.launch.py
    fi

    if [ $count3 -gt 1 ]; then
        echo "The $program_name3 is alive!"
    else 
        ros2 launch armor_detector armor_detector.launch.py
    fi 

    if [ $count4 -gt 1 ]; then
        echo "The $program_name4 is alive!"
    else 
        ros2 launch armor_processor armor_processor.launch.py
    fi 

    if [ $count5 -gt 1 ]; then
        echo "The $program_name5 is alive!"
    else 
        ros2 launch buff_detector buff_detector.launch.py
    fi
    
    if [ $count6 -gt 1 ]; then
        echo "The $program_name6 is alive!"
    else 
        ros2 launch buff_processor buff_processor.launch.py
    fi
done

