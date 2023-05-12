#!/bin/bash
###
 # @Description: This is a ros-based project!
 # @Author: Liu Biao
 # @Date: 2023-01-02 22:20:13
 # @LastEditTime: 2023-04-16 01:36:43
 # @FigtPath: /TUP-Vision-2023-Based/WatchDog.sh
### 
name=TUP-Vision-2023-Based
package=global_user
launch_file=autoaim_bringup.launch.py

cd /home/tup/Desktop/$name/
source /opt/openvino/setupvars.sh
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch $package $launch_file

while true
do 
    count=`ps -ef | grep $launch_file | grep -v "grep" | wc -1`

    if [ $count -gt 4]; then
        echo "The $launch_file is alive!"
    else
        kill $launch_file
        ros2 launch $package $launch_file
    fi
    sleep 5
done

