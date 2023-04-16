#!/bin/bash
###
 # @Description: This is a ros-based project!
 # @Author: Liu Biao
 # @Date: 2023-01-02 22:20:13
 # @LastEditTime: 2023-04-16 11:45:18
 # @FigtPath: /TUP-Vision-2023-Based/WatchDog.sh
### 
name=TUP-Vision-2023-Based
package=global_user
launch_file=autoaim_bringup.launch.py
sec=2 
cnt=0

cd /home/tup/Desktop/$name/
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch $package $launch_file

while true
do 
    count=`ps -ef | grep $launch_file | grep -v "grep" | wc -1`

    if [ $count -gt 1]; then
        echo "The $launch_file is alive!"
        sleep $sec
    else
        kill $launch_file
        ros2 launch $package $launch_file
        cnt=cnt+1
        sleep $sec
        if [ $cnt -gt 9]; then
            # echo "Reboot!"
            reboot
        fi
    fi
done

