#!/bin/bash
name=TUP-Vision-2023-Based
package=global_user
launch_file=vision_bringup.launch.py
cd /home/tup/Desktop/$name/
colcon build --symlink-install
source install/setup.bash
ros2 launch $package $launch_file
