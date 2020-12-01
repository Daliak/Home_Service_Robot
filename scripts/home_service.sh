#!/bin/sh
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e  " rosrun rviz rviz -c/rvizConfig/view_navigation.rviz " &
sleep 5
xterm  -e  " roslaunch rosrun add_markers add_markers " &
sleep 5
xterm  -e  " roslaunch rosrun pick_objects pick_objects "
