#!/bin/sh
xterm  -e  " source ../../devel/setup.bash;  roslaunch my_robot world.launch" & 

sleep 5
xterm  -e  " source ../../devel/setup.bash; export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_slam turtlebot3_gmapping.launch slam_methods:=gmapping" &

sleep 5
xterm  -e  " source ../../devel/setup.bash; export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch " &

sleep 5
xterm  -e  " export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" 
