#!/bin/sh
xterm  -e  " source ../../devel/setup.bash ; roslaunch my_robot world.launch" & 
sleep 5
xterm  -e  " source ../../devel/setup.bash; export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_navigation turtlebot3_navigation.launch" &
sleep 5
xterm  -e " source ../../devel/setup.bash; rosrun pick_objects pick_objects"
