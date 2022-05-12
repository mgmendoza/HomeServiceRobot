Install ROS noetic
sudo apt-get update && sudo apt-get install ros-noetic-desktop-full ros-noetic-dwa-local-planner ros-noetic-move-base


Build
mkdir -p ~/catkin_ws/src/ && cd ~/catkin_ws/src/
git clone https://github.com/mgmendoza/HomeServiceRobot.git
cd ../
catkin_make


To execute
chmod a+x src/scripts/home_service.sh

To launch
./home_service.sh



ROS Packages in the project:
turtlbot3_slam: To localize and build a map of the environment.

turtlebot3_teleop: To control the turtlebot3

turtlebot3_simulations: To bringup and deploy the turtlebot3 model.

turtlebot3_navigation: To navigate through the environment given a map of the world

note: turtlebot3 submodules need to be poperly set
