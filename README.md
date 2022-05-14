# Home Service Robot

This project uses the turtlebot3 ros packages and a custom environment of a home robot service that will be able to localie, map and navigate and environment. There are other functionalities like teleoperating the robot or specifying a pickup and dropoff for the robot to autonomously navigate to. 

ROS Packages in the project:
* turtlebot3_slam: To localize and build a map of the environment. This packages uses the ROS node "slam_gmapping" to create a 2D occupancy frid map using laser-based SLAM. To execute this node you can run ./test_slam.sh under scripts.

* turtlebot3_teleop: To execute this node you can run ./test_slam.sh. This node can be used to control the robot using keyboard commands.

* turtlebot3_simulations: This node will be executed using any of the executable scripts. It will bringup and deploy the turtlebot3 model.

* turtlebot3_navigation: This package is used to navigate through the environment given a map of the world. The node communicates with the ROS Navigation stack and sends goals for the robot to reach. The ROS navigation stack uses Dijkstra's algorithm to find a path for the robot to reach goals while avoiding obstacle.

* pickup_objects: This node uses the turtlebot3_navigation package that uses the AMCL node to locatize the robot in a known map. AMCL uses a Monte Carlo probabilistic localization system and takes in laser-based maps, laser scans, and particle filters to successfully localize the robot and reach goal poses. 

Note: turtlebot3 submodules need to be poperly set

These are the instructions to install the packages necessary to run the project: 

1. Install ROS noetic
```
sudo apt-get update && sudo apt-get install ros-noetic-desktop-full
apt-get install ros-noetic-dwa-local-planner ros-noetic-move-base
```

2. Build
```
mkdir -p ~/catkin_ws/src/ && cd ~/catkin_ws/src/
git clone https://github.com/mgmendoza/HomeServiceRobot.git
cd ../
catkin_make
```
3. To execute
```
chmod a+x src/scripts/home_service.sh
```
4. To launch
```
./home_service.sh
```



