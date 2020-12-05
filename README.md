# Project 5: Home Service Robot

The fifth project in the Udacity Robotics Software Engineer Nano Degree.

### What does it do ?
The robot will perform Localization, Path Planning and Navigation to simulate a robot picking up and dropping off an object.
Uses a mix of official ROS packages and student created packages:
* ROS official packages:
    * gmapping: used for creating a map and localization.With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.
    * turtlebot_teleop: With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.
    * turtlebot_rviz_launchers: With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.
    * turtlebot_gazebo: With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

* Created Packages:
    * pick_objects: a node that will communicate with the ROS navigation stack and autonomously send successive goals for the robot to reach
    * add_markers: a node that will subscribe to action_type, and based on the action type will add or delete markers.
    * add_markers_time: a node that should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.
                                         

### Getting Started
```
mkdir -p /home/workspace/catkin_ws/src
cd /home/workspace/catkin_ws/src
git clone https://github.com/Daliak/Home_Service_Robot.git . # notice the "." after to just clone the contents
cd /home/workspace/catkin_ws
catkin_make
source devel/setup.bash
cd /scripts
./home_service.sh
```
