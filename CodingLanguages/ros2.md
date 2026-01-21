# ROS2 (Ubuntu)

## Starting a ROS2 workspace

- To begin using a ROS2 workspace, run **source /opt/ros/jazzy/setup.bash**

## turtlesim

- To begin running turtlesim, run **ros2 run turtlesim turtlesim_node**
- In a new terminal, run **ros2 run turtlesim turtle_teleop_key**. This is where you will control the turtle

## rqt

- To run rqt, just call **rqt**
- To begin calling actions, go to Plugins > Services > Service Caller
- In the service dropdown menu, there will be many different options you can choose from to perform an action for your turtle
  - /spawn - Spawns a new turtle on the sim. You can give a name and specify the location you'd like it to spawn before calling
  - /<turtlename>/set_pen - Let's you set the pen color of the specified turtle. RGB ranges go from 0-255
- After creating a new turtle  
