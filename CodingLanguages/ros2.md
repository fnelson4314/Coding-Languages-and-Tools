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
  - /\<turtlename>/set_pen - Let's you set the pen color of the specified turtle. RGB ranges go from 0-255
- After creating a new turtle, you won't be able to directly control it immediately. Call **ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel**
- You can also start a new turtlesim window with a different name using **ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle**

## Nodes

- Each node in ROS should be responsible for a single, modular purpose. Nodes can send and receive data from other nodes
- The basic syntax for ros2 run is **ros2 run \<package_name> \<executable_name>**
- **ros2 node list** will show you a list of all the running nodes
- You can get info on individual nodes by calling **ros2 node info \<node_name>**
