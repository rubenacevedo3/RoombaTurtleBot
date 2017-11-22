# RoombaTurtleBot

## Overview

This ROS package modifies the TurtleBot simulation and implements a simple walker algorithm much like a Roomba robot vacuum cleaner. The robot moves forward until it reaches an obstacle (but not colliding), then it rotates in place until the way ahead is clear, then it moves forward again and repeats. The node walker subscribes to the turtlebot's ~sensor_state topic and publishes to the turtlebot's cmd_vel topic to make this all happen. The whole package uses the Gazebo to visualize the code working. The launch file also allows you to enable/disable rosbag to record a bag file.  

## License

MIT License

Copyright 2017 Ruben Acevedo 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
© 2017 GitHub, Inc.

## Dependencies

* Ubuntu 
* ROS Kinetic
* Catkin
* Gazebo
* Turtlebot Simulation stack
* roscpp package
* std_msgs package
* message_generation package
* sensor_msgs package

## Steps to Build

First, create and build a catkin workspace if you do not have one already

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

To add your catkin workspace into the ROS enviroment you need to source the generated setup file.

```
$ . ~/catkin_ws/devel/setup.bash
```

To download this repository to your catkin workpace do the following steps:

```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/rubenacevedo3/RoombaTurtleBot.git
$ git pull origin master
```

To build any catkin projects found in the src folder use: 
```
# In a catkin workspace
$ catkin_make
```

To install Turtlebot simulation stack type:
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

## Steps to Run 

To run the launch file:
(note this is to run the launch file without enabling rosbag)

(In terminal 1)
```
$ roscore
```

(In terminal 2)
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch roomba_turtle_bot roombaTurtleBot.launch
# press ctrl+C to stop
```

## Steps for recording bag files with the launch file

(In terminal 2)
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch roslaunch roomba_turtle_bot roombaTurtleBot.launch doRosbag:=true
# press ctrl+C to stop recording 
```
## Steps for inspecting the bag file

(In terminal 2)
```
$ cd ~/.ros
$ rosbag info bagfile.bag
```

## Steps for playing back the bag file
(In terminal 2)
```
$ cd ~/.ros
$ rosbag play bagfile.bag
```

## Note: bag file is too short
the bagfile on this repository is very short because my Gazebo would shutdown after 2 seconds of working.

I would get the following errros:

```
gzserver: /build/ogre-1.9-mqY1wq/ogre-1.9-1.9.0+dfsg1/OgreMain/src/OgreRenderSystem.cpp:546: virtual void Ogre::RenderSystem::setDepthBufferFor(Ogre::RenderTarget*): Assertion `bAttached && "A new DepthBuffer for a RenderTarget was created, but after creation" "it says it's incompatible with that RT"' failed.
Aborted (core dumped)
```
```
[gazebo-2] process has died [pid 16412, exit code 134, cmd /opt/ros/kinetic/lib/gazebo_ros/gzserver -e ode /opt/ros/kinetic/share/turtlebot_gazebo/worlds/playground.world __name:=gazebo __log:=/home/viki/.ros/log/6f71970e-cf2a-11e7-8602-000c2920a272/gazebo-2.log].
log file: /home/viki/.ros/log/6f71970e-cf2a-11e7-8602-000c2920a272/gazebo-2*.log
```

## Run cpplint 

Use cpplint to identify potential source code issues that are in conflict with the Google C++ style guide. 

To install and run from terminal:

```
$ sudo apt-get install python-pip
$ sudo pip install cpplint
$ cd ~/catkin_ws/src/RoombaTurtleBot/roomba_turtle_bot
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
```

## Run cppcheck 

Use cppcheck for static code analysis.

To run from terminal:

```
$ cd ~/catkin_ws/src/RoombaTurtleBot/roomba_turtle_bot
$ cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

