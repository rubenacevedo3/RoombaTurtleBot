# RoombaTurtleBot
Overview

This ROS package modifies the TurtleBot simulation and implements a simple walker algorithm much like a Roomba robot vacuum cleaner. The robot moves forward until it reaches an obstacle (but not colliding), then it rotates in place until the way ahead is clear, then it moves forward again and repeats. The node walker subscribes to the turtlebot's ~sensor_state topic and publishes to the turtlebot's cmd_vel topic to make this all happen. The whole package uses the Gazebo to visualize the code working. The launch file also alows you to enable/disable rosbag to record a bag file.  

License

MIT License

Copyright 2017 Ruben Acevedo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. © 2017 GitHub, Inc.

Build/Run Steps

First, create and build a catkin workspace if you do not have one already

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
To add your catkin workspace into the ROS enviroment you need to source the generated setup file.

$ . ~/catkin_ws/devel/setup.bash
To download this repository to your catkin workpace do the following steps:

$ cd ~/catkin_ws/src
$ mkdir week11hw
$ cd week11hw
$ git clone --recursive https://github.com/rubenacevedo3/beginner_tutorials.git
$ git pull origin Week11_HW
To build any catkin projects found in the src folder use:

# In a catkin workspace
$ catkin_make
To run the launch file:

$ roslaunch beginner_tutorials modifyPubFreq.launch
Run cpplint

Use cpplint to identify potential source code issues that are in conflict with the Google C++ style guide.

To install and run from terminal:

$ sudo apt-get install python-pip
$ sudo pip install cpplint
$ cd ~/catkin_ws/src/week10hw/beginner_tutorials
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
Run cppcheck

Use cppcheck for static code analysis.

To run from terminal:

$ cd ~/catkin_ws/src/week10hw/beginner_tutorials
$ cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
