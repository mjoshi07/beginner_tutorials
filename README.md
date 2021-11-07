# ROS beginner_tutorials

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This repository contains beginner tutorials in C++ for a publisher and subscriber node in [ROS](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

```
Talker   (src/talker.cpp)   : Publisher node
Listener (src/listener.cpp) : Subscriber node
```

## Dependencies
* ROS Melodic : installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)
* Ubuntu 18.04

## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/mjoshi07/beginner_tutorials
cd ..
catkin_make
```
## Run Instructions
**Method 1, Running the nodes seperately**
* Open a terminal
```
roscore
```
* press Ctrl+Shift+t to open up a new terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker_node
```
* by passing an integer as an argument after talker_node, you can change the frquency of the messages being published, example below publishes messages at 1 Hz
```
rosrun beginner_tutorials talker_node 1
```
* press Ctrl+Shift+t to open up a new terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener_node
```

**Method 2, Running the nodes by the launch file**
* Open a terminal
```
roscd && source setup.bash
```
* The launch file will start both the nodes synchronously
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
* You can also change the frequency of the messages being published by providing the value to "frequency" argument, example below publishes messages at 5 Hz
```
roslaunch beginner_tutorials beginner_tutorials.launch frequency:=5
```

## Service
* You can change the output string being published by the talker
```
rosservice call /custom_string "Learning to change string by service call"
```
## Logging
**Make sure roscore is running**
* Invoke rqt console GUI
```
rqt_console
```
* Invoke rqt logger level GUI
```
rqt_logger_level
```
