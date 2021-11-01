# ROS beginner_tutorials

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

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
## How to Run
* Open a terminal
```
roscore
```
* press Ctrl+Shift+t to open up a new terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials talker_node
```
* press Ctrl+Shift+t to open up a new terminal
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener_node
```
