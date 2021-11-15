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
## ROS tf frames
The talker node broadcasts a frame /talk with parent /world can be viewed by the following command:
```
rosrun tf tf_echo /world /talk
```
The tf tree can be viewed by the following command:
```
rosrun rqt_tf_tree rqt_tf_tree
```
## Running the rostests
```
catkin_make tests
catkin_make test
```
## Using launch file to run 
* Open a terminal
```
roscd && source setup.bash
```
* The launch file will start both the nodes synchronously and by default it does not record any data
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
* You can enable the rosbag record with "record:=true" at the end, it records for a duration of 15 seconds, and also change the frequency of the messages being published by providing the value to "frequency" argument, example below publishes messages at 5 Hz
```
roslaunch beginner_tutorials beginner_tutorials.launch frequency:=5 record:=true
```
## Inspecting Rosbag data
* In the results directory, there is a file ros_talker_listener.bag which was generated when we ran the launch file with record:=true
* Run the following command to inspect the file:
```
rosbag info ros_talker_listener.bag
```
* It will give some output like this
```
path:        ros_talker_listener.bag
version:     2.0
duration:    15.0s
start:       Nov 14 2021 19:53:53.05 (1636937633.05)
end:         Nov 14 2021 19:54:08.01 (1636937648.01)
size:        191.6 KB
messages:    892
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      148 msgs    : std_msgs/String   
             /rosout       300 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   296 msgs    : rosgraph_msgs/Log 
             /tf           148 msgs    : tf2_msgs/TFMessage

```
* With a roscore and a listener node running, you can play the recorded data to see it publishing
```
rosbag play ros_talker_listener.bag
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
