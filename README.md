# Project_ROCR
[![License:MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/viveksood97/Project_ROCR/LICENSE)
[![Build Status](https://app.travis-ci.com/viveksood97/Project_ROCR.svg?branch=main)](https://app.travis-ci.com/viveksood97/Project_ROCR)
[![Coverage Status](https://coveralls.io/repos/github/markosej11/Project_ROCR/badge.svg?branch=main)](https://coveralls.io/github/markosej11/Project_ROCR?branch=main)


Project ROCRs (ROS Operated Collection Robot)

# Author
Markose Jacob - markj11@terpmail.umd.edu (Graduate Student in Robotics at the University of Maryland College Park)

Yash Mandar Kulkarni - ykulkarn@umd.edu (Graduate Student in Robotics at the University of Maryland College Park)

Vivek Sood - vsood@umd.edu (Graduate Student in Robotics at the University of Maryland College Park, Dec 2021)

# License 
MIT License

Copyright (c) 2021 Vivek Sood, Markose Jacob, Yash Mandar Kulkarni

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Overview

ROCR is an autonomous cleanup robot which can be used indoors to pick up objects (toys) and place them at a designated location. The robot can uses SLAM and gmapping to navigate its way indoors. The RGBD camera helps it to identify the bojects it has to pick up and the two arms which using a rack and pinion method picks to the object and the robot autonomously navigates itself to the drop off point.

# Dependencies
The following dependencies are required to run this package:

1. ROS Melodic
2. catkin 
3. Ubuntu 18.04 

# Build Instructions
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/viveksood97/Project_ROCR.git
$ source devel/setup.bash
$ cd ..
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

# Running Instructions
The below command will spawn ROCR bot in gazebo 
```
roslaunch rocr rocr_bot.launch 
```

To move the ROCR using teleop (C++) run the below command
```
rosrun rocr teleop_twist_keyboard 
```

To move the ROCR using te) run the below command
```
rosrun rocr teleop_template.py
```

# Instructions to download dependencies
1. Controller dependencies
```
ros-<distro>-effort-controllers   
```
Example : $ ros-noetic-effort-controllers

```
ros-<distro>-velocity-controllers   
```
Example : $ ros-noetic-velocity-controllers 

# Instructions to run test
```
cd ~/catkin_ws
source ./devel/setup.bash
rostest rocr launch_test.launch    
```


# Sprint notes 
The link to our sprint notes can be found [here](https://docs.google.com/document/d/1bqV_HCkFut4tG7U7UBhUveYOTpLC3wL2CKf_Kpq-iWs/edit)

# API Google spread sheet
We have been following the agile methodology in this project to track all the tasks involved. We have been utilizing the notion platform to achieve this which can be seen in the link below:

[Product Backlog](https://docs.google.com/spreadsheets/d/15Pr1158XUufMSIfwu-V3ih1gd9oGYmvFHCGcRDBxyUs/edit#gid=0)

# Video presntation
The link to our video presentation : [Phase1](https://drive.google.com/drive/u/0/folders/1dvlhAqxTyvUwC-_0GbSXE5IGWAGvmSi3)


