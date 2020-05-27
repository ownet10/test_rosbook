#!/bin/bash -xve

#sync and make
rsync -ac ./~/catkin_ws/src/pimouse_ros/
cd ~/catkin_ws
catkin_make
