#!/bin/bash -xve

#sync and make
rsync -ac ./~/catkin_ws/src/test_rosbook/
cd ~/catkin_ws
catkin_make
