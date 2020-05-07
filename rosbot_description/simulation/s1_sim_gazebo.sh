#!/bin/bash

sudo killall gzserver
sudo killall gzclient
sudo killall rviz
sudo killall roscore
sudo killall rosmaster

roslaunch rosbot_description simulation_rosbot_world.launch