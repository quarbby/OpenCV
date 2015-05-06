#!/bin/bash

cd 
export ROS_MASTER_URI=http://bbauv:11311
export ROS_IP=$(ipconfig getifaddr en0)
cd ~/Code/bbauv/src/gui/controlpanel/src
ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_lynnette &
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_lynnette &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_lynnette

# roslaunch launch uncompressbags.launch &

rosrun gui vision.py _image:=/front_camera/camera/image_raw_lynnette _filter:=/Vision/image_filter_lynnette &
