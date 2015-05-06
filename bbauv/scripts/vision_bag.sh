#!/bin/bash

cd ~/Code/bbauv/src/gui/controlpanel/src
ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_lynnette &
ROS_NAMESPACE=/bot_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_lynnette &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=image_filter raw out:=image_filter_lynnette &

# roslaunch launch uncompressbags.launch &

if [[ $1 == "bot" ]];
then
    rosrun gui vision.py _image:=/bot_camera/camera/image_raw_lynnette _filter:=/Vision/image_filter_lynnette &
else
    rosrun gui vision.py _image:=/front_camera/camera/image_raw_lynnette _filter:=/Vision/image_filter_lynnette &
fi
