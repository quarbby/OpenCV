#!/bin/bash

ROS_NAMESPACE=/front_camera/camera rosrun image_transport republish compressed in:=image_raw raw out:=image_raw_lynn &
rosrun image_transport republish compressed in:=sonar_image raw out:=sonar_image_lynn &
ROS_NAMESPACE=/Vision rosrun image_transport republish compressed in:=sonar_filter raw out:=sonar_filter_lynn &

if [[ $1 == "True" ]];
then
echo $1
rosrun gui sonarthres.py __name:=Sonar_GUI_lynnette _image:=/front_camera/camera/image_raw_lynn _sonar:=/sonar_image_lynn _testing:=True
else
echo $1
rosrun gui sonarthres.py __name:=Sonar_GUI_lynnette _image:=/front_camera/camera/image_raw_lynn _sonar:=/sonar_image_lynn _filter:=/Vision/sonar_filter_lynn _testing:=False
fi
