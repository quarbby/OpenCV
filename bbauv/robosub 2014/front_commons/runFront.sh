#!/bin/bash

# Takes in name of state machine i.e. rgb_buoy 
# Uses default front camera topic 
# If add a True parameter: _alone:=True, else _alone:=False

if [[ $2 == "True" ]];
then
rosrun vision run.py $1.states _image:=front_camera/camera/image_raw _alone:=True
else
rosrun vision run.py $1.states _image:=front_camera/camera/image_raw _alone:=False
fi
