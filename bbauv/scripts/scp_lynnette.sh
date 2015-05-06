#!/bin/bash
#Takes in one parameter, the whole folder to be put to the vehicle. i.e. rgb_buoy

if [[ $1 == "cfg" ]]; 
then
scp ../cfg/sonar.cfg ../cfg/torpedo.cfg ../cfg/rgb.cfg bbauvsbc1@bbauv:/home/bbauvsbc1/bbauv/src/vision/cfg
else
#rsync -avz torpedo bbauvsbc1@bbauv....
scp $1/vision.py $1/states.py $1/comms.py bbauvsbc1@bbauv:/home/bbauvsbc1/bbauv/src/vision/scripts/$1
fi
