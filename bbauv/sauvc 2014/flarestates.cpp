/*
 * flarestates.cpp
 *
 *  Created on: 22 Jan, 2014
 *      Author: lynnette & thien
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string.h>

#include "flarestates.h"

//Look for flare state
LookForFlareState::LookForFlareState(FlareDetection *fd){
	ROS_INFO("Looking for yellow flare");
	this->detector = fd;
}

boost::shared_ptr<State> LookForFlareState::gotFrame(cv::Mat image, RectData rectData){
	bbauv_msgs::ControllerGoal msg;
	//detector->publishMovement(msg);
	return shared_from_this();
}

//Lost flare - should keep turning around
LostFlareState::LostFlareState(){
	ROS_INFO("Lost the flare...");
}

boost::shared_ptr<State> LostFlareState::gotFrame(cv::Mat image, RectData rectData){
		return shared_from_this();
}

//Surface State
SurfaceState::SurfaceState(double heading){
	ROS_INFO("Surfacing...");

	bbauv_msgs::controller msg;
	msg.depth_setpoint = 0.2;
	msg.heading_setpoint = heading;
	//publishMovement(msg);
}

boost::shared_ptr<State> SurfaceState::gotFrame(cv::Mat image, RectData rectData){
		return shared_from_this();
}


