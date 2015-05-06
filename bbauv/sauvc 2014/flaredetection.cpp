/*
 * flaredetection.cpp
 *
 *  Created on: 24 Jan, 2014
 *      Author: ndt
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <termios.h>
#include <signal.h>
#include <cmath>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "bbauv_msgs/compass_data.h"
#include "bbauv_msgs/controller.h"

#include "flarestates.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

const int DEPTH_POINT = 1.1;
ros::Publisher movementPub;
static int yellow_params[6] = {10, 0, 0, 79, 148, 255};

//Utility function
double radianToDegree(double degree) {
	return degree / M_PI * 180;
}

FlareDetection::FlareDetection(): it(nh), private_nh("~"), ac("LocomotionServer", true) {
	enabled = false;

	private_nh.param<int>("loopHz", loopRateHz, 20);
	string imageTopic; private_nh.param<std::string>("image", imageTopic, "/bottomcam/camera/image_rect_color");
	string compassTopic; private_nh.param<std::string>("compass", compassTopic, "/compass");

 	imageSub = it.subscribe(imageTopic, 1, &FlareDetection::imageCallback, this);
    compassSub = nh.subscribe(compassTopic, 1, &FlareDetection::compassCallback, this);
	movementPub = nh.advertise<bbauv_msgs::controller>("/movement", 1);

	areaThresh = 3000;
	rectData.detected = false;
	screen.width = 640;
	screen.height = 480;

	//Initialise parameters
	//lowerH, lowerS, lowerV, higherH, higherS, higherV
	lowerH = yellow_params[0];
	lowerS = yellow_params[1];
	lowerV = yellow_params[2];
	higherH = yellow_params[3];
	higherS = yellow_params[4];
	higherV = yellow_params[5];

	namedWindow("output");
}

FlareDetection::~FlareDetection() {
	cv::destroyWindow("output");
}

void FlareDetection::start() {
	state = boost::shared_ptr<State> (new LookForFlareState(this));
	enabled = true;
}

void FlareDetection::stop() {
	state = boost::shared_ptr<State>(new SurfaceState(rectData.heading));
	enabled = false;
}

void FlareDetection::compassCallback(const bbauv_msgs::compass_data& msg) {
	rectData.heading = msg.yaw;
}

void FlareDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}catch(cv_bridge::Exception& e){
		cv_bridge::CvImagePtr cv_ptr;
	}

	prepareFlareParams(cv_ptr->image);
}

void FlareDetection::prepareFlareParams(Mat image) {
	//Ignore blue stuffs
	Mat channels[3];
	split(image, channels);
	channels[0] = Mat(channels[0].rows, channels[0].cols, channels[0].type(), Scalar::all(0));
	Mat red_image;
	merge(channels, 3, red_image);

	Mat out = image.clone();
	resize(out, out, Size(640,480));

	//Thresholding
	cv::cvtColor(image, red_image, CV_BGR2HSV);
	cv::inRange(image, cv::Scalar(lowerH,lowerS,lowerV), cv::Scalar(higherH,higherS,higherV), red_image);
	cv::Mat erodeEl = cv::getStructuringElement(MORPH_RECT, cv::Size(9, 9));
	cv::Mat dilateEl = cv::getStructuringElement(MORPH_RECT, cv::Point(7, 7));
	cv::erode(red_image, red_image, erodeEl, Point(-1, -1), 1);
	cv::dilate(red_image, red_image, dilateEl, Point(-1, -1), 1);

	//Find x-center
	cv::vector< cv::vector<Point> > contours;
	cv::vector<cv::Vec4i> hierachy;

	findContours(red_image, contours, hierachy,
				 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	double max_area = 0;
	double areaThresh = 3000;
	Point2f center_max;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		if (area > areaThresh && area > max_area) {
			//Find the center using moments
			Moments mu;
			mu = moments(contours[i], false);
			double mu_area = mu.m00;
			center_max.x = mu.m10/mu_area;
			center_max.y = mu.m01/mu_area;
			max_area = area;

			//Find the bounding rect
			rectData.maxRect = minAreaRect(contours[i]);
		}
	}

	if (max_area > 0) {
		rectData.detected = true;
		rectData.center = center_max;

		//Find the  heading
		Point2f points[4];
		rectData.maxRect.points(points);
		Point2f edge1 = points[1] - points[0];
		Point2f edge2 = points[2] - points[1];
		//Choose the verticle edge
		if (norm(edge1) > norm(edge2)) {
			rectData.angle = radianToDegree(atan(edge1.x/edge1.y));
		} else {
			rectData.angle = radianToDegree(atan(edge2.x/edge2.y));
		}
		//Chose angle to turn if horizontal
		if (rectData.angle == 90) {
			if (rectData.center.x > (screen.width / 2)) {
				rectData.angle = -90;
			}
		} else if (rectData.angle == -90) {
			if (rectData.center.x < (screen.width) / 2) {
				rectData.angle = 90;
			}
		}

		//Testing
		circle(red_image, rectData.center, 50, Scalar(255, 255, 255));
		for (int i = 0; i < 4; i++) {
			Point2i pt1(int(points[i].x), int(points[i].y));
			Point2i pt2(int(points[(i+1)%4].x), int(points[(i+1)%4].y));
			cv::line(red_image, pt1, pt2, Scalar(255, 255, 255));
		}
		imshow("output", out);
		waitKey(5);
	}
	if (enabled){
		state = state->gotFrame(image, rectData);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flare_detection");

	FlareDetection flareDetector;
	flareDetector.start();
	ROS_INFO("Initialised Flare Detection...");

	ros::Rate loop_rate(flareDetector.loopRateHz);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

  	ros::spin();
	return 0;
}



