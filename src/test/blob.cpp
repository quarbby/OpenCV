/*
 * blobdetection.cpp
 *	BlobDetection with OpenCV
 *  Created on: 20 Jan, 2014
 *      Author: Lynnette
 */
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

IplImage *img;
IplImage *out;

class BlobDetection
{
public:
	BlobDetection();
	~BlobDetection();

	IplImage* detectBlob(IplImage *img);
	IplImage* getThresholdedImage(IplImage *imgHSV);
private:

};

BlobDetection::BlobDetection()
{
	namedWindow("blob-in", CV_WINDOW_AUTOSIZE);
	namedWindow("blob-out", CV_WINDOW_AUTOSIZE);
}

BlobDetection::~BlobDetection()
{
	cv::destroyWindow("blob-in");
	cv::destroyWindow("blob-out");
	cvReleaseImage(&img);
	cvReleaseImage(&out);
}

static BlobDetection* blobdetection;

int main(int argc, char** argv)
{
	BlobDetection local_blobdetection;
	blobdetection = &local_blobdetection;

	img = cvLoadImage("blob.jpg");
	cvShowImage("blob-in",img);
	cvSmooth(img, img, CV_GAUSSIAN, 3, 3); 	//smooth the original image
	IplImage *imgHSV = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
	//cvCvtColor(img, imgHSV, CV_BGR2HSV);  //Change colour format from BGR to HSV
	IplImage *out = blobdetection->getThresholdedImage(imgHSV);
	// cvSmooth(out, out, CV_GAUSSIAN, 3, 3); 

	cvShowImage("blob-out", out);

	cv::waitKey();
	blobdetection->~BlobDetection();

	return (0);
}

//Convert the image 
IplImage* BlobDetection::detectBlob(IplImage *img){
	img = cvCloneImage(img);
	cvSmooth(img, img, CV_GAUSSIAN, 3, 3); 	//smooth the original image
	IplImage *imgHSV = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
	cvCvtColor(img, imgHSV, CV_BGR2HSV);  //Change colour format from BGR to HSV
	IplImage* out = getThresholdedImage(imgHSV);
	return out;
}

//Threshold the HSV image
IplImage* BlobDetection::getThresholdedImage(IplImage *imgHSV){
       IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
       cvInRangeS(imgHSV, cvScalar(170,160,60), cvScalar(180,256,256), imgThresh);
       return imgThresh;
}


