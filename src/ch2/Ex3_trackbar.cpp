//Exercise 3 Trackbar slider
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;

int g_slider_position = 0;
CvCapture* g_capture = NULL;

void onTrackbarSlide(int pos){
	cvSetCaptureProperty(
		g_capture,
		CV_CAP_PROP_POS_FRAMES,
		pos
	);
}

int main(int argc, char** argv){
	cvNamedWindow("Example3", CV_WINDOW_AUTOSIZE);
	g_capture = cvCreateFileCapture("flame.avi");
	int frames = (int) cvGetCaptureProperty(
		g_capture,
		CV_CAP_PROP_FRAME_COUNT
	);
	if (frames != 0){
		cvCreateTrackbar("Position", "Example3", &g_slider_position, frames, onTrackbarSlide);
	}
	IplImage* frame;

	while (1){
		frame = cvQueryFrame(g_capture);
		if (!frame) break;
		cvShowImage("Example3", frame);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture(&g_capture);
	cvDestroyWindow("Example3");

	return (0);
}