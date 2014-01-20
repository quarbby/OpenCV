//Create camera capture
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

int main (int argc, char** argv){
	CvCapture* capture;
	if (argc == 1) {
		capture = cvCreateCameraCapture(0);
	}
	else {
		capture = cvCreateFileCapture (argv[1]);
	}
	assert (capture != NULL);
	IplImage *frame;
	frame = cvQueryFrame(capture);

	cvNamedWindow("Example9");
	cvShowImage("Example9", frame);

	cvWaitKey(0);
	cvDestroyWindow("Example9");
}