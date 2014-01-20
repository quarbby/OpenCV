//Exercise 2: play avi
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc, char** argv){
	cvNamedWindow("Example2", CV_WINDOW_AUTOSIZE);
	CvCapture* capture = cvCreateFileCapture("flame.avi");
	IplImage* frame;
	while (1){
		frame = cvQueryFrame(capture);
		if(!frame) break;
		cvShowImage("Example2", frame);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture(&capture);
	cvDestroyWindow("Example2");
}